#!/usr/bin/env python3
"""
Skeleton and HMD Data Synchronization Script

This script synchronizes skeleton data from Orbbec cameras with HMD data from Unity
using timestamp-based alignment and optional cross-correlation for delay estimation.

Usage:
    python sync_skeleton_hmd.py --skeleton skeleton_data.csv --hmd HMD_Walk_P01.csv --output synced_data.csv

CSV Format Expected:
    skeleton_data.csv: timestamp_ms,device_index,body_id,joint_id,joint_name,pos_x,pos_y,pos_z,...
    HMD_*.csv: timestamp_ms,frame,unity_time,hmd_pos_x,hmd_pos_y,hmd_pos_z,...
"""

import argparse
import pandas as pd
import numpy as np
from scipy import signal
from pathlib import Path


def load_skeleton_data(filepath: str) -> pd.DataFrame:
    """Load skeleton CSV and pivot to wide format (one row per timestamp)."""
    print(f"Loading skeleton data from: {filepath}")
    df = pd.read_csv(filepath)

    # Check required columns
    required = ['timestamp_ms', 'device_index', 'body_id', 'joint_id', 'pos_x', 'pos_y', 'pos_z']
    missing = [col for col in required if col not in df.columns]
    if missing:
        raise ValueError(f"Missing columns in skeleton data: {missing}")

    print(f"  Loaded {len(df)} rows, {df['timestamp_ms'].nunique()} unique timestamps")
    print(f"  Device indices: {sorted(df['device_index'].unique())}")
    print(f"  Body IDs: {sorted(df['body_id'].unique())}")

    return df


def load_hmd_data(filepath: str) -> pd.DataFrame:
    """Load HMD CSV data."""
    print(f"Loading HMD data from: {filepath}")
    df = pd.read_csv(filepath)

    if 'timestamp_ms' not in df.columns:
        raise ValueError("HMD data must have 'timestamp_ms' column")

    print(f"  Loaded {len(df)} rows")
    print(f"  Time range: {df['timestamp_ms'].min()} - {df['timestamp_ms'].max()} ms")

    return df


def estimate_delay_crosscorr(skeleton_df: pd.DataFrame, hmd_df: pd.DataFrame,
                              skeleton_joint: str = 'HEAD', hmd_col: str = 'hmd_pos_y') -> float:
    """
    Estimate time delay between skeleton and HMD data using cross-correlation.

    Uses vertical (Y) position of head joint and HMD as they should correlate well.
    Returns delay in milliseconds (positive = skeleton leads HMD).
    """
    print("\nEstimating delay using cross-correlation...")

    # Get head joint data from skeleton (joint_id 26 = HEAD in K4ABT)
    head_data = skeleton_df[skeleton_df['joint_name'] == skeleton_joint].copy()
    if len(head_data) == 0:
        # Try joint_id 26 (HEAD)
        head_data = skeleton_df[skeleton_df['joint_id'] == 26].copy()

    if len(head_data) == 0:
        print("  Warning: Could not find HEAD joint, using PELVIS (joint_id 0)")
        head_data = skeleton_df[skeleton_df['joint_id'] == 0].copy()

    if len(head_data) == 0:
        print("  Warning: No valid joint data for cross-correlation")
        return 0.0

    # Aggregate by timestamp (average if multiple bodies/devices)
    skeleton_ts = head_data.groupby('timestamp_ms')['pos_y'].mean().reset_index()
    skeleton_ts = skeleton_ts.sort_values('timestamp_ms')

    # Resample to regular intervals (e.g., 10ms)
    resample_interval = 10  # ms

    skel_start = skeleton_ts['timestamp_ms'].min()
    skel_end = skeleton_ts['timestamp_ms'].max()
    hmd_start = hmd_df['timestamp_ms'].min()
    hmd_end = hmd_df['timestamp_ms'].max()

    # Find overlapping time range
    start_time = max(skel_start, hmd_start)
    end_time = min(skel_end, hmd_end)

    if start_time >= end_time:
        print("  Warning: No overlapping time range between skeleton and HMD data")
        return 0.0

    print(f"  Overlapping time range: {start_time} - {end_time} ms ({(end_time - start_time)/1000:.1f} sec)")

    # Create regular time grid
    time_grid = np.arange(start_time, end_time, resample_interval)

    # Interpolate skeleton data
    skel_interp = np.interp(time_grid, skeleton_ts['timestamp_ms'].values, skeleton_ts['pos_y'].values)

    # Interpolate HMD data
    hmd_sorted = hmd_df.sort_values('timestamp_ms')
    hmd_interp = np.interp(time_grid, hmd_sorted['timestamp_ms'].values, hmd_sorted[hmd_col].values)

    # Normalize signals
    skel_norm = (skel_interp - np.mean(skel_interp)) / (np.std(skel_interp) + 1e-8)
    hmd_norm = (hmd_interp - np.mean(hmd_interp)) / (np.std(hmd_interp) + 1e-8)

    # Cross-correlation
    correlation = signal.correlate(skel_norm, hmd_norm, mode='full')
    lags = signal.correlation_lags(len(skel_norm), len(hmd_norm), mode='full')

    # Find peak
    peak_idx = np.argmax(np.abs(correlation))
    delay_samples = lags[peak_idx]
    delay_ms = delay_samples * resample_interval

    print(f"  Estimated delay: {delay_ms:.1f} ms (skeleton {'leads' if delay_ms > 0 else 'lags'} HMD)")
    print(f"  Correlation at peak: {correlation[peak_idx]:.3f}")

    return delay_ms


def synchronize_data(skeleton_df: pd.DataFrame, hmd_df: pd.DataFrame,
                     delay_ms: float = 0.0, tolerance_ms: float = 50.0) -> pd.DataFrame:
    """
    Synchronize skeleton and HMD data by timestamp matching.

    Args:
        skeleton_df: Skeleton data DataFrame
        hmd_df: HMD data DataFrame
        delay_ms: Delay to apply to skeleton timestamps (positive = add to skeleton time)
        tolerance_ms: Maximum time difference for matching

    Returns:
        Synchronized DataFrame with both skeleton and HMD data
    """
    print(f"\nSynchronizing data (delay={delay_ms:.1f}ms, tolerance={tolerance_ms:.1f}ms)...")

    # Apply delay correction to skeleton timestamps
    skeleton_df = skeleton_df.copy()
    skeleton_df['timestamp_ms_corrected'] = skeleton_df['timestamp_ms'] + delay_ms

    # Pivot skeleton data to wide format (one row per timestamp)
    # For simplicity, take first body and first device
    skeleton_wide = skeleton_df.pivot_table(
        index='timestamp_ms_corrected',
        columns='joint_id',
        values=['pos_x', 'pos_y', 'pos_z'],
        aggfunc='first'
    )

    # Flatten column names
    skeleton_wide.columns = [f'skel_j{col[1]}_{col[0].split("_")[1]}' for col in skeleton_wide.columns]
    skeleton_wide = skeleton_wide.reset_index()
    skeleton_wide = skeleton_wide.rename(columns={'timestamp_ms_corrected': 'timestamp_ms'})

    # Merge with HMD data using nearest timestamp matching
    hmd_df = hmd_df.copy()
    hmd_df = hmd_df.sort_values('timestamp_ms')
    skeleton_wide = skeleton_wide.sort_values('timestamp_ms')

    # Use merge_asof for nearest-timestamp matching
    synced = pd.merge_asof(
        hmd_df,
        skeleton_wide,
        on='timestamp_ms',
        tolerance=tolerance_ms,
        direction='nearest'
    )

    # Count matched rows
    skeleton_cols = [col for col in synced.columns if col.startswith('skel_')]
    matched = synced[skeleton_cols[0]].notna().sum() if skeleton_cols else 0

    print(f"  Total HMD frames: {len(hmd_df)}")
    print(f"  Matched frames: {matched} ({100*matched/len(hmd_df):.1f}%)")

    return synced


def main():
    parser = argparse.ArgumentParser(description='Synchronize skeleton and HMD data')
    parser.add_argument('--skeleton', required=True, help='Skeleton CSV file path')
    parser.add_argument('--hmd', required=True, help='HMD CSV file path')
    parser.add_argument('--output', default='synced_data.csv', help='Output CSV file path')
    parser.add_argument('--delay', type=float, default=None,
                       help='Manual delay in ms (if not set, auto-estimate using cross-correlation)')
    parser.add_argument('--tolerance', type=float, default=50.0,
                       help='Timestamp matching tolerance in ms')
    parser.add_argument('--no-crosscorr', action='store_true',
                       help='Skip cross-correlation delay estimation')

    args = parser.parse_args()

    # Load data
    skeleton_df = load_skeleton_data(args.skeleton)
    hmd_df = load_hmd_data(args.hmd)

    # Estimate or use provided delay
    if args.delay is not None:
        delay_ms = args.delay
        print(f"\nUsing manual delay: {delay_ms:.1f} ms")
    elif args.no_crosscorr:
        delay_ms = 0.0
        print("\nSkipping cross-correlation, using delay=0")
    else:
        delay_ms = estimate_delay_crosscorr(skeleton_df, hmd_df)

    # Synchronize
    synced_df = synchronize_data(skeleton_df, hmd_df, delay_ms, args.tolerance)

    # Save
    output_path = Path(args.output)
    synced_df.to_csv(output_path, index=False)
    print(f"\nSaved synchronized data to: {output_path}")
    print(f"  Columns: {len(synced_df.columns)}")
    print(f"  Rows: {len(synced_df)}")


if __name__ == '__main__':
    main()
