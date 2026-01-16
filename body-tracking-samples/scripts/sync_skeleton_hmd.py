#!/usr/bin/env python3
"""
Skeleton and HMD Data Synchronization Script

Synchronizes Orbbec skeleton data with Quest HMD/Controller data using
inter-point distance based cross-correlation for coordinate-system-invariant
delay estimation.

Usage:
    python sync_skeleton_hmd.py --skeleton skeleton.csv --hmd hmd.csv --output synced.csv

Methods:
    1. Inter-point Distance: Uses distances between head and hands (coordinate-invariant)
    2. Triangle Area: Uses area formed by head and two hands
    3. Head Y-only: Traditional single-point method (fallback)
"""

import argparse
import pandas as pd
import numpy as np
from scipy import signal
from scipy.ndimage import gaussian_filter1d
from pathlib import Path
from typing import Tuple, Optional


# =============================================================================
# Joint ID mappings for Azure Kinect Body Tracking SDK
# =============================================================================
JOINT_IDS = {
    'PELVIS': 0,
    'SPINE_NAVEL': 1,
    'SPINE_CHEST': 2,
    'NECK': 3,
    'HEAD': 26,
    'LEFT_SHOULDER': 5,
    'LEFT_ELBOW': 6,
    'LEFT_WRIST': 7,
    'LEFT_HAND': 8,
    'RIGHT_SHOULDER': 12,
    'RIGHT_ELBOW': 13,
    'RIGHT_WRIST': 14,
    'RIGHT_HAND': 15,
}


# =============================================================================
# Data Loading Functions
# =============================================================================

def load_skeleton_csv(filepath: str) -> pd.DataFrame:
    """
    Load skeleton CSV file.

    Supported formats:
        1. multi_device_body_viewer (wide):
           timestamp_ms, device_index, body_id, J0_x, J0_y, J0_z, J0_conf, ...

        2. Unity (wide):
           Frame, Time, P0_posX, P0_posY, P0_posZ, P1_posX, ..., Timestamp

        3. Legacy (long):
           timestamp_ms, device_index, body_id, joint_id, joint_name, pos_x, pos_y, pos_z
    """
    print(f"Loading skeleton data: {filepath}")
    df = pd.read_csv(filepath)

    # Detect format
    if 'J0_x' in df.columns:
        print("  Format: Wide (multi_device_body_viewer)")
    elif 'joint_id' in df.columns:
        print("  Format: Long (one row per joint)")
    elif 'P0_posX' in df.columns or 'P0_posx' in df.columns:
        print("  Format: Wide (Unity)")
    else:
        print(f"  Columns: {list(df.columns)[:10]}...")

    print(f"  Rows: {len(df)}")
    return df


def load_hmd_csv(filepath: str) -> pd.DataFrame:
    """
    Load HMD/Controller CSV file.

    Expected format:
        timestamp_ms, frame, unity_time,
        hmd_pos_x, hmd_pos_y, hmd_pos_z, hmd_rot_x, hmd_rot_y, hmd_rot_z, hmd_rot_w,
        left_pos_x, left_pos_y, left_pos_z, ...
        right_pos_x, right_pos_y, right_pos_z, ...
    """
    print(f"Loading HMD data: {filepath}")
    df = pd.read_csv(filepath)
    print(f"  Rows: {len(df)}")
    print(f"  Columns: {list(df.columns)[:10]}...")
    return df


# =============================================================================
# Signal Extraction Functions
# =============================================================================

def extract_skeleton_points(df: pd.DataFrame) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Extract head and hand positions from skeleton DataFrame.

    Returns:
        timestamps: (N,) array of timestamps in ms
        head: (N, 3) array of head positions [x, y, z]
        left_hand: (N, 3) array of left hand positions
        right_hand: (N, 3) array of right hand positions
    """
    # New wide format from multi_device_body_viewer: J{id}_x, J{id}_y, J{id}_z, J{id}_conf
    if 'J0_x' in df.columns:
        print("  Format: Wide (multi_device_body_viewer)")

        def get_joint(joint_id):
            x_col = f'J{joint_id}_x'
            y_col = f'J{joint_id}_y'
            z_col = f'J{joint_id}_z'
            return df[[x_col, y_col, z_col]].values

        head = get_joint(JOINT_IDS['HEAD'])
        left_hand = get_joint(JOINT_IDS['LEFT_WRIST'])  # WRIST is more stable than HAND
        right_hand = get_joint(JOINT_IDS['RIGHT_WRIST'])

        # Handle both timestamp_ms and timestamp_usec formats
        if 'timestamp_ms' in df.columns:
            timestamps = df['timestamp_ms'].values
        elif 'timestamp_usec' in df.columns:
            timestamps = df['timestamp_usec'].values / 1000.0  # Convert usec to ms
        else:
            raise KeyError("No timestamp column found (expected 'timestamp_ms' or 'timestamp_usec')")
        return timestamps, head, left_hand, right_hand

    # Unity wide format: P{joint_id}_posX, P{joint_id}_posY, P{joint_id}_posZ
    elif 'P0_posX' in df.columns or 'P0_posx' in df.columns:
        print("  Format: Wide (Unity)")
        # Detect case sensitivity
        case = 'X' if 'P0_posX' in df.columns else 'x'

        def get_joint(joint_id):
            x_col = f'P{joint_id}_pos{case}'
            y_col = f'P{joint_id}_posY' if case == 'X' else f'P{joint_id}_posy'
            z_col = f'P{joint_id}_posZ' if case == 'X' else f'P{joint_id}_posz'
            return df[[x_col, y_col, z_col]].values

        head = get_joint(JOINT_IDS['HEAD'])
        left_hand = get_joint(JOINT_IDS['LEFT_WRIST'])  # WRIST is more stable than HAND
        right_hand = get_joint(JOINT_IDS['RIGHT_WRIST'])

        # Get timestamps
        if 'Timestamp' in df.columns:
            # Parse timestamp string to ms
            timestamps = pd.to_datetime(df['Timestamp']).astype(np.int64) // 10**6
        elif 'timestamp_ms' in df.columns:
            timestamps = df['timestamp_ms'].values
        else:
            # Use frame number * assumed interval
            timestamps = df['Frame'].values * (1000 / 30)  # Assume 30fps

        return timestamps, head, left_hand, right_hand

    # Long format: timestamp_ms, joint_id, pos_x, pos_y, pos_z
    elif 'joint_id' in df.columns:
        # Pivot to wide format
        head_df = df[df['joint_id'] == JOINT_IDS['HEAD']].sort_values('timestamp_ms')
        left_df = df[df['joint_id'] == JOINT_IDS['LEFT_WRIST']].sort_values('timestamp_ms')
        right_df = df[df['joint_id'] == JOINT_IDS['RIGHT_WRIST']].sort_values('timestamp_ms')

        timestamps = head_df['timestamp_ms'].values
        head = head_df[['pos_x', 'pos_y', 'pos_z']].values
        left_hand = left_df[['pos_x', 'pos_y', 'pos_z']].values
        right_hand = right_df[['pos_x', 'pos_y', 'pos_z']].values

        return timestamps, head, left_hand, right_hand

    else:
        raise ValueError(f"Unknown skeleton CSV format. Columns: {list(df.columns)}")


def extract_hmd_points(df: pd.DataFrame) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Extract HMD and controller positions from HMD DataFrame.

    Returns:
        timestamps: (N,) array of timestamps in ms
        hmd: (N, 3) array of HMD positions [x, y, z]
        left_ctrl: (N, 3) array of left controller positions
        right_ctrl: (N, 3) array of right controller positions
    """
    # Try different column naming conventions
    if 'hmd_pos_x' in df.columns:
        hmd = df[['hmd_pos_x', 'hmd_pos_y', 'hmd_pos_z']].values
        left_ctrl = df[['left_pos_x', 'left_pos_y', 'left_pos_z']].values
        right_ctrl = df[['right_pos_x', 'right_pos_y', 'right_pos_z']].values
    elif 'HMD_PosX' in df.columns:
        hmd = df[['HMD_PosX', 'HMD_PosY', 'HMD_PosZ']].values
        left_ctrl = df[['L_ctrl_PosX', 'L_ctrl_PosY', 'L_ctrl_PosZ']].values
        right_ctrl = df[['R_ctrl_PosX', 'R_ctrl_PosY', 'R_ctrl_PosZ']].values
    else:
        raise ValueError(f"Unknown HMD CSV format. Columns: {list(df.columns)}")

    # Get timestamps
    if 'timestamp_ms' in df.columns:
        timestamps = df['timestamp_ms'].values
    elif 'Timestamp' in df.columns:
        timestamps = pd.to_datetime(df['Timestamp']).astype(np.int64) // 10**6
    else:
        timestamps = df['Frame'].values * (1000 / 30)

    return timestamps, hmd, left_ctrl, right_ctrl


# =============================================================================
# Inter-Point Distance Signal Functions
# =============================================================================

def compute_interpoint_distances(head: np.ndarray,
                                  left_hand: np.ndarray,
                                  right_hand: np.ndarray) -> np.ndarray:
    """
    Compute coordinate-invariant signal from 3 body points.

    Uses sum of inter-point distances:
        - hand-to-hand distance
        - head-to-left-hand distance
        - head-to-right-hand distance

    This is invariant to coordinate system translation and rotation.

    Args:
        head: (N, 3) head positions
        left_hand: (N, 3) left hand positions
        right_hand: (N, 3) right hand positions

    Returns:
        (N,) array of combined distance signal
    """
    # Hand-to-hand distance
    hand_to_hand = np.linalg.norm(left_hand - right_hand, axis=1)

    # Head-to-hands distances
    head_to_left = np.linalg.norm(head - left_hand, axis=1)
    head_to_right = np.linalg.norm(head - right_hand, axis=1)

    # Combined signal
    combined = hand_to_hand + head_to_left + head_to_right

    return combined


def compute_triangle_area(head: np.ndarray,
                          left_hand: np.ndarray,
                          right_hand: np.ndarray) -> np.ndarray:
    """
    Compute area of triangle formed by head and two hands.

    Area = 0.5 * ||(left - head) x (right - head)||

    This is also coordinate-invariant.

    Args:
        head: (N, 3) head positions
        left_hand: (N, 3) left hand positions
        right_hand: (N, 3) right hand positions

    Returns:
        (N,) array of triangle areas
    """
    v1 = left_hand - head
    v2 = right_hand - head
    cross = np.cross(v1, v2)
    area = 0.5 * np.linalg.norm(cross, axis=1)

    return area


# =============================================================================
# Cross-Correlation Functions
# =============================================================================

def normalize_signal(signal: np.ndarray) -> np.ndarray:
    """Normalize signal to zero mean and unit variance."""
    mean = np.mean(signal)
    std = np.std(signal)
    if std < 1e-8:
        return signal - mean
    return (signal - mean) / std


def resample_to_grid(timestamps: np.ndarray,
                     signal: np.ndarray,
                     start_time: float,
                     end_time: float,
                     interval_ms: float = 10.0) -> Tuple[np.ndarray, np.ndarray]:
    """
    Resample signal to regular time grid using linear interpolation.

    Args:
        timestamps: Original timestamps
        signal: Original signal values
        start_time: Start of output time range
        end_time: End of output time range
        interval_ms: Output sampling interval in ms

    Returns:
        time_grid: Regular time grid
        resampled: Interpolated signal values
    """
    time_grid = np.arange(start_time, end_time, interval_ms)
    resampled = np.interp(time_grid, timestamps, signal)
    return time_grid, resampled


def estimate_delay_crosscorr(signal1: np.ndarray,
                              signal2: np.ndarray,
                              sample_interval_ms: float,
                              smooth_sigma: float = 3.0) -> Tuple[float, float]:
    """
    Estimate time delay between two signals using cross-correlation.

    Args:
        signal1: First signal (e.g., skeleton)
        signal2: Second signal (e.g., HMD)
        sample_interval_ms: Sampling interval in ms
        smooth_sigma: Gaussian smoothing sigma (samples)

    Returns:
        delay_ms: Estimated delay (positive = signal1 leads signal2)
        correlation: Peak correlation value
    """
    # Smooth signals
    if smooth_sigma > 0:
        signal1 = gaussian_filter1d(signal1, sigma=smooth_sigma)
        signal2 = gaussian_filter1d(signal2, sigma=smooth_sigma)

    # Normalize
    sig1_norm = normalize_signal(signal1)
    sig2_norm = normalize_signal(signal2)

    # Cross-correlation
    correlation = signal.correlate(sig1_norm, sig2_norm, mode='full')
    lags = signal.correlation_lags(len(sig1_norm), len(sig2_norm), mode='full')

    # Find peak
    peak_idx = np.argmax(np.abs(correlation))
    delay_samples = lags[peak_idx]
    delay_ms = delay_samples * sample_interval_ms
    peak_corr = correlation[peak_idx] / len(sig1_norm)  # Normalize by length

    return delay_ms, peak_corr


# =============================================================================
# Main Synchronization Function
# =============================================================================

def estimate_delay(skeleton_df: pd.DataFrame,
                   hmd_df: pd.DataFrame,
                   method: str = 'distance',
                   resample_interval_ms: float = 10.0,
                   smooth_sigma: float = 3.0) -> Tuple[float, float, dict]:
    """
    Estimate time delay between skeleton and HMD data.

    Args:
        skeleton_df: Skeleton DataFrame
        hmd_df: HMD DataFrame
        method: 'distance' (inter-point), 'area' (triangle), or 'head_y' (single point)
        resample_interval_ms: Resampling interval
        smooth_sigma: Gaussian smoothing sigma

    Returns:
        delay_ms: Estimated delay (positive = skeleton leads HMD)
        correlation: Peak correlation value
        debug_info: Dictionary with intermediate values for debugging
    """
    print(f"\nEstimating delay using method: {method}")

    # Extract points
    skel_ts, skel_head, skel_left, skel_right = extract_skeleton_points(skeleton_df)
    hmd_ts, hmd_head, hmd_left, hmd_right = extract_hmd_points(hmd_df)

    print(f"  Skeleton: {len(skel_ts)} frames, {skel_ts[0]:.0f} - {skel_ts[-1]:.0f} ms")
    print(f"  HMD: {len(hmd_ts)} frames, {hmd_ts[0]:.0f} - {hmd_ts[-1]:.0f} ms")

    # Compute signals based on method
    if method == 'distance':
        skel_signal = compute_interpoint_distances(skel_head, skel_left, skel_right)
        hmd_signal = compute_interpoint_distances(hmd_head, hmd_left, hmd_right)
        print("  Using inter-point distance (coordinate-invariant)")

    elif method == 'area':
        skel_signal = compute_triangle_area(skel_head, skel_left, skel_right)
        hmd_signal = compute_triangle_area(hmd_head, hmd_left, hmd_right)
        print("  Using triangle area (coordinate-invariant)")

    elif method == 'head_y':
        skel_signal = skel_head[:, 1]  # Y component
        hmd_signal = hmd_head[:, 1]
        print("  Using head Y position only")

    else:
        raise ValueError(f"Unknown method: {method}")

    # Find overlapping time range
    start_time = max(skel_ts[0], hmd_ts[0])
    end_time = min(skel_ts[-1], hmd_ts[-1])

    if start_time >= end_time:
        print("  ERROR: No overlapping time range!")
        return 0.0, 0.0, {}

    overlap_duration = (end_time - start_time) / 1000
    print(f"  Overlapping range: {start_time:.0f} - {end_time:.0f} ms ({overlap_duration:.1f} sec)")

    # Resample to common grid
    time_grid, skel_resampled = resample_to_grid(
        skel_ts, skel_signal, start_time, end_time, resample_interval_ms
    )
    _, hmd_resampled = resample_to_grid(
        hmd_ts, hmd_signal, start_time, end_time, resample_interval_ms
    )

    print(f"  Resampled to {len(time_grid)} points at {resample_interval_ms}ms interval")

    # Cross-correlation
    delay_ms, correlation = estimate_delay_crosscorr(
        skel_resampled, hmd_resampled, resample_interval_ms, smooth_sigma
    )

    direction = "leads" if delay_ms > 0 else "lags"
    print(f"  Estimated delay: {delay_ms:.1f} ms (skeleton {direction} HMD)")
    print(f"  Peak correlation: {correlation:.4f}")

    debug_info = {
        'time_grid': time_grid,
        'skel_signal': skel_resampled,
        'hmd_signal': hmd_resampled,
        'skel_raw': skel_signal,
        'hmd_raw': hmd_signal,
        'skel_ts': skel_ts,
        'hmd_ts': hmd_ts,
    }

    return delay_ms, correlation, debug_info


def synchronize_data(skeleton_df: pd.DataFrame,
                     hmd_df: pd.DataFrame,
                     delay_ms: float,
                     tolerance_ms: float = 50.0) -> pd.DataFrame:
    """
    Synchronize skeleton and HMD data using estimated delay.

    Args:
        skeleton_df: Skeleton DataFrame
        hmd_df: HMD DataFrame
        delay_ms: Delay to apply to skeleton timestamps
        tolerance_ms: Maximum time difference for matching

    Returns:
        Synchronized DataFrame
    """
    print(f"\nSynchronizing data (delay={delay_ms:.1f}ms, tolerance={tolerance_ms:.1f}ms)")

    # This is a placeholder - actual implementation depends on desired output format
    # For now, just return HMD data with delay info
    result = hmd_df.copy()
    result['sync_delay_ms'] = delay_ms

    print(f"  Output rows: {len(result)}")

    return result


# =============================================================================
# Visualization (optional)
# =============================================================================

def plot_signals(debug_info: dict, delay_ms: float, output_path: Optional[str] = None):
    """Plot signals and cross-correlation result."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available, skipping plot")
        return

    fig, axes = plt.subplots(3, 1, figsize=(12, 8))

    # Raw signals
    ax = axes[0]
    ax.plot(debug_info['skel_ts'], debug_info['skel_raw'], label='Skeleton', alpha=0.7)
    ax.plot(debug_info['hmd_ts'], debug_info['hmd_raw'], label='HMD', alpha=0.7)
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Signal')
    ax.set_title('Raw Signals (Inter-point Distance)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Normalized resampled signals
    ax = axes[1]
    time_grid = debug_info['time_grid']
    skel_norm = normalize_signal(debug_info['skel_signal'])
    hmd_norm = normalize_signal(debug_info['hmd_signal'])
    ax.plot(time_grid, skel_norm, label='Skeleton (normalized)', alpha=0.7)
    ax.plot(time_grid, hmd_norm, label='HMD (normalized)', alpha=0.7)
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Normalized Signal')
    ax.set_title('Normalized Resampled Signals')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Aligned signals
    ax = axes[2]
    # Shift skeleton signal by delay
    shift_samples = int(delay_ms / (time_grid[1] - time_grid[0]))
    if shift_samples > 0:
        skel_shifted = np.roll(skel_norm, -shift_samples)
    else:
        skel_shifted = np.roll(skel_norm, -shift_samples)
    ax.plot(time_grid, skel_shifted, label=f'Skeleton (shifted by {delay_ms:.0f}ms)', alpha=0.7)
    ax.plot(time_grid, hmd_norm, label='HMD', alpha=0.7)
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Normalized Signal')
    ax.set_title(f'Aligned Signals (delay = {delay_ms:.1f} ms)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150)
        print(f"Plot saved to: {output_path}")
    else:
        plt.show()


# =============================================================================
# CLI Interface
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Synchronize skeleton and HMD data using cross-correlation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Using inter-point distance method (recommended)
    python sync_skeleton_hmd.py --skeleton skel.csv --hmd hmd.csv --method distance

    # Using triangle area method
    python sync_skeleton_hmd.py --skeleton skel.csv --hmd hmd.csv --method area

    # Manual delay override
    python sync_skeleton_hmd.py --skeleton skel.csv --hmd hmd.csv --delay 150
        """
    )

    parser.add_argument('--skeleton', required=True, help='Skeleton CSV file path')
    parser.add_argument('--hmd', required=True, help='HMD CSV file path')
    parser.add_argument('--output', default='synced_data.csv', help='Output CSV file path')
    parser.add_argument('--method', choices=['distance', 'area', 'head_y'], default='distance',
                        help='Cross-correlation method (default: distance)')
    parser.add_argument('--delay', type=float, default=None,
                        help='Manual delay in ms (skips auto-estimation)')
    parser.add_argument('--tolerance', type=float, default=50.0,
                        help='Timestamp matching tolerance in ms')
    parser.add_argument('--resample', type=float, default=10.0,
                        help='Resampling interval in ms')
    parser.add_argument('--smooth', type=float, default=3.0,
                        help='Gaussian smoothing sigma (samples)')
    parser.add_argument('--plot', action='store_true',
                        help='Show visualization plot')
    parser.add_argument('--plot-output', type=str, default=None,
                        help='Save plot to file')

    args = parser.parse_args()

    # Load data
    skeleton_df = load_skeleton_csv(args.skeleton)
    hmd_df = load_hmd_csv(args.hmd)

    # Estimate or use provided delay
    if args.delay is not None:
        delay_ms = args.delay
        correlation = None
        debug_info = {}
        print(f"\nUsing manual delay: {delay_ms:.1f} ms")
    else:
        delay_ms, correlation, debug_info = estimate_delay(
            skeleton_df, hmd_df,
            method=args.method,
            resample_interval_ms=args.resample,
            smooth_sigma=args.smooth
        )

    # Plot if requested
    if (args.plot or args.plot_output) and debug_info:
        plot_signals(debug_info, delay_ms, args.plot_output)

    # Synchronize
    synced_df = synchronize_data(skeleton_df, hmd_df, delay_ms, args.tolerance)

    # Save output
    output_path = Path(args.output)
    synced_df.to_csv(output_path, index=False)
    print(f"\nSaved synchronized data to: {output_path}")

    # Print summary
    print("\n" + "="*50)
    print("SUMMARY")
    print("="*50)
    print(f"  Method: {args.method}")
    print(f"  Delay: {delay_ms:.1f} ms")
    if correlation is not None:
        print(f"  Correlation: {correlation:.4f}")
    print(f"  Output: {output_path}")


if __name__ == '__main__':
    main()
