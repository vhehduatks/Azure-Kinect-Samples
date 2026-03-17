#!/usr/bin/env python
"""
Generate a publication-ready comparison table of egocentric body-pose datasets.

Focuses on multimodal characteristics from an HMD-centric perspective:
ego view direction, HMD device type, depth modality, train/test splits,
ground-truth provenance, etc.

Usage:
    python dataset_comparison_table.py [--output OUTPUT_PATH]
"""

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np


# ---------------------------------------------------------------------------
# Dataset definitions (sorted by year)
# ---------------------------------------------------------------------------

DATASETS = [
    {
        "name":       "EgoCap",
        "year":       2016,
        "ego_view":   "Down",
        "hmd_device": "Custom rig",
        "ego_sensor": "Stereo fisheye RGB",
        "ego_depth":  "\u2012",
        "exo_cams":   "Multi-view studio",
        "train":      "Real 75K",
        "train_gt":   "Captury MoCap",
        "test":       "Real 25K",
        "test_gt":    "Captury MoCap",
        "subjects":   "8",
        "actions":    "\u2012",
        "joints":     18,
        "twod_vis":   False,
        "dof":        "\u2012",
    },
    {
        "name":       "Mo\u00b2Cap\u00b2",
        "year":       2019,
        "ego_view":   "Down",
        "hmd_device": "Custom rig",
        "ego_sensor": "Mono fisheye RGB",
        "ego_depth":  "\u2012",
        "exo_cams":   "MoCap studio",
        "train":      "Synth 530K",
        "train_gt":   "Synthetic render",
        "test":       "Real 5.6K",
        "test_gt":    "Captury MoCap",
        "subjects":   "~8",
        "actions":    "8",
        "joints":     15,
        "twod_vis":   False,
        "dof":        "\u2012",
    },
    {
        "name":       "xR-EgoPose",
        "year":       2019,
        "ego_view":   "Down",
        "hmd_device": "Simulated",
        "ego_sensor": "Mono fisheye RGB",
        "ego_depth":  "Synth",
        "exo_cams":   "\u2012",
        "train":      "Synth 383K",
        "train_gt":   "Synthetic render",
        "test":       "Synth (subset)",
        "test_gt":    "Synthetic render",
        "subjects":   "46",
        "actions":    "9",
        "joints":     22,
        "twod_vis":   False,
        "dof":        "\u2012",
    },
    {
        "name":       "GlobalEgoMocap",
        "year":       2021,
        "ego_view":   "Down",
        "hmd_device": "Custom rig",
        "ego_sensor": "Mono fisheye RGB",
        "ego_depth":  "\u2012",
        "exo_cams":   "MoCap studio",
        "train":      "Synth (prior)",
        "train_gt":   "Synthetic render",
        "test":       "Real ~10K",
        "test_gt":    "Captury MoCap",
        "subjects":   "~3",
        "actions":    "\u2012",
        "joints":     15,
        "twod_vis":   False,
        "dof":        "\u2012",
    },
    {
        "name":       "EgoPW",
        "year":       2022,
        "ego_view":   "Down",
        "hmd_device": "Custom rig",
        "ego_sensor": "Mono fisheye RGB",
        "ego_depth":  "\u2012",
        "exo_cams":   "1 aux. cam",
        "train":      "Real 318K",
        "train_gt":   "Pseudo-GT (optim.)",
        "test":       "Real (subset)",
        "test_gt":    "Pseudo-GT (optim.)",
        "subjects":   "10",
        "actions":    "20",
        "joints":     15,
        "twod_vis":   False,
        "dof":        "\u2012",
    },
    {
        "name":       "EgoBody",
        "year":       2022,
        "ego_view":   "Forward",
        "hmd_device": "HoloLens 2 \u00d71",
        "ego_sensor": "HoloLens 2 RGB+D",
        "ego_depth":  "Real",
        "exo_cams":   "3\u20135 Kinect",
        "train":      "Real 219K",
        "train_gt":   "SMPL-X fit (Kinect)",
        "test":       "Real (subset)",
        "test_gt":    "SMPL-X fit (Kinect)",
        "subjects":   "36",
        "actions":    "\u2012",
        "joints":     22,
        "twod_vis":   False,
        "dof":        "1\u00d76DOF",
    },
    {
        "name":       "UnrealEgo",
        "year":       2022,
        "ego_view":   "Down",
        "hmd_device": "Simulated",
        "ego_sensor": "Stereo fisheye RGB",
        "ego_depth":  "Synth",
        "exo_cams":   "\u2012",
        "train":      "Synth 357K",
        "train_gt":   "Synthetic render",
        "test":       "Synth 48K",
        "test_gt":    "Synthetic render",
        "subjects":   "17",
        "actions":    "30",
        "joints":     16,
        "twod_vis":   False,
        "dof":        "\u2012",
    },
    {
        "name":       "SceneEgo",
        "year":       2023,
        "ego_view":   "Down",
        "hmd_device": "Custom rig",
        "ego_sensor": "Mono fisheye RGB",
        "ego_depth":  "Synth",
        "exo_cams":   "MoCap studio",
        "train":      "Synth 320K + Real 60K",
        "train_gt":   "GTA render + pseudo-GT",
        "test":       "Real ~28K",
        "test_gt":    "Captury MoCap",
        "subjects":   "~2",
        "actions":    "\u2012",
        "joints":     15,
        "twod_vis":   False,
        "dof":        "\u2012",
    },
    {
        "name":       "SLOPER4D",
        "year":       2023,
        "ego_view":   "Forward",
        "hmd_device": "Custom rig",
        "ego_sensor": "LiDAR + RGB",
        "ego_depth":  "Real (LiDAR)",
        "exo_cams":   "\u2012",
        "train":      "Real 100K",
        "train_gt":   "SMPL fit (LiDAR+IMU)",
        "test":       "Real (subset)",
        "test_gt":    "SMPL fit (LiDAR+IMU)",
        "subjects":   "12",
        "actions":    "\u2012",
        "joints":     24,
        "twod_vis":   False,
        "dof":        "\u2012",
    },
    {
        "name":       "Ego-Exo4D",
        "year":       2024,
        "ego_view":   "Forward",
        "hmd_device": "Aria glasses \u00d71",
        "ego_sensor": "Aria RGB+SLAM",
        "ego_depth":  "\u2012",
        "exo_cams":   "4\u20135 GoPro",
        "train":      "Real ~5M",
        "train_gt":   "Multi-view triang.",
        "test":       "Real (subset)",
        "test_gt":    "Multi-view triang.",
        "subjects":   "740",
        "actions":    "8",
        "joints":     17,
        "twod_vis":   False,
        "dof":        "1\u00d76DOF",
    },
    {
        "name":       "Nymeria",
        "year":       2024,
        "ego_view":   "Forward",
        "hmd_device": "Aria glasses \u00d71",
        "ego_sensor": "Aria RGB+ET+IMU",
        "ego_depth":  "\u2012",
        "exo_cams":   "\u2012",
        "train":      "Real 260M",
        "train_gt":   "IMU suit (Xsens)",
        "test":       "Real (subset)",
        "test_gt":    "IMU suit (Xsens)",
        "subjects":   "264",
        "actions":    "20",
        "joints":     22,
        "twod_vis":   False,
        "dof":        "1\u00d76DOF",
    },
    {
        "name":       "EMHI",
        "year":       2025,
        "ego_view":   "Down",
        "hmd_device": "PICO 4 \u00d71",
        "ego_sensor": "PICO 4 stereo+IMU",
        "ego_depth":  "\u2012",
        "exo_cams":   "8 Kinect",
        "train":      "Real 3.07M",
        "train_gt":   "SMPL fit (8-cam)",
        "test":       "Real (subset)",
        "test_gt":    "SMPL fit (8-cam)",
        "subjects":   "58",
        "actions":    "39",
        "joints":     22,
        "twod_vis":   True,
        "dof":        "3\u00d76 + 2\u00d73DOF",
    },
    {
        "name":       "Ours",
        "year":       2026,
        "ego_view":   "Down",
        "hmd_device": "Meta Quest 3 \u00d71",
        "ego_sensor": "Femto Bolt D+RGB",
        "ego_depth":  "Real",
        "exo_cams":   "3 Femto Bolt",
        "train":      "Real 132K",
        "train_gt":   "Depth fusion (3-cam)",
        "test":       "Real (subset)",
        "test_gt":    "Depth fusion (3-cam)",
        "subjects":   "41",
        "actions":    "20",
        "joints":     32,
        "twod_vis":   True,
        "dof":        "3\u00d76DOF",
    },
]

# ---------------------------------------------------------------------------
# Table column definitions  (label, key, proportional width)
# ---------------------------------------------------------------------------

COLUMNS = [
    ("Dataset",     "name",       6.5),
    ("Year",        "year",       1.8),
    ("Ego\nView",   "ego_view",   3.0),
    ("HMD Device",  "hmd_device", 6.0),
    ("HMD\nDOF",    "dof",        4.0),
    ("Ego Sensor",  "ego_sensor", 6.0),
    ("Ego\nDepth",  "ego_depth",  3.5),
    ("Exo Cams",    "exo_cams",   5.0),
    ("Train Data",  "train",      6.5),
    ("Train GT",    "train_gt",   7.0),
    ("Test Data",   "test",       5.5),
    ("Test GT",     "test_gt",    7.0),
    ("Subj.",       "subjects",   2.2),
    ("Acts.",       "actions",    2.0),
    ("Joints",      "joints",     2.2),
    ("2D+\nVis",    "twod_vis",   2.0),
]

# ---------------------------------------------------------------------------
# Colours
# ---------------------------------------------------------------------------

CLR_HEADER_BG   = "#2C3E50"
CLR_HEADER_FG   = "#FFFFFF"
CLR_OUR_ROW_BG  = "#D6EAF8"
CLR_BEST_CELL   = "#A9DFBF"
CLR_ALT_ROW     = "#F8F9F9"
CLR_WHITE       = "#FFFFFF"
CLR_STRENGTH_BG = "#EBF5FB"
CLR_CHECK       = "#27AE60"
CLR_BODY_TEXT   = "#2C3E50"
CLR_SYNTH_TEXT  = "#8E44AD"     # purple for synthetic entries
CLR_MIXED_TEXT  = "#D35400"     # orange for mixed entries
CLR_REAL_TEXT   = "#2C3E50"     # dark for real entries

BOOL_YES = "\u2714"
BOOL_NO  = "\u2012"

# Keys in "our" row that get green highlight
OUR_BEST_KEYS = {
    "joints", "ego_depth", "twod_vis", "train_gt", "test_gt",
    "ego_sensor", "hmd_device", "dof",
}


def _fmt(val):
    if isinstance(val, bool):
        return BOOL_YES if val else BOOL_NO
    return str(val)


def _is_ours(ds):
    return ds["name"] == "Ours"


def _text_colour(col_key, val, is_ours):
    """Pick text colour based on content — purple for synth, orange for mixed."""
    if is_ours:
        return CLR_BODY_TEXT
    if isinstance(val, bool):
        return CLR_CHECK if val else "#BDC3C7"
    s = str(val).lower()
    if col_key in ("train", "test", "train_gt", "test_gt", "ego_depth"):
        if "synth" in s and "real" in s:
            return CLR_MIXED_TEXT
        if "synth" in s or "simulated" in s:
            return CLR_SYNTH_TEXT
    if col_key == "hmd_device":
        if "simulated" in s:
            return CLR_SYNTH_TEXT
    return CLR_BODY_TEXT


# ---------------------------------------------------------------------------
# Build figure
# ---------------------------------------------------------------------------

def build_figure(output_path: Path):
    n_rows = len(DATASETS)
    n_cols = len(COLUMNS)

    col_widths_raw = [c[2] for c in COLUMNS]
    total_w = sum(col_widths_raw)
    col_widths = [w / total_w for w in col_widths_raw]

    fig_w = 22.0
    table_h_per_row = 0.44
    header_h = 0.58
    table_h = header_h + n_rows * table_h_per_row
    strength_h = 2.9
    margin_top = 0.65
    margin_bot = 0.25
    gap = 0.50
    fig_h = margin_top + table_h + gap + strength_h + margin_bot

    fig = plt.figure(figsize=(fig_w, fig_h), dpi=300, facecolor="white")

    # ---- Table axis ----
    ax_table = fig.add_axes([
        0.015,
        (margin_bot + strength_h + gap) / fig_h,
        0.97,
        table_h / fig_h,
    ])
    ax_table.axis("off")

    # Build cell text & colours
    cell_text = []
    cell_colours = []
    for i, ds in enumerate(DATASETS):
        row_text, row_clr = [], []
        is_ours = _is_ours(ds)
        base_bg = CLR_OUR_ROW_BG if is_ours else (CLR_ALT_ROW if i % 2 == 0 else CLR_WHITE)
        for _, col_key, _ in COLUMNS:
            val = ds[col_key]
            row_text.append(_fmt(val))
            if is_ours and col_key in OUR_BEST_KEYS:
                row_clr.append(CLR_BEST_CELL)
            else:
                row_clr.append(base_bg)
        cell_text.append(row_text)
        cell_colours.append(row_clr)

    col_labels = [c[0] for c in COLUMNS]

    the_table = ax_table.table(
        cellText=cell_text,
        colLabels=col_labels,
        colWidths=col_widths,
        cellLoc="center",
        loc="upper center",
    )
    the_table.auto_set_font_size(False)
    the_table.set_fontsize(7.5)

    for (row, col), cell in the_table.get_celld().items():
        cell.set_edgecolor("#D5D8DC")
        cell.set_linewidth(0.45)

        if row == 0:
            cell.set_facecolor(CLR_HEADER_BG)
            cell.set_text_props(color=CLR_HEADER_FG, fontweight="bold", fontsize=7.5)
            cell.set_height(header_h / table_h)
        else:
            data_idx = row - 1
            cell.set_facecolor(cell_colours[data_idx][col])
            ds = DATASETS[data_idx]
            is_ours = _is_ours(ds)
            col_key = COLUMNS[col][1]
            val = ds[col_key]

            fw = "bold" if is_ours else "normal"
            fs = 7.8 if is_ours else 7.2
            clr = _text_colour(col_key, val, is_ours)

            if isinstance(val, bool):
                fs_sym = fs + 1.5
                clr = CLR_CHECK if val else "#BDC3C7"
                cell.set_text_props(color=clr, fontweight=fw, fontsize=fs_sym)
            else:
                cell.set_text_props(color=clr, fontweight=fw, fontsize=fs)

            cell.set_height(table_h_per_row / table_h)

    # ---- Title ----
    fig.text(
        0.50, 1.0 - 0.22 / fig_h,
        "Comparison of Egocentric Body-Pose Datasets \u2014 Multimodal & HMD Perspective",
        ha="center", va="center",
        fontsize=14, fontweight="bold", color=CLR_HEADER_BG,
    )

    # ---- Legend for text colours ----
    legend_y = 1.0 - 0.50 / fig_h
    fig.text(0.26, legend_y,
             "\u25cf Real data", fontsize=7.0, color=CLR_REAL_TEXT, ha="left")
    fig.text(0.35, legend_y,
             "\u25cf Synthetic data", fontsize=7.0, color=CLR_SYNTH_TEXT, ha="left")
    fig.text(0.47, legend_y,
             "\u25cf Mixed (real + synth)", fontsize=7.0, color=CLR_MIXED_TEXT, ha="left")
    fig.text(0.62, legend_y,
             "\u25cf Our best-in-class", fontsize=7.0, color=CLR_CHECK, ha="left",
             bbox=dict(facecolor=CLR_BEST_CELL, edgecolor="none", pad=1.5, alpha=0.6))

    # ---- Strength summary panel ----
    ax_str = fig.add_axes([
        0.015,
        margin_bot / fig_h,
        0.97,
        strength_h / fig_h,
    ])
    ax_str.set_xlim(0, 1)
    ax_str.set_ylim(0, 1)
    ax_str.axis("off")

    ax_str.add_patch(mpatches.FancyBboxPatch(
        (0.003, 0.02), 0.994, 0.96,
        boxstyle="round,pad=0.012",
        facecolor=CLR_STRENGTH_BG, edgecolor="#AED6F1", linewidth=1.0,
    ))

    ax_str.text(0.02, 0.93, "Key Differentiators of Our Dataset",
                fontsize=10.5, fontweight="bold", color=CLR_HEADER_BG,
                va="top", transform=ax_str.transAxes)

    # Two-column layout for strengths
    strengths_left = [
        ("\u2714  Real multi-camera depth-fusion GT",
         "4 sync'd depth cameras \u2014 no SMPL fitting,\nno pseudo-labels, not synthetic."),
        ("\u2714  32 joints (highest count)",
         "Full Azure Kinect skeleton; most datasets\nprovide 15\u201324 joints."),
        ("\u2714  Paired 3D + 2D + per-joint visibility",
         "Every frame has 3D, 2D projection, and\nvisibility flags \u2014 rare in the literature."),
        ("\u2714  Active ego depth + RGB (not fisheye)",
         "Helmet-mounted Femto Bolt provides metric\ndepth per ego frame (not passive RGB)."),
    ]
    strengths_right = [
        ("\u2714  Synchronized HMD 6DOF tracking",
         "Head position & rotation at capture rate;\nenables ego-motion studies."),
        ("\u2714  Helmet-local + world-space + camera pose",
         "Three coordinate frames per frame; enables\nego\u2194world mapping and head-motion analysis."),
        ("\u2714  Real train & test with same GT source",
         "Both splits use depth-fusion GT \u2014 no domain\ngap between train and test supervision."),
        ("\u2714  20 action classes \u00d7 41 participants",
         "Broad activity vocabulary with real subjects;\ncompetitive diversity among real datasets."),
    ]

    def _draw_strengths(ax, items, x_start, y_start):
        y = y_start
        dy = 0.200
        for title, desc in items:
            ax.text(x_start, y, title,
                    fontsize=8.0, fontweight="bold", color=CLR_CHECK,
                    va="top", transform=ax.transAxes)
            ax.text(x_start + 0.018, y - 0.055, desc,
                    fontsize=6.8, color="#566573", linespacing=1.35,
                    va="top", transform=ax.transAxes)
            y -= dy

    _draw_strengths(ax_str, strengths_left, 0.025, 0.82)
    _draw_strengths(ax_str, strengths_right, 0.52, 0.82)

    # ---- Save ----
    fig.savefig(str(output_path), dpi=300, bbox_inches="tight",
                facecolor="white", edgecolor="none")
    plt.close(fig)
    print(f"Saved: {output_path}  ({output_path.stat().st_size / 1024:.0f} KB)")


# ---------------------------------------------------------------------------
# Short highlight table  (downward ego-view only)
# ---------------------------------------------------------------------------

SHORT_COLUMNS = [
    ("Dataset",      "name",       5.5),
    ("Year",         "year",       1.8),
    ("HMD Device",   "hmd_device", 5.5),
    ("HMD DOF",      "dof",        4.5),
    ("Ego Depth",    "ego_depth",  3.0),
    ("GT Source",    "train_gt",   7.0),
    ("Train Data",   "train",      6.5),
    ("Test Data",    "test",       5.5),
    ("Joints",       "joints",     2.2),
    ("2D+Vis",       "twod_vis",   2.2),
]

SHORT_BEST_KEYS = {
    "joints", "ego_depth", "twod_vis", "train_gt", "hmd_device",
    "train", "test", "dof",
}


def build_short_figure(output_path: Path):
    # --- Filter to downward ego-view only ---
    down_ds = [ds for ds in DATASETS if ds["ego_view"] == "Down"]

    n_rows = len(down_ds)
    n_cols = len(SHORT_COLUMNS)

    col_widths_raw = [c[2] for c in SHORT_COLUMNS]
    total_w = sum(col_widths_raw)
    col_widths = [w / total_w for w in col_widths_raw]

    fig_w = 14.0
    table_h_per_row = 0.42
    header_h = 0.50
    table_h = header_h + n_rows * table_h_per_row
    margin_top = 0.55
    margin_bot = 0.20
    fig_h = margin_top + table_h + margin_bot

    fig = plt.figure(figsize=(fig_w, fig_h), dpi=300, facecolor="white")
    ax = fig.add_axes([0.02, margin_bot / fig_h, 0.96, table_h / fig_h])
    ax.axis("off")

    # Build cells
    cell_text = []
    cell_colours = []
    for i, ds in enumerate(down_ds):
        row_text, row_clr = [], []
        is_ours = _is_ours(ds)
        base_bg = CLR_OUR_ROW_BG if is_ours else (CLR_ALT_ROW if i % 2 == 0 else CLR_WHITE)

        for _, col_key, _ in SHORT_COLUMNS:
            val = ds[col_key]
            row_text.append(_fmt(val))
            if is_ours and col_key in SHORT_BEST_KEYS:
                row_clr.append(CLR_BEST_CELL)
            else:
                row_clr.append(base_bg)

        cell_text.append(row_text)
        cell_colours.append(row_clr)

    col_labels = [c[0] for c in SHORT_COLUMNS]

    the_table = ax.table(
        cellText=cell_text,
        colLabels=col_labels,
        colWidths=col_widths,
        cellLoc="center",
        loc="upper center",
    )
    the_table.auto_set_font_size(False)
    the_table.set_fontsize(8.5)

    for (row, col), cell in the_table.get_celld().items():
        cell.set_edgecolor("#D5D8DC")
        cell.set_linewidth(0.5)

        if row == 0:
            cell.set_facecolor(CLR_HEADER_BG)
            cell.set_text_props(color=CLR_HEADER_FG, fontweight="bold", fontsize=8.5)
            cell.set_height(header_h / table_h)
        else:
            data_idx = row - 1
            cell.set_facecolor(cell_colours[data_idx][col])
            ds = down_ds[data_idx]
            is_ours = _is_ours(ds)
            col_key = SHORT_COLUMNS[col][1]
            val = ds[col_key]

            fw = "bold" if is_ours else "normal"
            fs = 9.0 if is_ours else 8.2

            if isinstance(val, bool):
                clr = CLR_CHECK if val else "#BDC3C7"
                cell.set_text_props(color=clr, fontweight=fw, fontsize=fs + 1)
            else:
                clr = _text_colour(col_key, val, is_ours)
                cell.set_text_props(color=clr, fontweight=fw, fontsize=fs)

            cell.set_height(table_h_per_row / table_h)

    # Title
    fig.text(
        0.50, 1.0 - 0.18 / fig_h,
        "Downward Ego-View Datasets \u2014 Strengths at a Glance",
        ha="center", va="center",
        fontsize=13, fontweight="bold", color=CLR_HEADER_BG,
    )

    # Legend
    legend_y = 1.0 - 0.42 / fig_h
    fig.text(0.18, legend_y,
             "\u25cf Real", fontsize=7.5, color=CLR_REAL_TEXT, ha="left")
    fig.text(0.26, legend_y,
             "\u25cf Synthetic", fontsize=7.5, color=CLR_SYNTH_TEXT, ha="left")
    fig.text(0.37, legend_y,
             "\u25cf Mixed", fontsize=7.5, color=CLR_MIXED_TEXT, ha="left")
    fig.text(0.47, legend_y,
             "\u25cf Our advantage", fontsize=7.5, color=CLR_CHECK, ha="left",
             bbox=dict(facecolor=CLR_BEST_CELL, edgecolor="none", pad=1.5, alpha=0.6))

    fig.savefig(str(output_path), dpi=300, bbox_inches="tight",
                facecolor="white", edgecolor="none")
    plt.close(fig)
    print(f"Saved: {output_path}  ({output_path.stat().st_size / 1024:.0f} KB)")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Dataset comparison table generator")
    parser.add_argument("--output", type=str,
                        default="dataset_comparison_table.png",
                        help="Output PNG path")
    parser.add_argument("--short", action="store_true",
                        help="Generate short highlight table instead of full table")
    args = parser.parse_args()
    if args.short:
        out = Path(args.output)
        if out.name == "dataset_comparison_table.png":
            out = out.with_name("dataset_comparison_short.png")
        build_short_figure(out)
    else:
        build_figure(Path(args.output))


if __name__ == "__main__":
    main()
