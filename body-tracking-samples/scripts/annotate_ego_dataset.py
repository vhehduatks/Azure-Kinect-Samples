#!/usr/bin/env python3
"""
Egocentric Dataset Annotation Tool

Visual drag-and-drop editor for correcting 2D skeleton annotations
produced by the ego-dataset pipeline (batch_ego_dataset.py).

Usage:
    # Open a specific ego_dataset directory
    python annotate_ego_dataset.py --input batch_out/Dancing1_20260214_001511/ego_dataset/

    # Open the session browser for a batch directory
    python annotate_ego_dataset.py --batch-dir batch_out/

    # Open a named session from a batch directory
    python annotate_ego_dataset.py --batch-dir batch_out/ --session Dancing1_20260214_001511

    # Launch with no arguments (use File > Open in the GUI)
    python annotate_ego_dataset.py
"""

import argparse
import sys


def main():
    parser = argparse.ArgumentParser(
        description="Ego-dataset annotation tool -- visual skeleton editor",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--input", "-i", default=None,
        help="Path to an ego_dataset/ directory to open immediately",
    )
    parser.add_argument(
        "--batch-dir", "-b", default=None,
        help="Path to a batch output directory (opens session browser)",
    )
    parser.add_argument(
        "--session", "-s", default=None,
        help="Session name within --batch-dir to load directly",
    )
    args = parser.parse_args()

    from PySide6.QtWidgets import QApplication
    from annotation_tool.app import AnnotationMainWindow

    app = QApplication(sys.argv)
    app.setApplicationName("Ego Dataset Annotation Tool")

    window = AnnotationMainWindow()
    window.show()

    # Handle CLI arguments
    if args.input:
        window.open_ego_dir(args.input)
    elif args.batch_dir and args.session:
        window.open_batch_session(args.batch_dir, args.session)
    elif args.batch_dir:
        # Open the session browser pre-populated with the batch dir
        from annotation_tool.session_browser import SessionBrowserDialog
        from PySide6.QtWidgets import QDialog
        window._last_batch_dir = args.batch_dir
        dlg = SessionBrowserDialog(window, initial_dir=args.batch_dir)
        if dlg.exec() == QDialog.Accepted:
            session = dlg.selected_session()
            if session:
                window._load_session(session)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
