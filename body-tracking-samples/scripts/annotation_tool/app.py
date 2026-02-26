"""Main application window assembling all annotation tool widgets."""

from pathlib import Path

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QAction, QKeySequence, QShortcut, QUndoStack
from PySide6.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QDockWidget,
    QMessageBox,
    QFileDialog,
    QStatusBar,
    QComboBox,
    QDialogButtonBox,
    QDialog,
    QLabel,
    QFormLayout,
    QCheckBox,
)

from .constants import JOINT_NAMES
from .data_model import AnnotationModel
from .undo_commands import (
    MoveJointCommand,
    MoveMultipleJointsCommand,
    ToggleVisibilityCommand,
    SetKeyframeCommand,
    BatchMoveCommand,
)
from .viewport import AnnotationViewport
from .timeline import TimelineWidget
from .properties import JointTreeWidget, HMDInfoPanel
from .extrinsic_panel import ExtrinsicPanel
from .session_browser import SessionBrowserDialog
from .interpolation import InterpolationEngine


class AnnotationMainWindow(QMainWindow):
    """Top-level window for the ego-dataset annotation tool."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Ego Dataset Annotation Tool")
        self.resize(1400, 900)

        # Core objects
        self.model = AnnotationModel(self)
        self.undo_stack = QUndoStack(self)

        # Play/pause timer
        self._play_timer = QTimer(self)
        self._play_timer.setInterval(33)  # ~30 fps
        self._play_timer.timeout.connect(self._advance_frame)

        # --- Widgets ---
        self.viewport = AnnotationViewport(self.model)
        self.timeline = TimelineWidget(self.model)
        self.joint_tree = JointTreeWidget(self.model)
        self.hmd_panel = HMDInfoPanel(self.model)
        self.extrinsic_panel = ExtrinsicPanel()

        # Central area: viewport + timeline
        central = QWidget()
        central_layout = QVBoxLayout(central)
        central_layout.setContentsMargins(0, 0, 0, 0)
        central_layout.setSpacing(0)
        central_layout.addWidget(self.viewport, stretch=1)
        central_layout.addWidget(self.timeline, stretch=0)
        self.setCentralWidget(central)

        # Right dock: joint tree + HMD info
        right_dock = QDockWidget("Properties", self)
        right_dock.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(2, 2, 2, 2)
        right_layout.addWidget(self.joint_tree, stretch=1)
        right_layout.addWidget(self.hmd_panel, stretch=0)
        self.prune_checkbox = QCheckBox("Prune Head Joints (26\u201331)")
        self.prune_checkbox.setToolTip(
            "Hide head/face joints (HEAD, NOSE, EYE_LEFT, EYE_RIGHT, EAR_LEFT, EAR_RIGHT)\n"
            "and mark them visible=false on save"
        )
        right_layout.addWidget(self.prune_checkbox, stretch=0)
        right_layout.addWidget(self.extrinsic_panel, stretch=0)
        right_dock.setWidget(right_widget)
        self.addDockWidget(Qt.RightDockWidgetArea, right_dock)

        # Status bar
        self._status = QStatusBar(self)
        self.setStatusBar(self._status)

        # --- Menus & Shortcuts ---
        self._build_menus()
        self._build_shortcuts()

        # --- Signal wiring ---
        self.viewport.group_drag_finished.connect(self._on_group_drag)
        self.joint_tree.visibility_toggled.connect(self._on_visibility_toggled)
        self.joint_tree.keyframe_toggled.connect(self._on_keyframe_toggled)
        self.model.dirty_changed.connect(self._update_title)
        self.model.session_loaded.connect(self._update_title)

        # Head-joint pruning
        self.prune_checkbox.toggled.connect(lambda on: self.model.set_pruning(on))
        self.model.pruning_changed.connect(self._on_pruning_changed)

        # Extrinsic panel
        self.extrinsic_panel.extrinsic_changed.connect(self.model.set_extrinsic_delta)
        self.extrinsic_panel.apply_all_clicked.connect(self._apply_extrinsic_all)
        self.extrinsic_panel.export_clicked.connect(self._export_extrinsic)
        self.model.session_loaded.connect(self._on_session_loaded_extrinsic)

    # ==================================================================
    # Menus
    # ==================================================================
    def _build_menus(self):
        mb = self.menuBar()

        # File
        file_menu = mb.addMenu("&File")
        file_menu.addAction("Open &Session...", self._open_session_browser, QKeySequence("Ctrl+O"))
        file_menu.addAction("Open &Directory...", self._open_directory)
        file_menu.addSeparator()
        self._save_action = file_menu.addAction("&Save", self._save, QKeySequence("Ctrl+S"))
        file_menu.addSeparator()
        file_menu.addAction("E&xit", self.close, QKeySequence("Alt+F4"))

        # Edit
        edit_menu = mb.addMenu("&Edit")
        undo_action = self.undo_stack.createUndoAction(self, "&Undo")
        undo_action.setShortcut(QKeySequence("Ctrl+Z"))
        edit_menu.addAction(undo_action)
        redo_action = self.undo_stack.createRedoAction(self, "&Redo")
        redo_action.setShortcut(QKeySequence("Ctrl+Y"))
        edit_menu.addAction(redo_action)
        edit_menu.addSeparator()
        edit_menu.addAction("Reset Joint to &Original", self._reset_selected_joint, QKeySequence("Delete"))
        edit_menu.addAction("Toggle &Keyframe", self._toggle_keyframe, QKeySequence("K"))
        edit_menu.addSeparator()
        edit_menu.addAction("Apply &Interpolation...", self._apply_interpolation, QKeySequence("Ctrl+I"))

        # View
        view_menu = mb.addMenu("&View")
        view_menu.addAction("&Fit to Window", self.viewport.fit_to_view, QKeySequence("F"))

        # Navigate
        nav_menu = mb.addMenu("&Navigate")
        nav_menu.addAction("&Previous Frame", self._prev_frame, QKeySequence("Left"))
        nav_menu.addAction("&Next Frame", self._next_frame, QKeySequence("Right"))
        nav_menu.addAction("Skip &Back 10", self._skip_back, QKeySequence("PgUp"))
        nav_menu.addAction("Skip &Forward 10", self._skip_forward, QKeySequence("PgDown"))
        nav_menu.addAction("&First Frame", self._first_frame, QKeySequence("Home"))
        nav_menu.addAction("&Last Frame", self._last_frame, QKeySequence("End"))
        nav_menu.addSeparator()
        nav_menu.addAction("Play / Pa&use", self._toggle_play, QKeySequence("Space"))

    def _build_shortcuts(self):
        # Additional shortcut aliases (A/D duplicate Left/Right)
        QShortcut(QKeySequence("A"), self).activated.connect(self._prev_frame)
        QShortcut(QKeySequence("D"), self).activated.connect(self._next_frame)
        QShortcut(QKeySequence("Escape"), self).activated.connect(self._deselect_all)

    # ==================================================================
    # Navigation
    # ==================================================================
    def _prev_frame(self):
        self.model.current_frame -= 1

    def _next_frame(self):
        self.model.current_frame += 1

    def _skip_back(self):
        self.model.current_frame -= 10

    def _skip_forward(self):
        self.model.current_frame += 10

    def _first_frame(self):
        self.model.current_frame = 0

    def _last_frame(self):
        self.model.current_frame = self.model.frame_count - 1

    def _toggle_play(self):
        if self._play_timer.isActive():
            self._play_timer.stop()
            self._status.showMessage("Paused", 2000)
        else:
            self._play_timer.start()
            self._status.showMessage("Playing...", 2000)

    def _advance_frame(self):
        if self.model.current_frame < self.model.frame_count - 1:
            self.model.current_frame += 1
        else:
            self._play_timer.stop()
            self._status.showMessage("End of session", 2000)

    # ==================================================================
    # Undo-driven editing slots
    # ==================================================================
    def _on_group_drag(self, moves):
        """Handle drag completion for one or more joints.

        Args:
            moves: list of (joint_id, old_u, old_v, new_u, new_v)
        """
        frame = self.model.current_frame
        if len(moves) == 1:
            jid, old_u, old_v, new_u, new_v = moves[0]
            cmd = MoveJointCommand(self.model, frame, jid,
                                   old_u, old_v, new_u, new_v)
        else:
            cmd = MoveMultipleJointsCommand(self.model, frame, moves)
        self.undo_stack.push(cmd)

    def _on_visibility_toggled(self, frame, jid, new_vis):
        _, _, _, old_vis = self.model.get_joint_2d(frame, jid)
        cmd = ToggleVisibilityCommand(self.model, frame, jid, old_vis, new_vis)
        self.undo_stack.push(cmd)

    def _on_keyframe_toggled(self, frame, jid, new_kf):
        cmd = SetKeyframeCommand(self.model, frame, jid, new_kf)
        self.undo_stack.push(cmd)

    def _toggle_keyframe(self):
        jids = self.viewport.get_selected_joint_ids()
        if not jids:
            jid = self.model.selected_joint
            if jid >= 0:
                jids = [jid]
        if not jids:
            self._status.showMessage("Select a joint first", 3000)
            return
        frame = self.model.current_frame
        if len(jids) == 1:
            jid = jids[0]
            current_kf = frame in self.model.get_keyframes(jid)
            cmd = SetKeyframeCommand(self.model, frame, jid, not current_kf)
            self.undo_stack.push(cmd)
            state = "set" if not current_kf else "cleared"
            name = JOINT_NAMES[jid] if jid < len(JOINT_NAMES) else f"Joint {jid}"
            self._status.showMessage(f"Keyframe {state} for {name} at frame {frame}", 3000)
        else:
            self.undo_stack.beginMacro(f"Toggle keyframes ({len(jids)} joints) @ frame {frame}")
            for jid in jids:
                current_kf = frame in self.model.get_keyframes(jid)
                cmd = SetKeyframeCommand(self.model, frame, jid, not current_kf)
                self.undo_stack.push(cmd)
            self.undo_stack.endMacro()
            self._status.showMessage(
                f"Toggled keyframe for {len(jids)} joints at frame {frame}", 3000
            )

    def _reset_selected_joint(self):
        jids = self.viewport.get_selected_joint_ids()
        if not jids:
            jid = self.model.selected_joint
            if jid >= 0:
                jids = [jid]
        if not jids:
            self._status.showMessage("Select a joint first", 3000)
            return

        frame = self.model.current_frame
        moves = []
        for jid in jids:
            u, v, _, _ = self.model.get_joint_2d(frame, jid)
            orig = self.model._get_original_joint(frame, jid)
            if orig is None:
                continue
            if abs(u - orig[0]) > 0.01 or abs(v - orig[1]) > 0.01:
                moves.append((jid, u, v, orig[0], orig[1]))

        if not moves:
            self._status.showMessage("Selected joint(s) already at original positions", 3000)
            return

        if len(moves) == 1:
            jid, old_u, old_v, new_u, new_v = moves[0]
            cmd = MoveJointCommand(self.model, frame, jid, old_u, old_v, new_u, new_v)
        else:
            cmd = MoveMultipleJointsCommand(self.model, frame, moves)
        self.undo_stack.push(cmd)
        self._status.showMessage(
            f"Reset {len(moves)} joint(s) to original at frame {frame}", 3000
        )

    def _deselect_all(self):
        self.model.selected_joint = -1
        self.viewport._scene.clearSelection()

    # ==================================================================
    # Interpolation
    # ==================================================================
    def _apply_interpolation(self):
        # Collect joints: prefer viewport multi-selection, fall back to model
        jids = self.viewport.get_selected_joint_ids()
        if not jids:
            jid = self.model.selected_joint
            if jid >= 0:
                jids = [jid]
        if not jids:
            self._status.showMessage("Select a joint first", 3000)
            return

        # Keep only joints that have >= 2 keyframes
        eligible = {}  # jid -> sorted keyframe list
        for jid in jids:
            kf = sorted(self.model.get_keyframes(jid))
            if len(kf) >= 2:
                eligible[jid] = kf

        if not eligible:
            n_kf = max(
                (len(self.model.get_keyframes(j)) for j in jids), default=0
            )
            QMessageBox.information(
                self, "Interpolation",
                f"Need at least 2 keyframes per joint.\n"
                f"Selected {len(jids)} joint(s), max keyframes on any: {n_kf}.",
            )
            return

        # Build summary text for the dialog
        joint_lines = []
        for jid, kf in eligible.items():
            name = JOINT_NAMES[jid] if jid < len(JOINT_NAMES) else f"Joint {jid}"
            joint_lines.append(f"  {name}: {len(kf)} KFs ({kf[0]}..{kf[-1]})")
        summary = "\n".join(joint_lines)

        # Dialog to choose mode
        dlg = QDialog(self)
        dlg.setWindowTitle("Apply Interpolation")
        layout = QFormLayout(dlg)
        layout.addRow("Joints:", QLabel(f"{len(eligible)}"))
        detail_label = QLabel(summary)
        detail_label.setStyleSheet("font-family: monospace; font-size: 9pt;")
        layout.addRow(detail_label)
        mode_combo = QComboBox()
        mode_combo.addItems(["Linear", "Cubic Spline"])
        layout.addRow("Mode:", mode_combo)
        btn_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btn_box.accepted.connect(dlg.accept)
        btn_box.rejected.connect(dlg.reject)
        layout.addWidget(btn_box)

        if dlg.exec() != QDialog.Accepted:
            return

        mode = "cubic" if mode_combo.currentIndex() == 1 else "linear"

        # Apply interpolation for every eligible joint inside one undo macro
        total_frames = 0
        self.undo_stack.beginMacro(
            f"Interpolate {len(eligible)} joint(s) ({mode})"
        )
        for jid, kf_frames in eligible.items():
            keyframes = []
            for f in kf_frames:
                u, v, _, _ = self.model.get_joint_2d(f, jid)
                keyframes.append((f, u, v))

            interpolated = InterpolationEngine.interpolate_joint(keyframes, mode)
            if not interpolated:
                continue

            moves = []
            for frame_idx, (new_u, new_v) in interpolated.items():
                old_u, old_v, _, _ = self.model.get_joint_2d(frame_idx, jid)
                moves.append((frame_idx, old_u, old_v, new_u, new_v))

            if moves:
                cmd = BatchMoveCommand(self.model, jid, moves)
                self.undo_stack.push(cmd)
                total_frames += len(moves)
        self.undo_stack.endMacro()

        self._status.showMessage(
            f"Interpolated {len(eligible)} joint(s), "
            f"{total_frames} frames total ({mode})",
            5000,
        )

    # ==================================================================
    # Head-joint pruning
    # ==================================================================
    def _on_pruning_changed(self):
        if self.model.pruning_enabled:
            self._status.showMessage(
                "Head joints (26\u201331) pruned \u2014 will be marked invisible on save", 5000
            )
        else:
            self._status.showMessage("Head joint pruning disabled", 3000)

    # ==================================================================
    # Extrinsic fine-tuning
    # ==================================================================
    def _on_session_loaded_extrinsic(self):
        if self.model.has_intrinsics():
            self._status.showMessage("Intrinsics estimated from 3D/2D pairs", 5000)
        else:
            self._status.showMessage(
                "Extrinsic tuning unavailable: could not estimate intrinsics", 5000
            )

    def _apply_extrinsic_all(self):
        if not self.model.has_intrinsics():
            QMessageBox.warning(
                self, "Extrinsic Tuning",
                "Cannot apply: camera intrinsics could not be estimated.\n"
                "Need frames with both skeleton_3d and skeleton_2d data.",
            )
            return
        if not self.model.has_extrinsic_delta():
            self._status.showMessage("Nothing to apply (all sliders at zero)", 3000)
            return
        ans = QMessageBox.question(
            self, "Apply Extrinsic to All Frames",
            "This will overwrite skeleton_2d in all annotation JSONs\n"
            "using the current extrinsic delta.\n\n"
            "Backups (.bak) will be created. Continue?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if ans != QMessageBox.Yes:
            return
        n = self.model.apply_extrinsic_to_all_frames()
        self.undo_stack.clear()
        self._status.showMessage(
            f"Applied extrinsic delta to {n} frames (.bak backups created)", 5000
        )

    def _export_extrinsic(self):
        if not self.model.has_extrinsic_delta():
            self._status.showMessage("Nothing to export (all sliders at zero)", 3000)
            return
        orig_path, _ = QFileDialog.getOpenFileName(
            self, "Select original T_checker_to_A JSON",
            "", "JSON files (*.json)",
        )
        if not orig_path:
            return
        out_path, _ = QFileDialog.getSaveFileName(
            self, "Save adjusted transform",
            str(Path(orig_path).with_name("T_checker_to_A_adjusted.json")),
            "JSON files (*.json)",
        )
        if not out_path:
            return
        self.model.export_adjusted_transform(orig_path, out_path)
        self._status.showMessage(f"Exported adjusted transform to {out_path}", 5000)

    # ==================================================================
    # Session opening / saving
    # ==================================================================
    def _open_session_browser(self):
        if not self._check_unsaved():
            return
        dlg = SessionBrowserDialog(self)
        if dlg.exec() == QDialog.Accepted:
            session = dlg.selected_session()
            if session:
                self._load_session(session)

    def _open_directory(self):
        if not self._check_unsaved():
            return
        d = QFileDialog.getExistingDirectory(self, "Select ego_dataset directory")
        if d:
            ego_dir = Path(d)
            if not (ego_dir / "annotations").is_dir():
                QMessageBox.warning(
                    self, "Invalid directory",
                    "Selected directory does not contain an 'annotations' subfolder.",
                )
                return
            # Check for synced_data.csv in parent
            synced = ego_dir.parent / "synced_data.csv"
            hmd_csv = str(synced) if synced.exists() else None
            self.model.load_session(str(ego_dir), hmd_csv=hmd_csv)
            self.undo_stack.clear()

    def _load_session(self, session: dict):
        hmd_csv = str(session["synced_csv"]) if session.get("synced_csv") else None
        self.model.load_session(
            str(session["ego_dir"]),
            hmd_csv=hmd_csv,
            session_name=session["name"],
        )
        self.undo_stack.clear()

    def _save(self):
        if self.model.dataset is None:
            return
        n = self.model.save()
        self._status.showMessage(f"Saved {n} annotation files (.bak backups created)", 5000)
        self._update_title()

    def _check_unsaved(self) -> bool:
        """Return True if OK to proceed (no unsaved changes or user dismissed)."""
        if not self.model.is_dirty:
            return True
        ans = QMessageBox.question(
            self, "Unsaved Changes",
            "There are unsaved annotation changes.\nDiscard and continue?",
            QMessageBox.Discard | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        return ans == QMessageBox.Discard

    # ==================================================================
    # Window title
    # ==================================================================
    def _update_title(self, *_args):
        name = self.model.session_name or "Ego Dataset Annotation Tool"
        dirty = " *" if self.model.is_dirty else ""
        count = f" ({self.model.frame_count} frames)" if self.model.dataset else ""
        self.setWindowTitle(f"{name}{count}{dirty}")

    # ==================================================================
    # Close event
    # ==================================================================
    def closeEvent(self, event):
        self._play_timer.stop()
        if self.model.is_dirty:
            ans = QMessageBox.question(
                self, "Unsaved Changes",
                "Save changes before closing?",
                QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel,
                QMessageBox.Save,
            )
            if ans == QMessageBox.Save:
                self._save()
                event.accept()
            elif ans == QMessageBox.Discard:
                event.accept()
            else:
                event.ignore()
        else:
            event.accept()

    # ==================================================================
    # Public API for CLI
    # ==================================================================
    def open_ego_dir(self, ego_dir: str):
        """Load a session from a direct ego_dataset path (called from CLI)."""
        ego_path = Path(ego_dir)
        synced = ego_path.parent / "synced_data.csv"
        hmd_csv = str(synced) if synced.exists() else None
        self.model.load_session(str(ego_path), hmd_csv=hmd_csv)

    def open_batch_session(self, batch_dir: str, session_name: str):
        """Load a specific session from a batch directory (called from CLI)."""
        from .constants import discover_batch_sessions
        sessions = discover_batch_sessions(Path(batch_dir))
        for s in sessions:
            if s["name"] == session_name:
                self._load_session(s)
                return
        raise ValueError(f"Session '{session_name}' not found in {batch_dir}")
