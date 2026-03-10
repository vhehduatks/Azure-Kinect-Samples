"""Main application window assembling all annotation tool widgets."""

from pathlib import Path

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QAction, QKeySequence, QShortcut, QUndoStack
from PySide6.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
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
    QPushButton,
)

from .constants import JOINT_NAMES
from .data_model import AnnotationModel
from .undo_commands import (
    MoveJointCommand,
    MoveMultipleJointsCommand,
    ToggleVisibilityCommand,
    BatchVisibilityCommand,
    BatchMultiJointVisibilityCommand,
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

        self._last_batch_dir: str = ""

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

        # Central area: viewport + timeline + interpolation button
        central = QWidget()
        central_layout = QVBoxLayout(central)
        central_layout.setContentsMargins(0, 0, 0, 0)
        central_layout.setSpacing(0)
        central_layout.addWidget(self.viewport, stretch=1)
        timeline_row = QHBoxLayout()
        timeline_row.setContentsMargins(0, 0, 0, 0)
        timeline_row.setSpacing(4)
        timeline_row.addWidget(self.timeline, stretch=1)
        self._vis_btn = QPushButton("Apply to Range (Vis)")
        self._vis_btn.setToolTip(
            "Toggle visibility for selected joint(s) across the active frame range"
        )
        self._vis_btn.setFixedWidth(140)
        self._vis_btn.setEnabled(False)
        self._vis_btn.clicked.connect(self._apply_visibility_range)
        timeline_row.addWidget(self._vis_btn)
        self._interp_btn = QPushButton("Interpolate")
        self._interp_btn.setEnabled(False)
        self._interp_btn.setToolTip(
            "Interpolate selected joint(s) between keyframes (I)"
        )
        self._interp_btn.setFixedWidth(100)
        self._interp_btn.clicked.connect(self._apply_interpolation_auto)
        timeline_row.addWidget(self._interp_btn)
        central_layout.addLayout(timeline_row)
        self.setCentralWidget(central)

        # Right dock: joint tree + HMD info
        right_dock = QDockWidget("Properties", self)
        right_dock.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(2, 2, 2, 2)
        right_layout.addWidget(self.joint_tree, stretch=1)
        right_layout.addWidget(self.hmd_panel, stretch=0)
        self._update_3d_checkbox = QCheckBox("Update 3D on drag")
        self._update_3d_checkbox.setChecked(True)
        self._update_3d_checkbox.setToolTip(
            "When checked, dragging a joint in the ego view also updates\n"
            "its 3D position via IK back-projection (bone-length preserving).\n"
            "Uncheck to modify only the 2D annotation."
        )
        right_layout.addWidget(self._update_3d_checkbox, stretch=0)
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

        # Update 3D toggle
        self._update_3d_checkbox.toggled.connect(
            lambda on: setattr(self.model, 'update_3d', on)
        )

        # Head-joint pruning
        self.prune_checkbox.toggled.connect(lambda on: self.model.set_pruning(on))
        self.model.pruning_changed.connect(self._on_pruning_changed)
        self.model.session_loaded.connect(self._reset_prune_checkbox)

        # Extrinsic panel
        self.extrinsic_panel.extrinsic_changed.connect(self.model.set_extrinsic_delta)
        self.extrinsic_panel.apply_all_clicked.connect(self._apply_extrinsic_all)
        self.extrinsic_panel.apply_range_clicked.connect(self._apply_extrinsic_range)
        self.extrinsic_panel.export_clicked.connect(self._export_extrinsic)
        self.extrinsic_panel.preview_3d_clicked.connect(self._preview_3d)
        self.extrinsic_panel.range_active_changed.connect(self._vis_btn.setEnabled)
        self.extrinsic_panel._direct_proj_checkbox.toggled.connect(
            lambda on: setattr(self.model, 'direct_projection', on)
        )
        self.model.session_loaded.connect(self._on_session_loaded_extrinsic)
        self.model.session_loaded.connect(
            lambda: self.extrinsic_panel.set_frame_count(self.model.frame_count)
        )

        # Interpolation button state tracking
        self.model.frame_changed.connect(
            lambda _: self._update_interpolation_button()
        )
        self.model.joint_selected.connect(
            lambda _: self._update_interpolation_button()
        )
        self.model.session_loaded.connect(self._update_interpolation_button)
        self.undo_stack.indexChanged.connect(
            lambda _: self._update_interpolation_button()
        )

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
        edit_menu.addAction("Toggle &Visibility", self._toggle_visibility, QKeySequence("V"))
        edit_menu.addSeparator()
        edit_menu.addAction("Apply &Interpolation...", self._apply_interpolation)

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
        QShortcut(QKeySequence("I"), self).activated.connect(self._apply_interpolation_auto)
        QShortcut(QKeySequence("R"), self).activated.connect(
            self.extrinsic_panel._on_apply_range
        )

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

        Auto-sets dragged joints as keyframes for interpolation.
        When an extrinsic delta is active (preview only, not applied),
        converts the dragged positions from adjusted space to raw space
        to prevent double-application of the extrinsic offset.

        Args:
            moves: list of (joint_id, old_u, old_v, new_u, new_v)
        """
        frame = self.model.current_frame

        # Convert adjusted-space positions to raw-space when extrinsic
        # delta is active.  The viewport shows joints at adjusted positions,
        # so drag coordinates are in adjusted space.  The model stores raw
        # values, and get_adjusted_joint_2d() re-applies the offset on read.
        if self.model.has_extrinsic_delta():
            raw_moves = []
            for jid, _old_u_adj, _old_v_adj, new_u_adj, new_v_adj in moves:
                raw_old_u, raw_old_v, _, _ = self.model.get_joint_2d(frame, jid)
                adj_u, adj_v, _, _ = self.model.get_adjusted_joint_2d(frame, jid)
                offset_u = adj_u - raw_old_u
                offset_v = adj_v - raw_old_v
                raw_new_u = new_u_adj - offset_u
                raw_new_v = new_v_adj - offset_v
                raw_moves.append((jid, raw_old_u, raw_old_v, raw_new_u, raw_new_v))
            moves = raw_moves

        # Determine which joints need auto-keyframing
        need_kf = [
            m[0] for m in moves
            if frame not in self.model.get_keyframes(m[0])
        ]

        # Use macro to group move + auto-keyframe into one undo step
        if need_kf:
            self.undo_stack.beginMacro(
                f"Move + auto-keyframe @ frame {frame}"
            )

        if len(moves) == 1:
            jid, old_u, old_v, new_u, new_v = moves[0]
            cmd = MoveJointCommand(self.model, frame, jid,
                                   old_u, old_v, new_u, new_v)
        else:
            cmd = MoveMultipleJointsCommand(self.model, frame, moves)
        self.undo_stack.push(cmd)

        for jid in need_kf:
            self.undo_stack.push(
                SetKeyframeCommand(self.model, frame, jid, True)
            )

        if need_kf:
            self.undo_stack.endMacro()

    def _on_visibility_toggled(self, frame, jid, new_vis):
        start, end = self.extrinsic_panel.frame_range()
        last_frame = self.model.frame_count - 1
        is_full = (min(start, end) == 0 and max(start, end) >= last_frame)

        if not is_full:
            changes = []
            for f in range(start, end + 1):
                _, _, _, old_vis = self.model.get_joint_2d(f, jid)
                if old_vis != new_vis:
                    changes.append((f, old_vis, new_vis))
            if changes:
                cmd = BatchVisibilityCommand(self.model, jid, changes)
                self.undo_stack.push(cmd)
                self._status.showMessage(
                    f"{'Show' if new_vis else 'Hide'} joint in {len(changes)} frames ({start}-{end})", 3000
                )
        else:
            _, _, _, old_vis = self.model.get_joint_2d(frame, jid)
            cmd = ToggleVisibilityCommand(self.model, frame, jid, old_vis, new_vis)
            self.undo_stack.push(cmd)

    def _on_keyframe_toggled(self, frame, jid, new_kf):
        cmd = SetKeyframeCommand(self.model, frame, jid, new_kf)
        self.undo_stack.push(cmd)

    def _get_selected_jids(self):
        """Return selected joint IDs from viewport or model fallback."""
        jids = self.viewport.get_selected_joint_ids()
        if not jids:
            jid = self.model.selected_joint
            if jid >= 0:
                jids = [jid]
        return jids

    def _toggle_visibility(self):
        """Toggle visibility for selected joint(s) at the current frame only."""
        jids = self._get_selected_jids()
        if not jids:
            self._status.showMessage("Select a joint first", 3000)
            return

        frame = self.model.current_frame
        any_visible = any(
            self.model.get_joint_2d(frame, jid)[3] for jid in jids
        )
        new_vis = not any_visible

        changes = []
        for jid in jids:
            _, _, _, old_vis = self.model.get_joint_2d(frame, jid)
            if old_vis != new_vis:
                changes.append((frame, jid, old_vis, new_vis))
        if not changes:
            return

        action = "Show" if new_vis else "Hide"
        desc = f"{action} {len(jids)} joint(s) @ frame {frame}"
        cmd = BatchMultiJointVisibilityCommand(self.model, changes, desc)
        self.undo_stack.push(cmd)
        self._status.showMessage(desc, 3000)

    def _apply_visibility_range(self):
        """Toggle visibility for selected joint(s) across the active frame range."""
        jids = self._get_selected_jids()
        if not jids:
            self._status.showMessage("Select a joint first", 3000)
            return

        frame = self.model.current_frame
        start, end = self.extrinsic_panel.frame_range()

        any_visible = any(
            self.model.get_joint_2d(frame, jid)[3] for jid in jids
        )
        new_vis = not any_visible

        changes = []
        for f in range(start, end + 1):
            for jid in jids:
                _, _, _, old_vis = self.model.get_joint_2d(f, jid)
                if old_vis != new_vis:
                    changes.append((f, jid, old_vis, new_vis))
        if not changes:
            return

        action = "Show" if new_vis else "Hide"
        desc = f"{action} {len(jids)} joint(s) frames {start}-{end}"
        cmd = BatchMultiJointVisibilityCommand(self.model, changes, desc)
        self.undo_stack.push(cmd)
        self._status.showMessage(desc, 3000)

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

        # Clear keyframes for interpolated joints
        for jid, kf_frames in eligible.items():
            for f in kf_frames:
                self.undo_stack.push(
                    SetKeyframeCommand(self.model, f, jid, False)
                )
        self.undo_stack.endMacro()

        self._status.showMessage(
            f"Interpolated {len(eligible)} joint(s), "
            f"{total_frames} frames total ({mode})",
            5000,
        )

    def _apply_interpolation_auto(self):
        """Auto-apply linear interpolation to all eligible joints.

        Scans ALL joints (not just viewport selection).  Eligible: joint
        has >= 2 keyframes AND current frame is within their keyframe range.
        No dialog — applies immediately.
        """
        from .constants import NUM_JOINTS

        frame = self.model.current_frame
        eligible = {}  # jid -> sorted keyframe list
        for jid in range(NUM_JOINTS):
            kfs = sorted(self.model.get_keyframes(jid))
            if len(kfs) >= 2 and kfs[0] <= frame <= kfs[-1]:
                eligible[jid] = kfs

        if not eligible:
            return

        total_frames = 0
        self.undo_stack.beginMacro(
            f"Interpolate {len(eligible)} joint(s) (linear)"
        )
        for jid, kf_frames in eligible.items():
            keyframes = []
            for f in kf_frames:
                u, v, _, _ = self.model.get_joint_2d(f, jid)
                keyframes.append((f, u, v))

            interpolated = InterpolationEngine.interpolate_joint(
                keyframes, "linear"
            )
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

        # Clear keyframes for interpolated joints
        for jid, kf_frames in eligible.items():
            for f in kf_frames:
                self.undo_stack.push(
                    SetKeyframeCommand(self.model, f, jid, False)
                )
        self.undo_stack.endMacro()

        self._status.showMessage(
            f"Interpolated {len(eligible)} joint(s), "
            f"{total_frames} frames (linear)",
            5000,
        )

    # ==================================================================
    # Interpolation button state
    # ==================================================================
    def _update_interpolation_button(self):
        """Enable interpolation button when ANY joint has keyframes around
        the current frame position.

        Scans all joints, not just the viewport selection, because dragging
        a different joint deselects the previous one.
        """
        if self.model.dataset is None:
            self._interp_btn.setEnabled(False)
            return

        from .constants import NUM_JOINTS

        frame = self.model.current_frame
        for jid in range(NUM_JOINTS):
            kfs = sorted(self.model.get_keyframes(jid))
            if len(kfs) >= 2 and kfs[0] <= frame <= kfs[-1]:
                self._interp_btn.setEnabled(True)
                return

        self._interp_btn.setEnabled(False)

    # ==================================================================
    # Head-joint pruning
    # ==================================================================
    def _reset_prune_checkbox(self):
        self.prune_checkbox.setChecked(False)

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
            if self.model.intrinsics_from_calibration:
                self._status.showMessage("Intrinsics from camera calibration (direct projection)", 5000)
            else:
                self._status.showMessage("Intrinsics estimated from 3D/2D pairs (differential projection)", 5000)
            # Sync checkbox with model default
            self.extrinsic_panel._direct_proj_checkbox.setChecked(self.model.direct_projection)
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
            "This will transform skeleton_3d and re-project skeleton_2d\n"
            "in all annotation JSONs using the current extrinsic delta.\n\n"
            "Backups (.bak) will be created. Continue?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if ans != QMessageBox.Yes:
            return
        n = self.model.apply_extrinsic_to_all_frames()
        self.undo_stack.clear()
        # Reset sliders to zero so the delta is not double-applied on the
        # now-updated data.  This also clears model._extrinsic_delta via
        # the extrinsic_changed signal.
        self.extrinsic_panel._on_reset()
        self._status.showMessage(
            f"Applied extrinsic delta to {n} frames (.bak backups created)", 5000
        )

    def _apply_extrinsic_range(self, start: int, end: int):
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
            self, "Apply Extrinsic to Range",
            f"This will transform skeleton_3d and re-project skeleton_2d\n"
            f"for frames {start}\u2013{end} ({end - start + 1} frames)\n"
            f"using the current extrinsic delta.\n\n"
            f"Backups (.bak) will be created. Continue?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if ans != QMessageBox.Yes:
            return
        n = self.model.apply_extrinsic_to_all_frames(
            start_frame=start, end_frame=end
        )
        self.undo_stack.clear()
        self.extrinsic_panel._on_reset()
        self._status.showMessage(
            f"Applied extrinsic delta to {n} frames "
            f"(range {start}\u2013{end}, .bak backups created)",
            5000,
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

    def _preview_3d(self):
        if self.model.dataset is None:
            self._status.showMessage("No session loaded", 3000)
            return
        frame = self.model.current_frame
        original_3d = self.model.dataset.get_joints_3d(frame)
        if original_3d is None:
            self._status.showMessage("No 3D skeleton data for this frame", 3000)
            return

        import numpy as np
        from .viz_3d import Skeleton3DDialog

        # Build adjusted 3D array (per-joint edits + extrinsic delta)
        adjusted_3d = np.copy(original_3d)
        for jid in range(len(original_3d)):
            result = self.model.get_effective_joint_3d(frame, jid)
            if result is not None:
                adjusted_3d[jid, 0] = result[0]
                adjusted_3d[jid, 1] = result[1]
                adjusted_3d[jid, 2] = result[2]

        rx, ry, rz, tx, ty, tz = self.model._extrinsic_delta
        n_edits = sum(
            1 for j in range(len(original_3d))
            if self.model.is_joint_edited(frame, j)
        )
        delta_text = (
            f"Frame {frame}  |  "
            f"R({rx:+.2f}\u00b0, {ry:+.2f}\u00b0, {rz:+.2f}\u00b0)  "
            f"T({tx:+.1f}, {ty:+.1f}, {tz:+.1f}) mm"
        )
        if n_edits > 0:
            delta_text += f"  |  {n_edits} joint(s) edited"

        dlg = Skeleton3DDialog(original_3d, adjusted_3d, delta_text, parent=self)
        dlg.show()

    # ==================================================================
    # Session opening / saving
    # ==================================================================
    def _open_session_browser(self):
        if not self._check_unsaved():
            return
        dlg = SessionBrowserDialog(self, initial_dir=self._last_batch_dir)
        if dlg.exec() == QDialog.Accepted:
            # Remember the batch directory for next time
            self._last_batch_dir = dlg._dir_edit.text().strip()
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
        # Remember batch directory from session path for re-opening the browser.
        # ego_dir is like batch_out/SessionName/ego_dataset/ → parent.parent = batch_out/
        ego_path = Path(str(session["ego_dir"]))
        self._last_batch_dir = str(ego_path.parent.parent)
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
        # Remember parent of ego_dataset as batch dir (e.g. batch_out/Session/)
        self._last_batch_dir = str(ego_path.parent.parent)
        synced = ego_path.parent / "synced_data.csv"
        hmd_csv = str(synced) if synced.exists() else None
        self.model.load_session(str(ego_path), hmd_csv=hmd_csv)

    def open_batch_session(self, batch_dir: str, session_name: str):
        """Load a specific session from a batch directory (called from CLI)."""
        self._last_batch_dir = batch_dir
        from .constants import discover_batch_sessions
        sessions = discover_batch_sessions(Path(batch_dir))
        for s in sessions:
            if s["name"] == session_name:
                self._load_session(s)
                return
        raise ValueError(f"Session '{session_name}' not found in {batch_dir}")
