"""3D skeleton comparison dialog for verifying extrinsic adjustments."""

from __future__ import annotations

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QSlider,
)

from .constants import (
    BONE_CONNECTIONS,
    EXCLUDED_JOINTS,
    PART_COLORS_RGB,
    get_joint_part,
    get_bone_part,
    NUM_JOINTS,
    JOINT_NAMES,
)

try:
    from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.figure import Figure

    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


class Skeleton3DDialog(QDialog):
    """Side-by-side 3D skeleton plot: original (blue) vs adjusted (red).

    Requires matplotlib.  If matplotlib is not installed the dialog shows an
    error message instead.
    """

    def __init__(
        self,
        original_3d: np.ndarray,
        adjusted_3d: np.ndarray,
        delta_text: str = "",
        parent=None,
    ):
        super().__init__(parent)
        self.setWindowTitle("3D Extrinsic Preview")
        self.resize(900, 700)
        layout = QVBoxLayout(self)

        if not HAS_MATPLOTLIB:
            layout.addWidget(
                QLabel(
                    "matplotlib is required for 3D preview.\n"
                    "Install with: pip install matplotlib"
                )
            )
            return

        fig = Figure(figsize=(10, 7), dpi=100)
        self._canvas = FigureCanvas(fig)
        layout.addWidget(self._canvas)

        if delta_text:
            lbl = QLabel(delta_text)
            lbl.setAlignment(Qt.AlignCenter)
            layout.addWidget(lbl)

        self._ax = self._draw(fig, original_3d, adjusted_3d)

        # --- View angle sliders ---
        # Axis mapping is right-handed: plot_X=cam_X, plot_Y=cam_Z, plot_Z=-cam_Y
        # Default viewpoint looks from behind the camera along +Z (depth):
        #   azim=-90 → viewer at -plot_Y (= -cam_Z = behind the camera)
        #   elev=20  → slightly above the horizontal plane
        # This makes +X go RIGHT and -cam_Y (up) go UP, matching the ego image.
        default_elev = 20
        default_azim = -90

        elev_row = QHBoxLayout()
        elev_row.addWidget(QLabel("Elevation:"))
        self._elev_slider = QSlider(Qt.Horizontal)
        self._elev_slider.setRange(-90, 90)
        self._elev_slider.setValue(default_elev)
        self._elev_label = QLabel(f"{default_elev}\u00b0")
        self._elev_label.setFixedWidth(40)
        elev_row.addWidget(self._elev_slider, stretch=1)
        elev_row.addWidget(self._elev_label)
        layout.addLayout(elev_row)

        azim_row = QHBoxLayout()
        azim_row.addWidget(QLabel("Azimuth:"))
        self._azim_slider = QSlider(Qt.Horizontal)
        self._azim_slider.setRange(-180, 180)
        self._azim_slider.setValue(default_azim)
        self._azim_label = QLabel(f"{default_azim}\u00b0")
        self._azim_label.setFixedWidth(40)
        azim_row.addWidget(self._azim_slider, stretch=1)
        azim_row.addWidget(self._azim_label)
        layout.addLayout(azim_row)

        self._elev_slider.valueChanged.connect(self._on_view_changed)
        self._azim_slider.valueChanged.connect(self._on_view_changed)

        self._ax.view_init(elev=default_elev, azim=default_azim)
        self._canvas.draw()

    # ------------------------------------------------------------------ #
    def _on_view_changed(self):
        elev = self._elev_slider.value()
        azim = self._azim_slider.value()
        self._elev_label.setText(f"{elev}\u00b0")
        self._azim_label.setText(f"{azim}\u00b0")
        self._ax.view_init(elev=elev, azim=azim)
        self._canvas.draw_idle()

    # ------------------------------------------------------------------ #
    @staticmethod
    def _cam_to_plot(arr, jid):
        """Map camera coords (X-right, Y-down, Z-forward) to plot coords.

        Right-handed mapping that keeps matplotlib rendering consistent:
          plot_X = cam_X   (horizontal, left-right)
          plot_Y = cam_Z   (horizontal, depth)
          plot_Z = -cam_Y  (vertical, up)
        """
        return arr[jid, 0], arr[jid, 2], -arr[jid, 1]

    def _draw(self, fig: Figure, orig: np.ndarray, adj: np.ndarray):
        ax = fig.add_subplot(111, projection="3d")

        min_conf = 1
        _c2p = self._cam_to_plot

        # --- Bones ---------------------------------------------------- #
        for pid, cid in BONE_CONNECTIONS:
            if pid in EXCLUDED_JOINTS or cid in EXCLUDED_JOINTS:
                continue
            # Original (semi-transparent blue)
            if orig[pid, 3] >= min_conf and orig[cid, 3] >= min_conf:
                px1, py1, pz1 = _c2p(orig, pid)
                px2, py2, pz2 = _c2p(orig, cid)
                ax.plot(
                    [px1, px2], [py1, py2], [pz1, pz2],
                    color="steelblue", alpha=0.35, linewidth=1.5,
                )
            # Adjusted (solid red)
            if adj[pid, 3] >= min_conf and adj[cid, 3] >= min_conf:
                px1, py1, pz1 = _c2p(adj, pid)
                px2, py2, pz2 = _c2p(adj, cid)
                ax.plot(
                    [px1, px2], [py1, py2], [pz1, pz2],
                    color="tomato", alpha=0.85, linewidth=2,
                )

        # --- Joints --------------------------------------------------- #
        for jid in range(NUM_JOINTS):
            if jid in EXCLUDED_JOINTS:
                continue
            if orig[jid, 3] >= min_conf:
                px, py, pz = _c2p(orig, jid)
                ax.scatter(
                    px, py, pz,
                    c="steelblue", s=18, alpha=0.4, edgecolors="white",
                    linewidths=0.3, depthshade=True,
                )
            if adj[jid, 3] >= min_conf:
                px, py, pz = _c2p(adj, jid)
                ax.scatter(
                    px, py, pz,
                    c="tomato", s=28, alpha=0.85, edgecolors="white",
                    linewidths=0.3, depthshade=True,
                )

        # --- Displacement arrows -------------------------------------- #
        for jid in range(NUM_JOINTS):
            if jid in EXCLUDED_JOINTS:
                continue
            if orig[jid, 3] < min_conf or adj[jid, 3] < min_conf:
                continue
            ox, oy, oz = _c2p(orig, jid)
            ax_, ay, az = _c2p(adj, jid)
            dx, dy, dz = ax_ - ox, ay - oy, az - oz
            dist = np.sqrt(dx * dx + dy * dy + dz * dz)
            if dist > 0.5:  # only show arrows > 0.5 mm
                ax.quiver(
                    ox, oy, oz, dx, dy, dz,
                    color="limegreen", alpha=0.6, arrow_length_ratio=0.15,
                    linewidth=1.0,
                )

        # --- Axes / legend -------------------------------------------- #
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Depth (mm)")
        ax.set_zlabel("Up (mm)")
        ax.set_title("Original (blue)  vs  Adjusted (red)")

        # Auto-range from valid joints
        all_pts = []
        for jid in range(NUM_JOINTS):
            if jid in EXCLUDED_JOINTS:
                continue
            for arr in (orig, adj):
                if arr[jid, 3] >= min_conf:
                    all_pts.append(list(_c2p(arr, jid)))
        if all_pts:
            pts = np.array(all_pts)
            center = np.median(pts, axis=0)
            half = max(np.ptp(pts, axis=0).max() / 2, 200)
            ax.set_xlim(center[0] - half, center[0] + half)
            ax.set_ylim(center[1] - half, center[1] + half)
            ax.set_zlim(center[2] - half, center[2] + half)

        from matplotlib.lines import Line2D

        legend = [
            Line2D([0], [0], color="steelblue", alpha=0.5, label="Original"),
            Line2D([0], [0], color="tomato", alpha=0.85, label="Adjusted"),
            Line2D([0], [0], color="limegreen", alpha=0.6, label="Displacement"),
        ]
        ax.legend(handles=legend, loc="upper left", fontsize=8)
        fig.tight_layout()

        return ax
