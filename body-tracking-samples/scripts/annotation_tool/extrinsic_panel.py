"""Extrinsic fine-tuning panel with 6DOF sliders for calibration adjustment."""

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QGroupBox,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QSlider,
    QDoubleSpinBox,
    QPushButton,
)


class _SliderRow(QHBoxLayout):
    """Label + slider + spin-box row for one DOF parameter."""

    value_changed = Signal(float)

    def __init__(
        self,
        label: str,
        min_val: float,
        max_val: float,
        step: float,
        decimals: int,
        suffix: str,
        parent_signal,
    ):
        super().__init__()
        self._step = step
        self._suppress = False

        # Label
        lbl = QLabel(label)
        lbl.setFixedWidth(22)
        self.addWidget(lbl)

        # Slider (integer ticks mapped to float via step)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(int(min_val / step))
        self.slider.setMaximum(int(max_val / step))
        self.slider.setValue(0)
        self.slider.setTickInterval(max(1, int((max_val - min_val) / step / 20)))
        self.addWidget(self.slider, stretch=1)

        # Spin box
        self.spin = QDoubleSpinBox()
        self.spin.setRange(min_val, max_val)
        self.spin.setSingleStep(step)
        self.spin.setDecimals(decimals)
        self.spin.setSuffix(suffix)
        self.spin.setValue(0.0)
        self.spin.setFixedWidth(90)
        self.addWidget(self.spin)

        # Keep slider <-> spin in sync
        self.slider.valueChanged.connect(self._slider_to_spin)
        self.spin.valueChanged.connect(self._spin_to_slider)

        # Forward to parent
        self._parent_signal = parent_signal

    def _slider_to_spin(self, tick: int):
        if self._suppress:
            return
        val = tick * self._step
        self._suppress = True
        self.spin.setValue(val)
        self._suppress = False
        self._parent_signal()

    def _spin_to_slider(self, val: float):
        if self._suppress:
            return
        tick = int(round(val / self._step))
        self._suppress = True
        self.slider.setValue(tick)
        self._suppress = False
        self._parent_signal()

    def value(self) -> float:
        return self.spin.value()

    def reset(self):
        self._suppress = True
        self.slider.setValue(0)
        self.spin.setValue(0.0)
        self._suppress = False


class ExtrinsicPanel(QGroupBox):
    """Panel with 6 sliders for fine-tuning the T_checker_to_A extrinsic.

    Signals
    -------
    extrinsic_changed(rx, ry, rz, tx, ty, tz)
        Emitted on any slider/spin change.
    apply_all_clicked()
        User pressed "Apply to All Frames".
    export_clicked()
        User pressed "Export T_checker_to_A...".
    reset_clicked()
        User pressed "Reset".
    """

    extrinsic_changed = Signal(float, float, float, float, float, float)
    apply_all_clicked = Signal()
    export_clicked = Signal()
    reset_clicked = Signal()
    preview_3d_clicked = Signal()

    def __init__(self, parent=None):
        super().__init__("Extrinsic Tuning", parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 10, 6, 6)
        layout.setSpacing(4)

        # -- Rotation sliders (degrees) --
        layout.addWidget(QLabel("Rotation (deg)"))
        self._rx = _SliderRow("rx", -10.0, 10.0, 0.05, 2, "\u00b0", self._emit)
        self._ry = _SliderRow("ry", -10.0, 10.0, 0.05, 2, "\u00b0", self._emit)
        self._rz = _SliderRow("rz", -10.0, 10.0, 0.05, 2, "\u00b0", self._emit)
        layout.addLayout(self._rx)
        layout.addLayout(self._ry)
        layout.addLayout(self._rz)

        # -- Translation sliders (mm) --
        layout.addWidget(QLabel("Translation (mm)"))
        self._tx = _SliderRow("tx", -200.0, 200.0, 0.5, 1, " mm", self._emit)
        self._ty = _SliderRow("ty", -200.0, 200.0, 0.5, 1, " mm", self._emit)
        self._tz = _SliderRow("tz", -200.0, 200.0, 0.5, 1, " mm", self._emit)
        layout.addLayout(self._tx)
        layout.addLayout(self._ty)
        layout.addLayout(self._tz)

        # -- Buttons --
        btn_row = QHBoxLayout()
        self._reset_btn = QPushButton("Reset")
        self._apply_btn = QPushButton("Apply to All")
        self._export_btn = QPushButton("Export...")
        btn_row.addWidget(self._reset_btn)
        btn_row.addWidget(self._apply_btn)
        btn_row.addWidget(self._export_btn)
        layout.addLayout(btn_row)

        btn_row2 = QHBoxLayout()
        self._preview_3d_btn = QPushButton("Preview 3D")
        self._preview_3d_btn.setToolTip(
            "Show original vs adjusted 3D skeleton for the current frame"
        )
        btn_row2.addWidget(self._preview_3d_btn)
        btn_row2.addStretch()
        layout.addLayout(btn_row2)

        # Connect buttons
        self._reset_btn.clicked.connect(self._on_reset)
        self._apply_btn.clicked.connect(self.apply_all_clicked)
        self._export_btn.clicked.connect(self.export_clicked)
        self._preview_3d_btn.clicked.connect(self.preview_3d_clicked)

    def _emit(self):
        self.extrinsic_changed.emit(
            self._rx.value(),
            self._ry.value(),
            self._rz.value(),
            self._tx.value(),
            self._ty.value(),
            self._tz.value(),
        )

    def _on_reset(self):
        self._rx.reset()
        self._ry.reset()
        self._rz.reset()
        self._tx.reset()
        self._ty.reset()
        self._tz.reset()
        self._emit()
        self.reset_clicked.emit()

    def values(self):
        """Return current (rx, ry, rz, tx, ty, tz) tuple."""
        return (
            self._rx.value(),
            self._ry.value(),
            self._rz.value(),
            self._tx.value(),
            self._ty.value(),
            self._tz.value(),
        )
