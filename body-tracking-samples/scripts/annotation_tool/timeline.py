"""Timeline widget: color bar + slider + frame label."""

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen, QMouseEvent
from PySide6.QtWidgets import QWidget, QSlider, QLabel, QHBoxLayout, QVBoxLayout

from .data_model import AnnotationModel


class TimelineColorBar(QWidget):
    """Thin painted bar showing per-frame status colors.

    Colors (highest priority first):
        blue   -- keyframe
        orange -- edited
        green  -- checkerboard detected
        gray   -- normal
    """

    frame_clicked = Signal(int)

    def __init__(self, model: AnnotationModel, parent=None):
        super().__init__(parent)
        self.model = model
        self.setFixedHeight(14)
        self.setMinimumWidth(100)
        self.setCursor(Qt.PointingHandCursor)

    def paintEvent(self, event):
        n = self.model.frame_count
        if n == 0:
            return
        painter = QPainter(self)
        w = self.width()
        h = self.height()

        for i in range(n):
            x1 = int(i * w / n)
            x2 = int((i + 1) * w / n)
            if x2 <= x1:
                x2 = x1 + 1

            if self.model.has_keyframe_at(i):
                color = QColor(70, 130, 230)    # blue
            elif self.model.is_frame_edited(i):
                color = QColor(230, 150, 50)    # orange
            else:
                fd = self.model.get_frame_data(i)
                if fd and fd.get("checkerboard_detected", False):
                    color = QColor(50, 180, 80)  # green
                else:
                    color = QColor(160, 160, 160)  # gray

            painter.fillRect(x1, 0, x2 - x1, h, color)

        # Current frame indicator
        if n > 0:
            cx = int(self.model.current_frame * w / max(n - 1, 1))
            painter.setPen(QPen(Qt.white, 2))
            painter.drawLine(cx, 0, cx, h)
            painter.setPen(QPen(Qt.black, 1))
            painter.drawLine(cx + 1, 0, cx + 1, h)

        painter.end()

    def mousePressEvent(self, event: QMouseEvent):
        n = self.model.frame_count
        if n == 0:
            return
        frame = int(event.position().x() * n / max(self.width(), 1))
        frame = max(0, min(frame, n - 1))
        self.model.current_frame = frame
        self.frame_clicked.emit(frame)

    def mouseMoveEvent(self, event: QMouseEvent):
        if event.buttons() & Qt.LeftButton:
            self.mousePressEvent(event)


class TimelineWidget(QWidget):
    """Composite widget: color bar + slider + frame counter."""

    def __init__(self, model: AnnotationModel, parent=None):
        super().__init__(parent)
        self.model = model

        self._color_bar = TimelineColorBar(model)
        self._slider = QSlider(Qt.Horizontal)
        self._slider.setMinimum(0)
        self._slider.setMaximum(0)
        self._slider.setSingleStep(1)
        self._slider.setPageStep(10)

        self._label = QLabel("Frame 0 / 0")
        self._label.setFixedWidth(130)
        self._label.setAlignment(Qt.AlignCenter)

        slider_row = QHBoxLayout()
        slider_row.setContentsMargins(0, 0, 0, 0)
        slider_row.addWidget(self._slider, stretch=1)
        slider_row.addWidget(self._label)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(2)
        layout.addWidget(self._color_bar)
        layout.addLayout(slider_row)

        # Connections
        self._slider.valueChanged.connect(self._on_slider_changed)
        model.session_loaded.connect(self._on_session_loaded)
        model.frame_changed.connect(self._on_frame_changed)
        model.joint_moved.connect(lambda *_: self._color_bar.update())

    # ---- slots -------------------------------------------------------
    def _on_session_loaded(self):
        n = self.model.frame_count
        self._slider.setMaximum(max(n - 1, 0))
        self._slider.setValue(0)
        self._update_label(0)
        self._color_bar.update()

    def _on_frame_changed(self, frame: int):
        self._slider.blockSignals(True)
        self._slider.setValue(frame)
        self._slider.blockSignals(False)
        self._update_label(frame)
        self._color_bar.update()

    def _on_slider_changed(self, value: int):
        self.model.current_frame = value

    def _update_label(self, frame: int):
        self._label.setText(f"Frame {frame} / {max(self.model.frame_count - 1, 0)}")
