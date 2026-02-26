"""Session browser dialog for opening batch_out/ directories."""

from pathlib import Path
from typing import Optional

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDialog,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QFileDialog,
    QHeaderView,
    QAbstractItemView,
)

from .constants import discover_batch_sessions


class SessionBrowserDialog(QDialog):
    """Dialog for choosing a session from a batch output directory."""

    def __init__(self, parent=None, initial_dir: str = ""):
        super().__init__(parent)
        self.setWindowTitle("Open Session")
        self.setMinimumSize(750, 450)
        self._sessions: list = []

        layout = QVBoxLayout(self)

        # Directory picker row
        dir_row = QHBoxLayout()
        dir_row.addWidget(QLabel("Batch Directory:"))
        self._dir_edit = QLineEdit(initial_dir)
        dir_row.addWidget(self._dir_edit, stretch=1)
        self._browse_btn = QPushButton("Browse...")
        dir_row.addWidget(self._browse_btn)
        self._scan_btn = QPushButton("Scan")
        dir_row.addWidget(self._scan_btn)
        layout.addLayout(dir_row)

        # Session table
        self._table = QTableWidget(0, 4)
        self._table.setHorizontalHeaderLabels(["Session", "Frames", "Status", "HMD"])
        self._table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._table.setSelectionMode(QAbstractItemView.SingleSelection)
        self._table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        header = self._table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        for c in (1, 2, 3):
            header.setSectionResizeMode(c, QHeaderView.ResizeMode.ResizeToContents)
        layout.addWidget(self._table)

        # Buttons
        btn_row = QHBoxLayout()
        btn_row.addStretch()
        self._open_btn = QPushButton("Open")
        self._open_btn.setDefault(True)
        self._open_btn.setEnabled(False)
        self._cancel_btn = QPushButton("Cancel")
        btn_row.addWidget(self._open_btn)
        btn_row.addWidget(self._cancel_btn)
        layout.addLayout(btn_row)

        # Connections
        self._browse_btn.clicked.connect(self._browse)
        self._scan_btn.clicked.connect(self._scan)
        self._open_btn.clicked.connect(self.accept)
        self._cancel_btn.clicked.connect(self.reject)
        self._table.doubleClicked.connect(self.accept)
        self._table.itemSelectionChanged.connect(self._on_selection_changed)

        # Auto-scan if initial_dir given
        if initial_dir:
            self._scan()

    # ---- slots -------------------------------------------------------
    def _browse(self):
        d = QFileDialog.getExistingDirectory(self, "Select Batch Directory",
                                             self._dir_edit.text())
        if d:
            self._dir_edit.setText(d)
            self._scan()

    def _scan(self):
        dir_path = self._dir_edit.text().strip()
        if not dir_path or not Path(dir_path).is_dir():
            return
        sessions = discover_batch_sessions(Path(dir_path))
        self._sessions = sessions
        self._table.setRowCount(len(sessions))
        for i, s in enumerate(sessions):
            self._table.setItem(i, 0, QTableWidgetItem(s["name"]))
            ann_dir = s["ego_dir"] / "annotations"
            n_frames = len(list(ann_dir.glob("frame_*.json"))) if ann_dir.exists() else 0
            self._table.setItem(i, 1, QTableWidgetItem(str(n_frames)))
            self._table.setItem(i, 2, QTableWidgetItem(s.get("status", "?")))
            hmd = "Yes" if s.get("synced_csv") else "No"
            self._table.setItem(i, 3, QTableWidgetItem(hmd))
        self._open_btn.setEnabled(False)

    def _on_selection_changed(self):
        self._open_btn.setEnabled(self._table.currentRow() >= 0)

    # ---- result ------------------------------------------------------
    def selected_session(self) -> Optional[dict]:
        row = self._table.currentRow()
        if 0 <= row < len(self._sessions):
            return self._sessions[row]
        return None
