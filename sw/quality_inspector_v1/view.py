"""Small, presentation-only Qt window for the quality inspector."""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QFrame, QGridLayout, QHBoxLayout, QLabel,
    QMainWindow, QPushButton, QScrollArea, QSizePolicy, QSplitter,
    QToolButton, QVBoxLayout, QWidget, QTextEdit,
)

ITEM_KEYS = ("geometry", "sharpness", "performance", "edges", "gamma", "cnn", "resources", "latency")
TITLES = {
    "geometry": "校正精度", "sharpness": "清晰度", "performance": "帧率与掉帧", "edges": "边缘与视野",
    "gamma": "Gamma", "cnn": "CNN", "resources": "资源与时序", "latency": "硬件延迟",
}
DEFAULT_SELECTED = {"geometry", "sharpness", "performance", "edges", "gamma"}
STATUS_TEXT = {
    "not_measured": "未测", "running": "测量中", "measured": "已测",
    "condition_required": "需条件", "error": "错误", "cancelled": "已取消",
}


class ResultRow(QFrame):
    def __init__(self, key: str):
        super().__init__()
        self.key = key
        self.setObjectName("resultRow")
        self.setFrameShape(QFrame.Shape.StyledPanel)
        layout = QGridLayout(self)
        layout.setContentsMargins(8, 5, 8, 5)
        layout.setHorizontalSpacing(8)
        self.checkbox = QCheckBox()
        self.checkbox.setChecked(key in DEFAULT_SELECTED)
        self.checkbox.setToolTip(f"将{TITLES[key]}加入所选测量")
        self.title_label = QLabel(TITLES[key])
        self.title_label.setMinimumWidth(54)
        self.title_label.setStyleSheet("font-weight: 600")
        self.status_label = QLabel("未测")
        self.status_label.setObjectName("status")
        self.status_label.setMinimumWidth(48)
        self.value_label = QLabel("尚无测量")
        self.value_label.setWordWrap(True)
        self.value_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self.value_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.action_button = QPushButton("导入" if key=="resources" else "测量")
        self.action_button.setFixedWidth(58)
        self.detail_button=QPushButton('结果')
        self.detail_button.setFixedWidth(50)
        self.detail_button.setEnabled(False)
        layout.addWidget(self.checkbox, 0, 0)
        layout.addWidget(self.title_label, 0, 1)
        layout.addWidget(self.status_label, 0, 2)
        layout.addWidget(self.value_label, 0, 3)
        layout.addWidget(self.action_button, 0, 4)
        layout.addWidget(self.detail_button,0,5)
        layout.setColumnStretch(3, 1)

    def update_result(self, row: dict):
        status = str(row.get("status", "not_measured"))
        self.status_label.setText(STATUS_TEXT.get(status, status))
        value = row.get("value", "")
        detail = row.get("detail", "")
        if row.get('source')=='historical' and status!='not_measured':
            self.status_label.setText(STATUS_TEXT.get(status,status)+' · 历史')
        self.value_label.setText(str(value) if value not in (None, "") else STATUS_TEXT.get(status, status))
        self.value_label.setToolTip(str(detail))
        self.setToolTip(str(detail))
        self.detail_button.setEnabled(status not in ('not_measured','running'))


class InspectorWindow(QMainWindow):
    connect_requested = Signal(str, str)
    disconnect_requested = Signal()
    run_requested = Signal(object)
    stop_requested = Signal()
    open_requested = Signal()
    export_requested = Signal()
    resources_requested = Signal()
    latency_requested = Signal()
    details_requested=Signal(str)
    latency_help_requested=Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("FPGA 画质检测")
        self.setMinimumSize(1000, 700)
        self.resize(1280, 820)
        self._connected = False
        self._busy = False
        self._session = {}
        self._build_ui()
        self._style()

    def _build_ui(self):
        root = QWidget()
        outer = QVBoxLayout(root)
        outer.setContentsMargins(16, 12, 16, 12)
        outer.setSpacing(10)
        self.setCentralWidget(root)

        top = QHBoxLayout()
        brand = QLabel("FPGA 画质检测")
        brand.setStyleSheet("font-size: 20px; font-weight: 700; color: #143c42")
        top.addWidget(brand)
        top.addStretch(1)
        top.addWidget(QLabel("端口"))
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.addItem("COM14")
        self.port_combo.setFixedWidth(100)
        top.addWidget(self.port_combo)
        top.addWidget(QLabel("视频设备"))
        self.device_combo = QComboBox()
        self.device_combo.setEditable(True)
        self.device_combo.addItem("USB Video")
        self.device_combo.setMinimumWidth(190)
        top.addWidget(self.device_combo)
        self.connection_button = QPushButton("连接")
        self.connection_button.clicked.connect(self._toggle_connection)
        top.addWidget(self.connection_button)
        outer.addLayout(top)
        self.connection_label = QLabel("未连接 · 连接仅执行诊断")
        self.connection_label.setStyleSheet("color:#61777a")
        outer.addWidget(self.connection_label)

        split = QSplitter(Qt.Orientation.Horizontal)
        split.setChildrenCollapsible(False)
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 6, 0)
        self.preview_title = QLabel("实时画面")
        self.preview_title.setStyleSheet("font-size:15px;font-weight:600")
        left_layout.addWidget(self.preview_title)
        self.preview_label = QLabel("等待视频画面")
        self.preview_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.preview_label.setMinimumSize(360, 240)
        self.preview_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.preview_label.setStyleSheet("background:#101719;color:#b9c9c9;border-radius:8px")
        left_layout.addWidget(self.preview_label, 1)
        self.preview_condition_label = QLabel("放稳棋盘格并避开反光；自动保存校正前→校正后→重复基线。清晰度需斜放约5°；结束保留真实标定0°。")
        self.preview_condition_label.setWordWrap(True)
        self.preview_condition_label.setStyleSheet("color:#9a5f16;background:#fff5df;padding:8px;border-radius:6px")
        left_layout.addWidget(self.preview_condition_label)
        self.overview_label = QLabel("尚未开始测量")
        self.overview_label.setWordWrap(True)
        self.overview_label.setStyleSheet("color:#52696b;padding:8px")
        left_layout.addWidget(self.overview_label)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(6, 0, 0, 0)
        right_layout.setSpacing(8)
        list_head = QHBoxLayout()
        title = QLabel("测量项目")
        title.setStyleSheet("font-size:16px;font-weight:700")
        list_head.addWidget(title)
        list_head.addStretch(1)
        list_head.addWidget(QLabel("性能时长"))
        self.duration_combo = QComboBox()
        self.duration_combo.addItems(["10 秒", "60 秒", "120 秒"])
        self.duration_combo.setFixedWidth(90)
        list_head.addWidget(self.duration_combo)
        right_layout.addLayout(list_head)
        latency_options=QHBoxLayout()
        latency_options.addWidget(QLabel('延迟每角度帧数'))
        self.latency_samples_combo=QComboBox();self.latency_samples_combo.addItems(['300','1000'])
        latency_options.addWidget(self.latency_samples_combo)
        latency_options.addStretch(1)
        self.latency_csv_button=QPushButton('导入外部延迟CSV')
        self.latency_csv_button.clicked.connect(self.latency_requested.emit)
        latency_options.addWidget(self.latency_csv_button)
        right_layout.addLayout(latency_options)
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        rows_host = QWidget()
        self.rows_layout = QVBoxLayout(rows_host)
        self.rows_layout.setContentsMargins(0, 0, 0, 0)
        self.rows_layout.setSpacing(6)
        self.rows = {}
        for key in ITEM_KEYS:
            row = self.rows[key] = ResultRow(key)
            self.rows_layout.addWidget(row)
            row.action_button.clicked.connect(lambda checked=False, item=key: self._run_item(item))
            row.detail_button.clicked.connect(lambda checked=False,item=key:self.details_requested.emit(item))
        self.rows_layout.addStretch(1)
        self.scroll_area.setWidget(rows_host)
        right_layout.addWidget(self.scroll_area, 1)
        actions = QHBoxLayout()
        self.start_button = QPushButton("一键测量")
        self.start_button.setObjectName("primary")
        self.start_button.clicked.connect(lambda: self.run_requested.emit(self.selected_keys()))
        self.stop_button = QPushButton("停止")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.stop_requested.emit)
        actions.addWidget(self.start_button, 1)
        actions.addWidget(self.stop_button)
        right_layout.addLayout(actions)
        split.addWidget(left)
        split.addWidget(right)
        split.setStretchFactor(0, 5)
        split.setStretchFactor(1, 6)
        outer.addWidget(split, 1)

        bottom = QHBoxLayout()
        self.open_button = QPushButton("打开已有测量")
        self.open_button.clicked.connect(self.open_requested.emit)
        self.export_button = QPushButton("导出报告")
        self.export_button.clicked.connect(self.export_requested.emit)
        bottom.addWidget(self.open_button)
        bottom.addWidget(self.export_button)
        self.comparison_button=QPushButton('查看前后对比')
        self.comparison_button.setEnabled(False)
        self.comparison_button.clicked.connect(lambda:self.details_requested.emit('comparison'))
        bottom.addWidget(self.comparison_button)
        self.latency_help_button=QPushButton('延迟怎么测')
        self.latency_help_button.clicked.connect(self.latency_help_requested.emit)
        bottom.addWidget(self.latency_help_button)
        bottom.addStretch(1)
        outer.addLayout(bottom)
        self.log_toggle = QToolButton()
        self.log_toggle.setText("日志 ▸")
        self.log_toggle.setCheckable(True)
        self.log_toggle.toggled.connect(self._toggle_logs)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumHeight(100)
        self.log_view.hide()
        outer.addWidget(self.log_toggle)
        outer.addWidget(self.log_view)

    def _style(self):
        self.setStyleSheet("""
            QMainWindow, QWidget { background: #f6f9f8; color:#20383a; }
            QComboBox, QPushButton { background:white; border:1px solid #cbd9d7; border-radius:6px; padding:7px 10px; }
            QPushButton:hover { border-color:#56a99f; }
            QPushButton:disabled { color:#9aabaa; background:#edf2f1; }
            QPushButton#primary { color:white; background:#167e78; border:0; font-weight:700; padding:10px; }
            QFrame#resultRow { background:white; border:1px solid #dce7e5; border-radius:7px; }
            QLabel#status { color:#167e78; font-weight:600; }
            QScrollArea { border:0; background:transparent; }
        """)

    def _toggle_connection(self):
        if self._connected:
            self.disconnect_requested.emit()
        else:
            self.connect_requested.emit(self.port_combo.currentText(), self.device_combo.currentText())

    def _run_item(self, key):
        if key == "resources":
            self.resources_requested.emit()
        elif key == "latency":
            self.run_requested.emit([key])
        else:
            self.run_requested.emit([key])

    def _toggle_logs(self, shown):
        self.log_view.setVisible(shown)
        self.log_toggle.setText("日志 ▾" if shown else "日志 ▸")

    def set_busy(self, busy: bool, message: str = ""):
        self._busy = bool(busy)
        self.start_button.setEnabled(not busy)
        self.connection_button.setEnabled(not busy)
        self.open_button.setEnabled(not busy)
        self.export_button.setEnabled(not busy)
        self.latency_csv_button.setEnabled(not busy)
        self.latency_samples_combo.setEnabled(not busy)
        for row in self.rows.values():
            row.action_button.setEnabled(not busy)
        self.stop_button.setEnabled(bool(busy))
        if message:
            self.overview_label.setText(str(message))

    def set_connected(self, connected: bool, detail: str = ""):
        self._connected = bool(connected)
        self.connection_button.setText("断开" if connected else "连接")
        self.connection_label.setText(detail or ("已连接" if connected else "未连接 · 连接仅执行诊断"))

    def set_preview(self, image: QImage):
        copied = image.copy()
        self._preview_image=copied
        pixmap = QPixmap.fromImage(copied)
        self.preview_label.setPixmap(pixmap.scaled(self.preview_label.size(), Qt.AspectRatioMode.KeepAspectRatio,
                                                   Qt.TransformationMode.SmoothTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        image=getattr(self,'_preview_image',None)
        pixmap=QPixmap.fromImage(image) if image is not None else None
        if pixmap is not None:
            self.preview_label.setPixmap(pixmap.scaled(self.preview_label.size(), Qt.AspectRatioMode.KeepAspectRatio,
                                                       Qt.TransformationMode.SmoothTransformation))

    def set_item(self, key: str, rowdict: dict):
        if key in self.rows:
            self.rows[key].update_result(dict(rowdict))

    def set_overview(self, overview: dict):
        self.overview_label.setText(str(overview.get("message", overview.get("summary", overview))))

    def append_log(self, message: str):
        self.log_view.append(str(message))

    def set_session(self, session: dict):
        self._session = dict(session)
        for key, row in self._session.get("results", {}).items():
            self.set_item(key, row)
        self.comparison_button.setEnabled(any(PathLikeName.endswith('before_after_comparison.png')
            for row in self._session.get('results',{}).values() for PathLikeName in map(str,row.get('evidence',[]))))

    def duration_seconds(self) -> int:
        return (10, 60, 120)[self.duration_combo.currentIndex()]

    def selected_keys(self) -> list[str]:
        return [key for key in ITEM_KEYS if self.rows[key].checkbox.isChecked()]
