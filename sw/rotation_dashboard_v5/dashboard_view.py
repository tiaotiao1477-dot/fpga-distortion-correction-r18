"""Pure PySide6 dashboard view for the FPGA rotation console.

The coordinator owns all UART, video acquisition and correction work.  This
module only emits the v2-compatible signals and paints coordinator-provided
state on the Qt GUI thread.
"""

from __future__ import annotations

import math
from typing import Any

from PySide6.QtCore import QRectF, QSignalBlocker, Qt, QTimer, Signal
from PySide6.QtGui import QImage, QPainter, QPixmap
from PySide6.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSlider,
    QToolButton,
    QVBoxLayout,
    QWidget,
)


class _PreviewCanvas(QWidget):
    """Paints received frames in fit or native-pixel inspection mode."""

    framePainted = Signal(object)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._pixmap: QPixmap | None = None
        self._frame_identity = None
        self._last_painted_identity = None
        self._view_mode = "fit"
        self._source_origin = (0, 0)
        self._drag_position = None
        self._stale_message: str | None = "等待视频"
        self.setMinimumSize(320, 180)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent)

    def set_view_mode(self, mode: str) -> None:
        self._view_mode = mode
        self._source_origin = self._centered_origin()
        self.update()

    def view_mode(self) -> str:
        return self._view_mode

    def _centered_origin(self) -> tuple[int, int]:
        if self._pixmap is None or self._view_mode != "native":
            return (0, 0)
        dpr = max(0.01, float(self.devicePixelRatioF()))
        view_w = max(1, round(self.width() * dpr))
        view_h = max(1, round(self.height() * dpr))
        return (max(0, (self._pixmap.width() - view_w) // 2),
                max(0, (self._pixmap.height() - view_h) // 2))

    def _clamp_origin(self, x: int, y: int) -> tuple[int, int]:
        if self._pixmap is None:
            return (0, 0)
        dpr = max(0.01, float(self.devicePixelRatioF()))
        max_x = max(0, self._pixmap.width() - round(self.width() * dpr))
        max_y = max(0, self._pixmap.height() - round(self.height() * dpr))
        return (min(max(0, x), max_x), min(max(0, y), max_y))

    def set_frame(self, image: QImage, identity: object = None) -> None:
        had_frame = self._pixmap is not None
        self._pixmap = QPixmap.fromImage(image)
        self._frame_identity = identity
        if self._view_mode == "native":
            if not had_frame:
                self._source_origin = self._centered_origin()
            else:
                self._source_origin = self._clamp_origin(*self._source_origin)
        self.update()

    def set_live(self, live: bool, message: str = "等待视频") -> None:
        next_message = None if live else message
        if next_message != self._stale_message:
            self._stale_message = next_message
            self.update()

    def paintEvent(self, event: Any) -> None:  # noqa: N802 - Qt override
        painter = QPainter(self)
        painter.fillRect(self.rect(), Qt.GlobalColor.black)
        if self._pixmap is not None and not self._pixmap.isNull():
            if self._view_mode == "native":
                dpr = max(0.01, float(self.devicePixelRatioF()))
                source = QRectF(self._source_origin[0], self._source_origin[1],
                                min(self._pixmap.width(), round(self.width() * dpr)),
                                min(self._pixmap.height(), round(self.height() * dpr)))
                target_w = source.width() / dpr
                target_h = source.height() / dpr
                target = QRectF(round((self.width() - target_w) * dpr / 2) / dpr,
                                round((self.height() - target_h) * dpr / 2) / dpr,
                                target_w, target_h)
                painter.drawPixmap(target, self._pixmap, source)
            else:
                scaled = self._pixmap.scaled(
                    self.size(),
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation,
                )
                x = (self.width() - scaled.width()) // 2
                y = (self.height() - scaled.height()) // 2
                painter.drawPixmap(x, y, scaled)
        if self._stale_message is not None:
            painter.fillRect(self.rect(), Qt.GlobalColor.black)
            painter.setPen(Qt.GlobalColor.white)
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, self._stale_message)
        painted_identity = (self._frame_identity if self._stale_message is None
                            and self._pixmap is not None and not self._pixmap.isNull() else None)
        painter.end()
        if painted_identity is not None and painted_identity != self._last_painted_identity:
            self._last_painted_identity = painted_identity
            self.framePainted.emit(painted_identity)

    def mousePressEvent(self, event: Any) -> None:  # noqa: N802
        if self._view_mode == "native" and event.button() == Qt.MouseButton.LeftButton:
            self._drag_position = event.position()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: Any) -> None:  # noqa: N802
        if self._drag_position is not None:
            delta = event.position() - self._drag_position
            dpr = max(0.01, float(self.devicePixelRatioF()))
            self._source_origin = self._clamp_origin(
                self._source_origin[0] - round(delta.x() * dpr),
                self._source_origin[1] - round(delta.y() * dpr),
            )
            self._drag_position = event.position()
            self.update()
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: Any) -> None:  # noqa: N802
        if self._drag_position is not None and event.button() == Qt.MouseButton.LeftButton:
            self._drag_position = None
            self.unsetCursor()
            event.accept()
            return
        super().mouseReleaseEvent(event)


class DashboardWindow(QWidget):
    """Coordinator-facing presentation layer; contains no device operations."""

    # These signals and public methods intentionally retain the v2 contract.
    connectRequested = Signal(str)
    angleRequested = Signal(str)
    correctRequested = Signal()
    previewRetryRequested = Signal()
    closingRequested = Signal()
    cropRequested = Signal(bool)
    framePainted = Signal(object)
    monitorRefreshRequested = Signal()
    modelLoadRequested = Signal()
    modelEnableRequested = Signal(bool)
    modelPrepareRequested = Signal()
    modelKRequested = Signal()
    monitorContextRequested = Signal()
    automaticCorrectionRequested = Signal(bool)

    _COLORS = {
        "bg": "#f3f7f7",
        "panel": "#ffffff",
        "panel_alt": "#f7faf9",
        "line": "#dce6e6",
        "text": "#1c3045",
        "muted": "#6d8190",
        "accent": "#087e91",
        "accent_hover": "#096b7a",
        "danger": "#c24d56",
        "success": "#14835f",
    }

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._allow_close = False
        self._connected = False
        self._control_ready = False
        self._prepare_available = False
        self._video_stalled = False
        self._busy = False
        self._angle_known = False
        self._preview_live = False
        self._crop_enabled = False
        self._crop_known = False
        self._crop_request_target: bool | None = None
        self._monitor_context_qualified = False
        self._monitor_busy = False
        self._ack_angle: float | None = None
        self._pending_angle: float | None = None
        self._target_dirty = False
        self._slider_sync = False
        self._debounce = QTimer(self)
        self._debounce.setSingleShot(True)
        self._debounce.setInterval(180)
        self._debounce.timeout.connect(self._submit_current_angle)

        self.setWindowTitle("FPGA 图像控制台")
        self.setMinimumSize(1000, 700)
        self.resize(1280, 800)
        self.setObjectName("dashboardWindow")
        self._build_ui()
        self._apply_style()
        self._refresh_controls()
        self._refresh_target_status()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(24, 20, 24, 14)
        root.setSpacing(14)

        header = QHBoxLayout()
        title_box = QVBoxLayout()
        title_box.setSpacing(3)
        title = QLabel("图像控制台")
        title.setObjectName("pageTitle")
        subtitle = QLabel("FPGA 图像输出 · CNN 自动校正与 Gamma 监测")
        subtitle.setObjectName("subtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header.addLayout(title_box)
        header.addStretch(1)
        self._top_status = QLabel("●  未连接")
        self._top_status.setObjectName("connectionBadge")
        self._preview_status = QLabel("●  等待预览")
        self._preview_status.setObjectName("previewBadge")
        header.addWidget(self._top_status, 0, Qt.AlignmentFlag.AlignVCenter)
        header.addWidget(self._preview_status, 0, Qt.AlignmentFlag.AlignVCenter)
        root.addLayout(header)

        body = QHBoxLayout()
        body.setSpacing(16)
        root.addLayout(body, 1)

        preview_panel = QFrame()
        preview_panel.setObjectName("panel")
        preview_layout = QVBoxLayout(preview_panel)
        preview_layout.setContentsMargins(16, 15, 16, 14)
        preview_layout.setSpacing(10)

        preview_heading = QHBoxLayout()
        preview_title = QLabel("FPGA 输出预览")
        preview_title.setObjectName("sectionTitle")
        self._preview_badge = QLabel("等待视频")
        self._preview_badge.setObjectName("inlineBadge")
        preview_heading.addWidget(preview_title)
        preview_heading.addStretch(1)
        preview_heading.addWidget(self._preview_badge)
        preview_layout.addLayout(preview_heading)

        view_controls = QHBoxLayout()
        self._fit_button = QToolButton()
        self._fit_button.setText("适合窗口")
        self._fit_button.setCheckable(True)
        self._fit_button.setChecked(True)
        self._fit_button.setObjectName("viewButton")
        self._native_button = QToolButton()
        self._native_button.setText("原始像素 1:1")
        self._native_button.setToolTip("仅改变本窗口的显示比例：1 个显示像素对应 1 个视频像素，不改变 FPGA 输出；窗口放不下时可拖动画面查看。")
        self._native_button.setCheckable(True)
        self._native_button.setObjectName("viewButton")
        self._fit_button.clicked.connect(lambda checked=False: self._set_view_mode("fit"))
        self._native_button.clicked.connect(lambda checked=False: self._set_view_mode("native"))
        view_controls.addWidget(self._fit_button)
        view_controls.addWidget(self._native_button)
        view_controls.addStretch(1)
        self._crop_checkbox = QCheckBox("旋转去边")
        self._crop_checkbox.setObjectName("cropCheck")
        self._crop_checkbox.setToolTip("请求板卡启用或关闭旋转去边参数。勾选状态只在板卡确认后更新。")
        self._crop_checkbox.clicked.connect(self._request_crop)
        view_controls.addWidget(self._crop_checkbox)
        preview_layout.addLayout(view_controls)

        self._canvas = _PreviewCanvas()
        self._canvas.setObjectName("previewCanvas")
        self._canvas.framePainted.connect(self.framePainted)
        preview_layout.addWidget(self._canvas, 1)

        preview_footer = QHBoxLayout()
        self._preview_detail = QLabel("视频尚未启动")
        self._preview_detail.setWordWrap(True)
        self._preview_detail.setObjectName("mutedText")
        self._crop_detail = QLabel("去边状态待确认")
        self._crop_detail.setWordWrap(True)
        self._crop_detail.setObjectName("mutedText")
        self._retry_button = QPushButton("重试预览")
        self._retry_button.setObjectName("secondaryButton")
        self._retry_button.clicked.connect(lambda checked=False: self.previewRetryRequested.emit())
        preview_footer.addWidget(self._preview_detail, 1)
        preview_footer.addWidget(self._crop_detail, 1)
        preview_footer.addWidget(self._retry_button)
        preview_layout.addLayout(preview_footer)
        body.addWidget(preview_panel, 1)

        self._sidebar = QFrame()
        self._sidebar.setObjectName("panel")
        self._sidebar.setMinimumWidth(320)
        self._sidebar.setMaximumWidth(344)
        side = QVBoxLayout(self._sidebar)
        side.setContentsMargins(12, 12, 12, 11)
        side.setSpacing(4)
        self._sidebar_scroll = QScrollArea()
        self._sidebar_scroll.setObjectName("sidebarScroll")
        self._sidebar_scroll.setWidgetResizable(True)
        self._sidebar_scroll.setFrameShape(QFrame.Shape.NoFrame)
        self._sidebar_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self._sidebar_scroll.setWidget(self._sidebar)
        self._sidebar_scroll.setFixedWidth(360)
        body.addWidget(self._sidebar_scroll)

        side_title = QLabel("控制面板")
        side_title.setObjectName("sectionTitle")
        side.addWidget(side_title)

        side.addWidget(self._field_label("串口"))
        self._port_input = QLineEdit("COM14")
        self._port_input.setObjectName("field")
        self._port_input.setPlaceholderText("例如 COM14")
        self._port_input.setClearButtonEnabled(True)
        side.addWidget(self._port_input)
        self._connect_button = QPushButton("连接控制板")
        self._connect_button.setObjectName("primaryButton")
        self._connect_button.clicked.connect(lambda checked=False: self._emit_connect())
        side.addWidget(self._connect_button)

        self._separator(side)
        angle_heading = QHBoxLayout()
        angle_heading.addWidget(self._field_label("目标角度"))
        angle_heading.addStretch(1)
        direction = QLabel("绝对角度 · 逆时针为正")
        direction.setObjectName("mutedText")
        angle_heading.addWidget(direction)
        side.addLayout(angle_heading)

        angle_entry = QHBoxLayout()
        self._angle_input = QDoubleSpinBox()
        self._angle_input.setObjectName("angleInput")
        self._angle_input.setRange(-180.0, 180.0)
        self._angle_input.setDecimals(2)
        self._angle_input.setSingleStep(0.01)
        self._angle_input.setSuffix("°")
        self._angle_input.setKeyboardTracking(True)
        self._angle_input.setAlignment(Qt.AlignmentFlag.AlignRight)
        self._angle_input.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
        self._angle_input.valueChanged.connect(self._on_numeric_angle_changed)
        self._angle_input.editingFinished.connect(self._submit_if_angle_dirty)
        angle_entry.addWidget(self._angle_input, 1)
        self._step_down = QPushButton("−1°")
        self._step_down.setObjectName("stepButton")
        self._step_down.clicked.connect(lambda checked=False: self._step_angle(-1.0))
        self._step_up = QPushButton("+1°")
        self._step_up.setObjectName("stepButton")
        self._step_up.clicked.connect(lambda checked=False: self._step_angle(1.0))
        angle_entry.addWidget(self._step_down)
        angle_entry.addWidget(self._step_up)
        side.addLayout(angle_entry)

        self._angle_slider = QSlider(Qt.Orientation.Horizontal)
        self._angle_slider.setObjectName("angleSlider")
        self._angle_slider.setRange(-1800, 1800)
        self._angle_slider.setSingleStep(1)
        self._angle_slider.setPageStep(10)
        self._angle_slider.setTracking(True)
        self._angle_slider.valueChanged.connect(self._on_slider_angle_changed)
        side.addWidget(self._angle_slider)
        self._target_status = QLabel("调整后发送目标角度")
        self._target_status.setObjectName("targetStatus")
        self._target_status.setWordWrap(True)
        self._target_status.setFixedHeight(32)
        side.addWidget(self._target_status)

        apply_row = QHBoxLayout()
        self._apply_button = QPushButton("应用角度")
        self._apply_button.setObjectName("primaryButton")
        self._apply_button.clicked.connect(lambda checked=False: self._submit_current_angle())
        apply_row.addWidget(self._apply_button, 1)
        side.addLayout(apply_row)

        quick_label = self._field_label("快速设定")
        side.addWidget(quick_label)
        quick_grid = QGridLayout()
        quick_grid.setHorizontalSpacing(7)
        quick_grid.setVerticalSpacing(7)
        self._quick_buttons: list[QPushButton] = []
        for index, value in enumerate((0, 15, -15, 45, -45, 90, -90)):
            text = "0°" if value == 0 else f"{value:+d}°"
            button = QPushButton(text)
            button.setObjectName("quickButton")
            button.setMinimumHeight(26)
            button.clicked.connect(lambda checked=False, v=value: self._submit_angle(float(v)))
            quick_grid.addWidget(button, index // 4, index % 4)
            self._quick_buttons.append(button)
        side.addLayout(quick_grid)

        self._separator(side)
        self._effective_caption = QLabel("已生效角度 · 板卡确认")
        self._effective_caption.setObjectName("mutedText")
        side.addWidget(self._effective_caption)
        self._effective_angle = QLabel("—")
        self._effective_angle.setObjectName("ackAngle")
        side.addWidget(self._effective_angle)
        self._control_status = QLabel("控制板未连接")
        self._control_status.setObjectName("controlStatus")
        self._control_status.setWordWrap(True)
        self._control_status.setFixedHeight(34)
        side.addWidget(self._control_status)

        action_row = QHBoxLayout()
        self._correct_button = QPushButton("自动摆正")
        self._correct_button.setObjectName("primaryButton")
        self._correct_button.clicked.connect(lambda checked=False: self.correctRequested.emit())
        self._restore_button = QPushButton("恢复 0°")
        self._restore_button.setObjectName("secondaryButton")
        self._restore_button.clicked.connect(lambda checked=False: self._submit_angle(0.0))
        action_row.addWidget(self._correct_button, 1)
        action_row.addWidget(self._restore_button)
        side.addLayout(action_row)
        self._build_monitor_summary(side)
        side.addStretch(1)

        self._notice = QFrame()
        self._notice.setObjectName("notice")
        notice_layout = QHBoxLayout(self._notice)
        notice_layout.setContentsMargins(13, 9, 13, 9)
        self._notice_mark = QLabel("●")
        self._notice_mark.setObjectName("noticeMark")
        notice_layout.addWidget(self._notice_mark, 0, Qt.AlignmentFlag.AlignTop)
        notice_text = QVBoxLayout()
        notice_text.setSpacing(2)
        self._notice_title = QLabel("就绪")
        self._notice_title.setObjectName("noticeTitle")
        self._notice_detail = QLabel("连接控制板并启动视频预览。")
        self._notice_detail.setObjectName("noticeDetail")
        self._notice_detail.setWordWrap(True)
        notice_text.addWidget(self._notice_title)
        notice_text.addWidget(self._notice_detail)
        notice_layout.addLayout(notice_text, 1)
        root.addWidget(self._notice)

        log_header = QHBoxLayout()
        self._log_toggle = QToolButton()
        self._log_toggle.setText("▸  操作记录")
        self._log_toggle.setCheckable(True)
        self._log_toggle.setChecked(False)
        self._log_toggle.setObjectName("logToggle")
        self._log_toggle.toggled.connect(self._toggle_log)
        log_header.addWidget(self._log_toggle)
        log_header.addStretch(1)
        root.addLayout(log_header)
        self._log = QPlainTextEdit()
        self._log.setObjectName("logView")
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(500)
        self._log.setFixedHeight(110)
        self._log.hide()
        root.addWidget(self._log)

    def _build_monitor_summary(self, side: QVBoxLayout) -> None:
        """Add compact status to the rotation page and collapse technical details."""
        self._auto_available = False
        self._auto_availability_reason = "模型未加载 / 尚未就绪"
        self._auto_running = False
        summary = QFrame()
        summary.setObjectName("monitorSummary")
        summary_layout = QVBoxLayout(summary)
        summary_layout.setContentsMargins(9, 7, 9, 7)
        summary_layout.setSpacing(3)
        summary_title = QLabel("算法状态")
        summary_title.setObjectName("sectionTitle")
        summary_layout.addWidget(summary_title)
        self._auto_correction = QCheckBox("CNN 自动校正")
        self._auto_correction.setObjectName("autoCorrection")
        self._auto_correction.setChecked(False)
        self._auto_correction.setFixedHeight(25)
        self._auto_correction.toggled.connect(self.automaticCorrectionRequested.emit)
        summary_layout.addWidget(self._auto_correction)
        self._auto_reason = QLabel(self._auto_availability_reason)
        self._auto_reason.setObjectName("autoReason")
        self._auto_reason.setFixedHeight(19)
        summary_layout.addWidget(self._auto_reason)
        self._cnn_prepare_button = QPushButton("准备并启用 CNN")
        self._cnn_prepare_button.setObjectName("secondaryButton")
        self._cnn_prepare_button.setToolTip("首次加载模型需要几分钟；完成后再开启自动校正。")
        self._cnn_prepare_button.clicked.connect(self.modelPrepareRequested.emit)
        summary_layout.addWidget(self._cnn_prepare_button)
        self._cnn_summary = QLabel("CNN：未知")
        self._cnn_summary.setObjectName("summaryValue")
        self._cnn_summary.setFixedHeight(19)
        summary_layout.addWidget(self._cnn_summary)
        self._gamma_summary = QLabel("Gamma：未知")
        self._gamma_summary.setObjectName("summaryValue")
        self._gamma_summary.setFixedHeight(19)
        summary_layout.addWidget(self._gamma_summary)
        side.addWidget(summary)

        self._monitor_details_toggle = QToolButton()
        self._monitor_details_toggle.setText("▸  详细状态")
        self._monitor_details_toggle.setObjectName("detailsToggle")
        self._monitor_details_toggle.setCheckable(True)
        self._monitor_details_toggle.setChecked(False)
        self._monitor_details_toggle.setFixedHeight(27)
        self._monitor_details_toggle.toggled.connect(self._toggle_monitor_details)
        side.addWidget(self._monitor_details_toggle)
        self._monitor_details = QFrame()
        self._monitor_details.setObjectName("monitorDetails")
        detail_layout = QVBoxLayout(self._monitor_details)
        detail_layout.setContentsMargins(8, 7, 8, 7)
        detail_layout.setSpacing(5)
        self._monitor_details.hide()

        self._monitor_freshness = QLabel("● 未知 · 尚无设备监测快照")
        self._monitor_freshness.setObjectName("monitorStatus")
        self._monitor_freshness.setWordWrap(True)
        detail_layout.addWidget(self._monitor_freshness)
        self._auto_correction_detail = QLabel("")
        self._auto_correction_detail.setObjectName("mutedText")
        self._auto_correction_detail.setWordWrap(True)
        detail_layout.addWidget(self._auto_correction_detail)
        self._monitor_context_detail = QLabel("")
        self._monitor_context_detail.setObjectName("mutedText")
        self._monitor_context_detail.setWordWrap(True)
        detail_layout.addWidget(self._monitor_context_detail)

        self._monitor_refresh_button = QPushButton("刷新设备监测")
        self._monitor_refresh_button.setObjectName("secondaryButton")
        self._monitor_refresh_button.clicked.connect(self.monitorRefreshRequested.emit)
        detail_layout.addWidget(self._monitor_refresh_button)

        self._cnn_status = self._metric_card(detail_layout, "CNN 状态")
        self._cnn_bytes = self._metric_card(detail_layout, "模型字节 / CRC32")
        self._cnn_ready = self._metric_card(detail_layout, "启用 / K 有效 / 忙碌 / 错误")
        self._cnn_counts = self._metric_card(detail_layout, "预测数 / 速率 / 周期")
        self._cnn_prediction = self._metric_card(detail_layout, "预测角度 / 四角坐标")
        self._cnn_angle_note = QLabel("开启自动校正后，稳定预测可按设置下发；实际生效以板卡 ACK 为准。")
        self._cnn_angle_note.setObjectName("mutedText")
        self._cnn_angle_note.setWordWrap(True)
        detail_layout.addWidget(self._cnn_angle_note)

        self._gamma_status = self._metric_card(detail_layout, "Gamma 状态 / 自适应")
        self._gamma_applied = self._metric_card(detail_layout, "实际应用 LUT")
        self._gamma_next = self._metric_card(detail_layout, "下一帧候选 LUT")
        self._gamma_mean = self._metric_card(detail_layout, "有效像素均值 / 数量")
        self._gamma_frame = self._metric_card(detail_layout, "来源帧 / 配置版本")
        bank_help = QLabel("LUT：0 强提亮 · 1 提亮 · 2 直通 · 3 压亮 · 4 强压亮")
        bank_help.setObjectName("mutedText")
        bank_help.setWordWrap(True)
        detail_layout.addWidget(bank_help)

        controls = QGridLayout()
        controls.setHorizontalSpacing(6)
        controls.setVerticalSpacing(6)
        self._monitor_context_button = QPushButton("检查控制资格")
        self._monitor_context_button.setObjectName("secondaryButton")
        self._monitor_context_button.clicked.connect(self.monitorContextRequested.emit)
        self._model_load_button = QPushButton("加载模型")
        self._model_load_button.setObjectName("secondaryButton")
        self._model_load_button.clicked.connect(self.modelLoadRequested.emit)
        self._model_k_button = QPushButton("配置 K")
        self._model_k_button.setObjectName("secondaryButton")
        self._model_k_button.clicked.connect(self.modelKRequested.emit)
        self._model_enable_button = QPushButton("启用 CNN")
        self._model_enable_button.setObjectName("primaryButton")
        self._model_enable_button.clicked.connect(lambda: self.modelEnableRequested.emit(True))
        self._model_disable_button = QPushButton("禁用 CNN")
        self._model_disable_button.setObjectName("secondaryButton")
        self._model_disable_button.clicked.connect(lambda: self.modelEnableRequested.emit(False))
        controls.addWidget(self._monitor_context_button, 0, 0, 1, 2)
        controls.addWidget(self._model_load_button, 1, 0)
        controls.addWidget(self._model_k_button, 1, 1)
        controls.addWidget(self._model_enable_button, 2, 0)
        controls.addWidget(self._model_disable_button, 2, 1)
        detail_layout.addLayout(controls)
        side.addWidget(self._monitor_details)

        self.set_auto_availability(False, self._auto_availability_reason)
        self.set_monitor_state({"freshness": "unknown", "detail": "尚无设备监测快照"})
        self._refresh_monitor_controls()

    def _toggle_monitor_details(self, expanded: bool) -> None:
        self._monitor_details.setVisible(expanded)
        self._monitor_details_toggle.setText("▾  详细状态" if expanded else "▸  详细状态")

    @staticmethod
    def _metric_card(layout: QVBoxLayout, caption: str) -> QLabel:
        frame = QFrame()
        frame.setObjectName("monitorCard")
        card_layout = QVBoxLayout(frame)
        card_layout.setContentsMargins(10, 7, 10, 7)
        card_layout.setSpacing(2)
        label = QLabel(caption)
        label.setObjectName("fieldLabel")
        value = QLabel("未知")
        value.setObjectName("monitorValue")
        value.setWordWrap(True)
        card_layout.addWidget(label)
        card_layout.addWidget(value)
        layout.addWidget(frame)
        return value

    def _field_label(self, text: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName("fieldLabel")
        return label

    @staticmethod
    def _separator(layout: QVBoxLayout) -> None:
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setObjectName("separator")
        layout.addWidget(line)

    def _apply_style(self) -> None:
        c = self._COLORS
        self.setStyleSheet(f"""
            QWidget#dashboardWindow {{ background: {c['bg']}; color: {c['text']};
                font-family: 'Segoe UI', 'Microsoft YaHei UI'; font-size: 13px; }}
            QLabel {{ color: {c['text']}; }}
            QFrame#panel {{ background: {c['panel']}; border: 1px solid {c['line']}; border-radius: 14px; }}
            QScrollArea#sidebarScroll {{ background: transparent; border: 0; }}
            QScrollArea#sidebarScroll QWidget#qt_scrollarea_viewport {{ background: transparent; }}
            QScrollArea#monitorScroll, QWidget#monitorViewport, QWidget#monitorPage {{
                background: {c['panel']}; color: {c['text']}; }}
            QFrame#monitorSummary {{ background: {c['panel_alt']}; border: 1px solid {c['line']};
                border-radius: 8px; }}
            QFrame#monitorDetails {{ background: {c['panel_alt']}; border: 1px solid {c['line']};
                border-radius: 8px; }}
            QLabel#summaryValue {{ color: {c['muted']}; font-size: 11px; }}
            QLabel#autoReason {{ color: {c['muted']}; font-size: 11px; }}
            QCheckBox#autoCorrection {{ color: {c['text']}; spacing: 6px; font-weight: 650; }}
            QCheckBox#autoCorrection:disabled {{ color: #87969c; }}
            QToolButton#detailsToggle {{ color: {c['muted']}; border: 0; padding: 4px 2px;
                font-weight: 600; text-align: left; }}
            QFrame#monitorCard {{ background: {c['panel_alt']}; border: 1px solid {c['line']};
                border-radius: 8px; }}
            QLabel#monitorValue {{ font-size: 12px; font-weight: 600; }}
            QLabel#monitorStatus {{ color: {c['muted']}; font-size: 11px; font-weight: 650; }}
            QLabel#pageTitle {{ font-size: 25px; font-weight: 700; color: {c['text']}; }}
            QLabel#subtitle, QLabel#mutedText {{ color: {c['muted']}; }}
            QLabel#subtitle {{ font-size: 12px; }}
            QLabel#sectionTitle {{ font-size: 15px; font-weight: 650; }}
            QLabel#connectionBadge, QLabel#previewBadge, QLabel#inlineBadge {{
                color: {c['muted']}; background: {c['panel']}; padding: 6px 10px;
                border: 1px solid {c['line']}; border-radius: 10px; font-size: 11px; font-weight: 600; }}
            QLabel#inlineBadge {{ color: {c['accent']}; background: #edf8f7; border-color: #d2e9e7; }}
            QWidget#previewCanvas {{ background: #101820; border: 1px solid #ccd8da; border-radius: 10px; }}
            QLabel#fieldLabel {{ color: {c['muted']}; font-size: 11px; font-weight: 650; }}
            QLineEdit#field, QDoubleSpinBox#angleInput {{ color: {c['text']}; background: {c['panel_alt']};
                border: 1px solid {c['line']}; border-radius: 8px; padding: 7px 10px; }}
            QLineEdit#field:focus, QDoubleSpinBox#angleInput:focus {{ border-color: {c['accent']}; }}
            QLineEdit#field:disabled, QDoubleSpinBox#angleInput:disabled, QSlider:disabled {{
                color: #9baab0; background: #eef2f2; }}
            QPushButton {{ border: 0; border-radius: 8px; padding: 7px 11px; font-weight: 600; }}
            QPushButton#primaryButton {{ background: {c['accent']}; color: white; }}
            QPushButton#primaryButton:hover {{ background: {c['accent_hover']}; }}
            QPushButton#secondaryButton, QPushButton#stepButton {{ background: #f4f8f7; color: {c['text']};
                border: 1px solid {c['line']}; }}
            QPushButton#secondaryButton:hover, QPushButton#stepButton:hover,
            QPushButton#quickButton:hover {{ background: #eaf2f1; }}
            QPushButton#quickButton {{ background: #f7faf9; color: {c['text']};
                border: 1px solid {c['line']}; padding: 2px 4px; }}
            QPushButton#primaryButton:disabled, QPushButton#secondaryButton:disabled,
            QPushButton#quickButton:disabled, QPushButton#stepButton:disabled {{
                color: #9baab0; background: #eef2f2; border-color: #e0e8e8; }}
            QLabel#ackAngle {{ font-size: 27px; font-weight: 700; color: {c['accent']}; }}
            QLabel#targetStatus {{ color: {c['muted']}; font-size: 11px; }}
            QLabel#controlStatus {{ color: {c['muted']}; font-size: 12px; }}
            QFrame#separator {{ color: {c['line']}; max-height: 1px; border: 0; background: {c['line']}; }}
            QFrame#notice {{ background: #eef7f6; border: 1px solid #d9e9e8; border-radius: 10px; }}
            QLabel#noticeMark, QLabel#noticeTitle {{ font-weight: 700; color: {c['accent']}; }}
            QLabel#noticeDetail {{ color: #516977; }}
            QToolButton#viewButton, QToolButton#logToggle {{ color: {c['muted']}; border: 1px solid transparent;
                border-radius: 7px; padding: 6px 9px; font-weight: 600; }}
            QToolButton#viewButton:checked {{ color: {c['accent']}; background: #edf8f7;
                border-color: #d2e9e7; }}
            QCheckBox#cropCheck {{ color: {c['text']}; spacing: 6px; }}
            QCheckBox#cropCheck:disabled {{ color: #9baab0; }}
            QSlider#angleSlider::groove:horizontal {{ height: 5px; background: #dbe5e5; border-radius: 2px; }}
            QSlider#angleSlider::sub-page:horizontal {{ background: #76b8b8; border-radius: 2px; }}
            QSlider#angleSlider::handle:horizontal {{ width: 16px; margin: -6px 0; border-radius: 8px;
                background: {c['accent']}; border: 2px solid white; }}
            QPlainTextEdit#logView {{ color: #465e6d; background: {c['panel_alt']};
                border: 1px solid {c['line']}; border-radius: 8px; padding: 7px;
                font-family: Consolas, monospace; font-size: 11px; }}
        """)

    @staticmethod
    def _format_angle(value: float) -> str:
        rounded = round(float(value), 2)
        if abs(rounded) < 0.005:
            rounded = 0.0
        return f"{rounded:.2f}".rstrip("0").rstrip(".")

    def _emit_connect(self) -> None:
        self.connectRequested.emit(self._port_input.text().strip())

    def _emit_angle(self) -> None:
        self._submit_if_angle_dirty()

    def _set_target_widgets(self, value: float) -> None:
        self._slider_sync = True
        spin_blocker = QSignalBlocker(self._angle_input)
        slider_blocker = QSignalBlocker(self._angle_slider)
        self._angle_input.setValue(max(-180.0, min(180.0, float(value))))
        self._angle_slider.setValue(round(max(-180.0, min(180.0, float(value))) * 10))
        del spin_blocker
        del slider_blocker
        self._slider_sync = False

    def _on_numeric_angle_changed(self, value: float) -> None:
        if self._slider_sync:
            return
        self._slider_sync = True
        blocker = QSignalBlocker(self._angle_slider)
        self._angle_slider.setValue(round(value * 10))
        del blocker
        self._slider_sync = False
        self._target_dirty = self._ack_angle is None or abs(math.remainder(value - self._ack_angle, 360.0)) > 0.004
        if self._connected and self._target_dirty:
            self._debounce.start()
        else:
            self._debounce.stop()
        self._refresh_target_status()

    def _on_slider_angle_changed(self, slider_value: int) -> None:
        if self._slider_sync:
            return
        value = slider_value / 10.0
        self._slider_sync = True
        blocker = QSignalBlocker(self._angle_input)
        self._angle_input.setValue(value)
        del blocker
        self._slider_sync = False
        self._target_dirty = self._ack_angle is None or abs(math.remainder(value - self._ack_angle, 360.0)) > 0.004
        if self._connected and self._target_dirty:
            self._debounce.start()
        else:
            self._debounce.stop()
        self._refresh_target_status()

    def _step_angle(self, delta: float) -> None:
        value = max(-180.0, min(180.0, self._angle_input.value() + delta))
        self._set_target_widgets(value)
        self._target_dirty = self._ack_angle is None or abs(math.remainder(value - self._ack_angle, 360.0)) > 0.004
        self._submit_angle(value)

    def _submit_current_angle(self) -> None:
        self._submit_angle(self._angle_input.value())

    def _submit_if_angle_dirty(self) -> None:
        if self._target_dirty:
            self._submit_current_angle()

    def _submit_angle(self, value: float) -> None:
        self._debounce.stop()
        value = max(-180.0, min(180.0, float(value)))
        self._set_target_widgets(value)
        self._target_dirty = False
        self._refresh_target_status()
        if not self._connected or not self._control_ready:
            return
        self._pending_angle = value
        self._refresh_target_status()
        self.angleRequested.emit(self._format_angle(value))

    def _refresh_target_status(self) -> None:
        value = self._angle_input.value()
        if not self._connected:
            detail = "请先连接控制板，再发送目标角度。"
        elif not self._control_ready:
            detail = "参数状态需重新确认，请点击“重新连接”。"
        elif self._debounce.isActive():
            detail = "等待角度调整完成…"
        elif self._pending_angle is not None:
            pending = self._format_angle(self._pending_angle)
            if abs(math.remainder(value - self._pending_angle, 360.0)) <= 0.004:
                detail = f"目标 {pending}° · 等待应用或板卡确认"
            else:
                detail = f"待处理 {pending}° · 当前输入 {self._format_angle(value)}° 尚未确认"
        elif self._target_dirty:
            detail = "目标角度与上次板卡确认值不同 · 尚未确认"
        elif self._ack_angle is not None and abs(math.remainder(value - self._ack_angle, 360.0)) <= 0.004:
            detail = "目标角度与上次板卡确认值一致"
        else:
            detail = "此目标角度尚无板卡确认"
        self._target_status.setText(detail)

    def _refresh_controls(self) -> None:
        reconnect = self._connected and not self._control_ready
        self._port_input.setEnabled(not self._busy and (not self._connected or reconnect))
        self._connect_button.setEnabled(not self._busy and (not self._connected or reconnect))
        self._connect_button.setText("重新连接" if reconnect else "已连接" if self._connected else "连接控制板")

        # Angle inputs deliberately remain usable while a command is in flight.
        writable = self._connected and self._control_ready
        self._cnn_prepare_button.setEnabled(writable and self._prepare_available and not self._busy and not self._monitor_busy)
        self._angle_input.setEnabled(writable)
        self._angle_slider.setEnabled(writable)
        self._step_down.setEnabled(writable)
        self._step_up.setEnabled(writable)
        self._apply_button.setEnabled(writable)
        for button in self._quick_buttons:
            button.setEnabled(writable)
        self._restore_button.setEnabled(writable)

        self._correct_button.setEnabled(
            writable and not self._busy and self._preview_live
        )
        self._retry_button.setEnabled(not self._busy)
        # Crop requests can be queued by the coordinator while another command runs.
        self._crop_checkbox.setEnabled(writable and self._angle_known and self._crop_known)
        self._top_status.setText("●  待恢复" if reconnect else "●  已连接" if self._connected else "●  未连接")
        color = "#a76a00" if reconnect else self._COLORS["success"] if self._connected else self._COLORS["muted"]
        self._top_status.setStyleSheet(f"color: {color};")
        self._preview_status.setText("●  处理帧已停止" if self._video_stalled else "●  实时预览" if self._preview_live else "●  预览未启动")
        self._preview_status.setStyleSheet(
            f"color: {self._COLORS['danger'] if self._video_stalled else self._COLORS['success'] if self._preview_live else self._COLORS['muted']};"
        )
        if self._video_stalled:
            self._preview_badge.setText('处理帧已停止')
        self._refresh_target_status()

    def _set_view_mode(self, mode: str) -> None:
        self._canvas.set_view_mode(mode)
        self._fit_button.setChecked(mode == "fit")
        self._native_button.setChecked(mode == "native")

    def view_mode(self) -> str:
        return self._canvas.view_mode()

    def _request_crop(self, checked: bool) -> None:
        # Keep the checkmark on the last coordinator-confirmed state until ACK.
        target = not (self._crop_enabled if self._crop_request_target is None
                      else self._crop_request_target)
        self._crop_request_target = target
        blocker = QSignalBlocker(self._crop_checkbox)
        self._crop_checkbox.setChecked(self._crop_enabled)
        del blocker
        if not self._connected or not self._crop_known:
            return
        self.cropRequested.emit(target)

    def set_crop_state(self, enabled: bool | None, detail: str,
                       pending: bool | None = None) -> None:
        """Reflect coordinator-confirmed crop state and its status detail."""
        self._crop_known = enabled is not None
        if enabled is not None:
            self._crop_enabled = bool(enabled)
        self._crop_request_target = None if pending is None else bool(pending)
        blocker = QSignalBlocker(self._crop_checkbox)
        self._crop_checkbox.setChecked(self._crop_enabled)
        del blocker
        self._crop_detail.setText(detail)
        self._refresh_controls()

    def set_pending_angle(self, angle: float | None) -> None:
        """Optionally reflect the coordinator's currently pending angle command."""
        self._pending_angle = None if angle is None else max(-180.0, min(180.0, float(angle)))
        if angle is not None and not self._target_dirty:
            self._set_target_widgets(self._pending_angle)
        elif angle is None and not self._target_dirty and self._ack_angle is not None:
            self._set_target_widgets(self._ack_angle)
        self._refresh_target_status()

    def _toggle_log(self, expanded: bool) -> None:
        self._log.setVisible(expanded)
        self._log_toggle.setText("▾  操作记录" if expanded else "▸  操作记录")

    def show_frame(self, frame_bgr: Any, identity: object = None) -> QImage:
        """Display a BGR NumPy frame and return an owning BGR888 QImage copy."""
        height, width, channels = frame_bgr.shape
        if channels != 3:
            raise ValueError("frame_bgr must have exactly three BGR channels")
        bytes_per_line = int(frame_bgr.strides[0])
        image = QImage(
            frame_bgr.data,
            int(width),
            int(height),
            bytes_per_line,
            QImage.Format.Format_BGR888,
        ).copy()
        self._canvas.set_frame(image, identity)
        return image

    def set_preview_status(self, state: str, detail: str) -> None:
        """Reflect the v2 preview status text and detail supplied by the app."""
        self._preview_badge.setText('处理帧已停止' if self._video_stalled else state)
        self._preview_detail.setText(detail)
        self._preview_live = state == "实时预览"
        if state == "实时预览":
            self._canvas.set_live(True)
        elif state == "预览已中断":
            self._canvas.set_live(False, "视频已中断")
        else:
            self._canvas.set_live(False, "等待视频")
        self._refresh_controls()

    def set_control_state(
        self,
        connected: bool,
        busy: bool,
        angle: float | None,
        status: str,
        *, control_ready: bool | None = None,
    ) -> None:
        """Update connection/busy state and the last ACK-confirmed angle."""
        prior_pending_angle = self._pending_angle
        self._connected = bool(connected)
        self._control_ready = bool(connected if control_ready is None else control_ready)
        self._busy = bool(busy)
        self._angle_known = angle is not None
        self._ack_angle = None if angle is None else float(angle)
        if angle is None and prior_pending_angle is not None:
            self._target_dirty = True
        self._control_status.setText(status)
        self._effective_caption.setText(
            "当前已生效角度 · 板卡确认" if busy else "已生效角度 · 板卡确认"
        )
        self._effective_angle.setText(
            "—" if angle is None else f"{self._format_angle(float(angle))}°"
        )

        if angle is not None and self._pending_angle is not None:
            if abs(math.remainder(float(angle) - self._pending_angle, 360.0)) <= 0.004:
                self._pending_angle = None
                self._target_dirty = False
        if not self._target_dirty and self._pending_angle is None and angle is not None:
            self._set_target_widgets(float(angle))
        if not self._connected:
            self._debounce.stop()
        self._refresh_controls()

    @staticmethod
    def _monitor_value(value: Any, suffix: str = "") -> str:
        if value is None:
            return "未知"
        return f"{value}{suffix}"

    @staticmethod
    def _compact_summary(value: str, limit: int = 25) -> str:
        value = " ".join(str(value).split())
        return value if len(value) <= limit else value[:limit - 1] + "…"

    @staticmethod
    def _gamma_bank(value: Any) -> str:
        meanings = {0: "强提亮", 1: "提亮", 2: "直通", 3: "压亮", 4: "强压亮", 5: "混合处理"}
        try:
            bank = int(value)
        except (TypeError, ValueError):
            return "未知"
        return f"{bank} · {meanings[bank]}" if bank in meanings else f"{bank} · 未知 LUT"

    @staticmethod
    def _corners_text(corners: Any) -> str:
        if not isinstance(corners, (list, tuple)) or len(corners) != 4:
            return "四角：未知"
        try:
            points = [f"({int(point[0])}, {int(point[1])})" for point in corners]
        except (TypeError, ValueError, IndexError):
            return "四角：数据无效"
        return "四角：" + " · ".join(points)

    def set_monitor_state(self, state: dict[str, Any]) -> None:
        """Render serialized CNN/Gamma status supplied by the V4 coordinator.

        Expected keys are documented in rotation_dashboard_v4/README.md. This
        view never applies a CNN prediction to the acknowledged rotation angle.
        """
        state = dict(state or {})
        gamma_source = state.get("gamma") if isinstance(state.get("gamma"), dict) else {}
        freshness = str(state.get("freshness", "unknown")).lower()
        if freshness == "unknown" and gamma_source.get("supported") is False:
            freshness = "unsupported"
        elif freshness == "unknown" and (gamma_source.get("stale") is True or
                                          float(gamma_source.get("age_seconds") or 0) > 2.5):
            freshness = "stale"
        elif freshness == "unknown" and gamma_source.get("error"):
            freshness = "error"
        elif freshness == "unknown" and state.get("connected") and (
                gamma_source.get("frame_id") is not None or state.get("cnn")):
            freshness = "fresh"
        detail = str(state.get("detail") or "")
        statuses = {
            "fresh": "实时", "valid": "实时", "unknown": "未知",
            "stale": "过期", "unsupported": "不支持", "error": "错误",
        }
        display = statuses.get(freshness, "未知")
        self._monitor_freshness.setText(f"● {display}" + (f" · {detail}" if detail else ""))
        color = (self._COLORS["success"] if freshness in ("fresh", "valid") else
                 self._COLORS["danger"] if freshness == "error" else
                 "#a76a00" if freshness == "stale" else self._COLORS["muted"])
        self._monitor_freshness.setStyleSheet(f"color: {color};")

        cnn = state.get("cnn") if isinstance(state.get("cnn"), dict) else {}
        gamma = gamma_source
        if isinstance(state.get("context"), dict):
            context = state["context"]
            self.set_monitor_context(bool(context.get("qualified")), str(context.get("detail", "")))
        cnn_status = cnn.get("status")
        if cnn_status is None:
            if cnn.get("busy"):
                cnn_status = "忙碌"
            elif cnn.get("enabled") is True:
                cnn_status = "已启用"
            elif cnn.get("enabled") is False:
                cnn_status = "已禁用"
            else:
                cnn_status = "未知"
        self._cnn_status.setText(str(cnn_status))
        self._cnn_bytes.setText(
            f"{self._monitor_value(cnn.get('loaded_bytes'), ' bytes')} / "
            f"{self._monitor_value(cnn.get('crc32'))}"
        )
        self._cnn_ready.setText(
            f"启用 {self._monitor_value(cnn.get('enabled'))} · "
            f"K 有效 {self._monitor_value(cnn.get('k_valid', cnn.get('k_ready')))} · "
            f"忙碌 {self._monitor_value(cnn.get('busy'))} · "
            f"错误 {self._monitor_value(cnn.get('error'))}"
        )
        self._cnn_counts.setText(
            f"{self._monitor_value(cnn.get('prediction_count'))} 次 · "
            f"{self._monitor_value(cnn.get('rate_hz', cnn.get('predictions_per_sec')), ' Hz')} · "
            f"{self._monitor_value(cnn.get('cycles'))} 周期"
        )
        angle = cnn.get("predicted_angle", cnn.get("predicted_angle_deg"))
        angle_text = "未知" if angle is None else f"{float(angle):.2f}°"
        self._cnn_prediction.setText(f"预测 {angle_text} · {self._corners_text(cnn.get('corners'))}")

        loaded = cnn.get("loaded")
        enabled = cnn.get("enabled")
        k_ready = cnn.get("k_valid", cnn.get("k_ready"))
        if cnn_status not in (None, "", "未知"):
            cnn_short = str(cnn_status)
        elif freshness == 'paused':
            cnn_short = "正在准备模型"
        elif loaded is False:
            cnn_short = "模型未加载"
        elif loaded is True and enabled is True and k_ready is True:
            cnn_short = "模型已就绪 · 已启用"
        elif loaded is True:
            cnn_short = "模型已加载 · 尚未就绪"
        else:
            cnn_short = "等待设备监测"
        if freshness in ("stale", "unsupported", "error", "unknown"):
            cnn_short = {"stale": "监测已过期", "unsupported": "设备不支持",
                         "error": "监测错误", "unknown": "状态未知"}[freshness]
        self._cnn_summary.setText(f"CNN：{self._compact_summary(cnn_short, 23)}")
        self._cnn_summary.setToolTip(f"CNN：{cnn_short} · {detail}")

        gamma_status = gamma.get("status", "不支持" if gamma.get("supported") is False else "未知")
        self._gamma_status.setText(
            f"{gamma_status} · 自适应 {self._monitor_value(gamma.get('enabled'))}"
        )
        self._gamma_applied.setText(self._gamma_bank(gamma.get("applied_bank")))
        self._gamma_next.setText(self._gamma_bank(gamma.get("next_bank")))
        mean = gamma.get("mean", gamma.get("mean_luminance"))
        mean_text = "未知" if mean is None else f"{float(mean):.2f}"
        self._gamma_mean.setText(
            f"{mean_text} · {self._monitor_value(gamma.get('valid_count'))} 像素"
        )
        self._gamma_frame.setText(
            f"{self._monitor_value(gamma.get('frame_id'))} · "
            f"{self._monitor_value(gamma.get('cfg_version', gamma.get('config_version')))}"
        )
        gamma_enabled = gamma.get("enabled")
        applied_bank = gamma.get("applied_bank")
        if gamma.get("supported") is False or freshness == "unsupported":
            gamma_short = "设备不支持 A8"
        elif gamma.get("stale") or freshness == "stale":
            gamma_short = "统计已过期"
        elif gamma.get("error") or freshness == "error":
            gamma_short = "监测错误"
        elif not gamma.get("valid", True):
            gamma_short = str(gamma.get("status") or "等待有效统计")
        elif applied_bank is not None:
            gamma_short = f"{self._gamma_bank(applied_bank)} · {'自适应开' if gamma_enabled else '自适应关'}"
        else:
            gamma_short = str(gamma.get("status") or "等待设备监测")
        self._gamma_summary.setText(f"Gamma：{self._compact_summary(gamma_short, 21)}")
        self._gamma_summary.setToolTip(f"Gamma：{gamma_short} · {detail}")

    def set_monitor_context(self, qualified: bool, detail: str) -> None:
        """Enable CNN actions only when matching acceptance/session is present."""
        self._monitor_context_qualified = bool(qualified)
        self._monitor_context_detail.setText(str(detail))
        self._refresh_monitor_controls()

    def set_monitor_busy(self, busy: bool) -> None:
        """Reflect monitor-tool work without changing rotation-control busy state."""
        self._monitor_busy = bool(busy)
        self._refresh_monitor_controls()

    def set_auto_correction_state(self, enabled: bool, detail: str) -> None:
        self._auto_running = bool(enabled)
        blocker = QSignalBlocker(self._auto_correction)
        self._auto_correction.setChecked(bool(enabled))
        del blocker
        self._auto_correction_detail.setText(str(detail))
        if enabled:
            reason = "已开启 · 手动操作可随时关闭"
        elif self._auto_available:
            reason = "已就绪 · 默认关闭"
        else:
            reason = self._auto_availability_reason
        self._set_auto_reason(reason, detail)
        self._refresh_auto_control()

    def set_auto_availability(self, ready: bool, reason: str) -> None:
        """Show readiness and allow explicit activation only when ready."""
        self._auto_available = bool(ready)
        self._auto_availability_reason = str(reason or (
            "已就绪 · 默认关闭" if ready else "尚未就绪"))
        if not ready and self._auto_running:
            self._auto_running = False
            blocker = QSignalBlocker(self._auto_correction)
            self._auto_correction.setChecked(False)
            del blocker
        shown = ("已开启 · 手动操作可随时关闭" if self._auto_running else
                 "已就绪 · 默认关闭" if ready else self._auto_availability_reason)
        self._set_auto_reason(shown, self._auto_availability_reason)
        self._refresh_auto_control()

    def _set_auto_reason(self, visible: str, full: str) -> None:
        self._auto_reason.setText(self._compact_summary(str(visible), 35))
        self._auto_reason.setToolTip(str(full))

    def _refresh_auto_control(self) -> None:
        self._auto_correction.setEnabled(self._auto_available or self._auto_running)
        if not self._auto_available and not self._auto_running:
            self._auto_correction.setToolTip(self._auto_availability_reason)
        else:
            self._auto_correction.setToolTip("需要时可明确开启；手动旋转/去边操作会关闭自动校正。")

    def _refresh_monitor_controls(self) -> None:
        ready = self._monitor_context_qualified and not self._monitor_busy
        self._cnn_prepare_button.setEnabled(
            self._connected and self._control_ready and self._prepare_available and not self._busy and not self._monitor_busy)
        self._monitor_refresh_button.setEnabled(not self._monitor_busy)
        self._monitor_context_button.setEnabled(not self._monitor_busy)
        for button in (self._model_load_button, self._model_k_button,
                       self._model_enable_button, self._model_disable_button):
            button.setEnabled(ready)

    def set_prepare_availability(self, ready, reason=''):
        self._prepare_available = bool(ready)
        self._cnn_prepare_button.setToolTip(str(reason) or '加载模型并配置已测内参；首次需要几分钟。')
        self._refresh_monitor_controls()

    def set_video_health(self, state, detail=''):
        self._video_stalled = state == 'stalled'
        self._preview_status.setToolTip(str(detail))
        self._refresh_controls()

    def append_log(self, text: str) -> None:
        self._log.appendPlainText(str(text))

    def show_notice(self, title: str, detail: str, level: str = "info") -> None:
        palette = {
            "info": self._COLORS["accent"],
            "success": self._COLORS["success"],
            "warning": "#a76a00",
            "error": self._COLORS["danger"],
        }
        color = palette.get(level, palette["info"])
        self._notice_title.setText(title)
        self._notice_detail.setText(detail)
        self._notice_title.setStyleSheet(f"color: {color};")
        self._notice_mark.setStyleSheet(f"color: {color};")

    def allow_close(self) -> None:
        self._allow_close = True
        self.close()

    def closeEvent(self, event: Any) -> None:  # noqa: N802 - Qt override
        if self._allow_close:
            event.accept()
            return
        event.ignore()
        self.closingRequested.emit()
