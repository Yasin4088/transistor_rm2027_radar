import argparse
import importlib.util
import os
import sys
import time
from datetime import datetime
from pathlib import Path


def _force_pyqt5_qt_plugin_path():
    spec = importlib.util.find_spec("PyQt5")
    if not spec or not spec.origin:
        return
    pyqt_directory = Path(spec.origin).resolve().parent
    plugins = pyqt_directory / "Qt5" / "plugins"
    if plugins.is_dir():
        os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = str(plugins)
    os.environ.pop("QT_PLUGIN_PATH", None)


_force_pyqt5_qt_plugin_path()

from serial.tools import list_ports

from PyQt5.QtCore import QProcess, QProcessEnvironment, QSize, Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QStyle,
    QVBoxLayout,
    QWidget,
)

from calibration_presets import (
    DEFAULT_PRESET_PATH,
    PRESET_ENVIRONMENT_VARIABLE,
    CalibrationPresetError,
    list_presets,
)
from config_editor import load_config, update_config_values
from recording_storage import suggest_recording_storage
from runtime_status import clear_runtime_status, read_runtime_status


PROJECT_ROOT = Path(__file__).resolve().parent
CONFIG_PATH = PROJECT_ROOT / "config.yaml"
REFEREE_FRESH_SECONDS = 2.0


APP_STYLE = """
QWidget {
    color: #17212b;
    font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
    font-size: 14px;
}
QMainWindow, QWidget#central {
    background: #f2f4f5;
}
QFrame#header {
    background: #182129;
    border: 0;
}
QLabel#brand {
    color: #ffffff;
    font-size: 24px;
    font-weight: 700;
}
QLabel#subtitle, QLabel#clock {
    color: #aebbc5;
}
QFrame#statusBand {
    background: #ffffff;
    border-bottom: 1px solid #d7dde1;
}
QFrame#statusItem {
    border-right: 1px solid #e1e5e8;
}
QLabel#statusTitle {
    color: #697781;
    font-size: 12px;
}
QLabel#statusValue {
    color: #17212b;
    font-size: 15px;
    font-weight: 600;
}
QFrame#section {
    background: #ffffff;
    border: 1px solid #d9dfe3;
    border-radius: 5px;
}
QLabel#sectionTitle {
    color: #34424d;
    font-size: 15px;
    font-weight: 700;
}
QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {
    min-height: 32px;
    padding: 0 9px;
    background: #ffffff;
    border: 1px solid #bcc6cc;
    border-radius: 4px;
    selection-background-color: #247f6b;
}
QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus {
    border: 2px solid #247f6b;
}
QCheckBox {
    spacing: 8px;
    min-height: 26px;
}
QCheckBox::indicator {
    width: 18px;
    height: 18px;
}
QPushButton {
    min-height: 38px;
    padding: 0 14px;
    border: 1px solid #aeb9bf;
    border-radius: 4px;
    background: #ffffff;
    font-weight: 600;
}
QPushButton:hover {
    background: #edf1f2;
}
QPushButton:pressed {
    background: #dfe5e7;
}
QPushButton:disabled {
    color: #9ba5ab;
    background: #eef1f2;
}
QPushButton#redSide, QPushButton#blueSide {
    min-height: 38px;
    font-size: 15px;
}
QPushButton#redSide:checked {
    color: #ffffff;
    background: #c73b47;
    border-color: #9f2631;
}
QPushButton#blueSide:checked {
    color: #ffffff;
    background: #2877bd;
    border-color: #185b94;
}
QPushButton#saveButton {
    color: #185f51;
    border-color: #41917f;
}
QPushButton#calibrateButton {
    color: #ffffff;
    background: #9b6618;
    border-color: #7c4f0e;
    min-height: 50px;
    font-size: 16px;
}
QPushButton#startButton {
    color: #ffffff;
    background: #247f6b;
    border-color: #185f51;
    min-height: 50px;
    font-size: 16px;
}
QPushButton#stopButton {
    color: #a22530;
    border-color: #c85c65;
    min-height: 46px;
}
QPlainTextEdit {
    background: #11191f;
    color: #d6e0e5;
    border: 1px solid #26343d;
    border-radius: 4px;
    padding: 8px;
    font-family: "JetBrains Mono", "Noto Sans Mono CJK SC", monospace;
    font-size: 12px;
}
QScrollArea {
    border: 0;
    background: transparent;
}
"""


class StatusIndicator(QFrame):
    COLORS = {
        "neutral": "#8b989f",
        "good": "#25866f",
        "warning": "#d08a22",
        "bad": "#c73b47",
        "blue": "#2877bd",
    }

    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.setObjectName("statusItem")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setMinimumHeight(74)

        self.dot = QLabel()
        self.dot.setFixedSize(12, 12)
        title_label = QLabel(title)
        title_label.setObjectName("statusTitle")
        self.value_label = QLabel("待检测")
        self.value_label.setObjectName("statusValue")

        text_layout = QVBoxLayout()
        text_layout.setContentsMargins(0, 0, 0, 0)
        text_layout.setSpacing(2)
        text_layout.addWidget(title_label)
        text_layout.addWidget(self.value_label)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(18, 10, 18, 10)
        layout.setSpacing(11)
        layout.addWidget(self.dot)
        layout.addLayout(text_layout)
        self.set_status("待检测", "neutral")

    def set_status(self, text, level="neutral"):
        color = self.COLORS.get(level, self.COLORS["neutral"])
        self.dot.setStyleSheet(f"background: {color}; border-radius: 6px;")
        self.value_label.setText(text)


class MatchLauncher(QMainWindow):
    def __init__(self):
        super().__init__()
        self.process = None
        self.process_kind = None
        self.serial_devices = set()
        self.config = load_config(CONFIG_PATH)

        self.setWindowTitle("SHARK 雷达比赛启动台")
        self.setMinimumSize(1100, 720)
        self.resize(1440, 900)
        self.setStyleSheet(APP_STYLE)
        self._build_ui()
        self._load_settings()
        self._connect_signals()

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self._poll_status)
        self.poll_timer.start(800)
        self.clock_timer = QTimer(self)
        self.clock_timer.timeout.connect(self._update_clock)
        self.clock_timer.start(1000)
        self._update_clock()
        self._poll_status()
        self._append_log("比赛启动台已就绪")

    def _build_ui(self):
        central = QWidget()
        central.setObjectName("central")
        self.setCentralWidget(central)
        page = QVBoxLayout(central)
        page.setContentsMargins(0, 0, 0, 0)
        page.setSpacing(0)

        header = QFrame()
        header.setObjectName("header")
        header.setFixedHeight(88)
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(30, 14, 30, 14)

        brand_layout = QVBoxLayout()
        brand_layout.setSpacing(2)
        brand = QLabel("SHARK 雷达比赛启动台")
        brand.setObjectName("brand")
        subtitle = QLabel("江南大学霞客湾校区 SHARK 战队 · RoboMaster 2026")
        subtitle.setObjectName("subtitle")
        brand_layout.addWidget(brand)
        brand_layout.addWidget(subtitle)
        self.clock_label = QLabel()
        self.clock_label.setObjectName("clock")
        self.clock_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.clock_label.setFont(QFont(self.clock_label.font().family(), 15, QFont.DemiBold))
        header_layout.addLayout(brand_layout)
        header_layout.addStretch()
        header_layout.addWidget(self.clock_label)
        page.addWidget(header)

        status_band = QFrame()
        status_band.setObjectName("statusBand")
        status_layout = QHBoxLayout(status_band)
        status_layout.setContentsMargins(14, 0, 14, 0)
        status_layout.setSpacing(0)
        self.program_status = StatusIndicator("运行状态")
        self.camera_status = StatusIndicator("相机状态")
        self.referee_status = StatusIndicator("裁判系统")
        status_layout.addWidget(self.program_status)
        status_layout.addWidget(self.camera_status)
        status_layout.addWidget(self.referee_status)
        page.addWidget(status_band)

        body = QHBoxLayout()
        body.setContentsMargins(22, 18, 22, 20)
        body.setSpacing(18)
        page.addLayout(body, 1)

        self.settings_scroll = QScrollArea()
        self.settings_scroll.setWidgetResizable(True)
        self.settings_scroll.setMinimumWidth(610)
        self.settings_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        settings_widget = QWidget()
        settings_layout = QVBoxLayout(settings_widget)
        settings_layout.setContentsMargins(0, 0, 6, 0)
        settings_layout.setSpacing(10)

        match_section, match_form = self._make_section("比赛设置")
        self.red_side = QPushButton("己方红方")
        self.red_side.setObjectName("redSide")
        self.red_side.setCheckable(True)
        self.blue_side = QPushButton("己方蓝方")
        self.blue_side.setObjectName("blueSide")
        self.blue_side.setCheckable(True)
        self.side_group = QButtonGroup(self)
        self.side_group.setExclusive(True)
        self.side_group.addButton(self.red_side)
        self.side_group.addButton(self.blue_side)
        side_row = QWidget()
        side_layout = QHBoxLayout(side_row)
        side_layout.setContentsMargins(0, 0, 0, 0)
        side_layout.setSpacing(8)
        side_layout.addWidget(self.red_side)
        side_layout.addWidget(self.blue_side)
        match_form.addRow("己方阵营", side_row)

        self.camera_mode = QComboBox()
        self.camera_mode.addItem("海康工业相机", "hik")
        self.camera_mode.addItem("USB 相机", "video")
        self.camera_mode.addItem("测试视频 / 图片", "test")
        match_form.addRow("图像输入", self.camera_mode)

        self.algorithm_mode = QComboBox()
        self.algorithm_mode.addItem("2131aef 视觉识别链路（默认）", "legacy")
        self.algorithm_mode.addItem("车辆跟踪 + 批量装甲板（可选）", "hkust_tracker")
        self.algorithm_mode.setToolTip("默认使用 2131aef 的完整视觉识别链路")
        match_form.addRow("识别算法", self.algorithm_mode)

        toggles = QWidget()
        toggle_layout = QHBoxLayout(toggles)
        toggle_layout.setContentsMargins(0, 0, 0, 0)
        self.multi_car = QCheckBox("多车识别")
        self.record_video_checkbox = QCheckBox("录制视觉画面 + 标定地图")
        self.record_video_checkbox.setToolTip(
            "启动比赛后自动录制；优先写入已挂载外接硬盘，未检测到时写入默认目录"
        )
        self.blind_zone = QCheckBox("盲区预测")
        toggle_layout.addWidget(self.multi_car)
        toggle_layout.addWidget(self.record_video_checkbox)
        toggle_layout.addWidget(self.blind_zone)
        toggle_layout.addStretch()
        match_form.addRow("运行选项", toggles)

        self.recording_path = QLineEdit()
        self.recording_path.setPlaceholderText("请选择录像存储目录")
        self.recording_path.setToolTip("每场录像会保存到此目录下带统一 match_run_id 的子目录")
        self.browse_recording_button = QPushButton("选择…")
        self.browse_recording_button.setIcon(
            self.style().standardIcon(QStyle.SP_DirOpenIcon)
        )
        self.browse_recording_button.setIconSize(QSize(18, 18))
        self.browse_recording_button.setToolTip("选择录像存储目录")
        recording_path_row = QWidget()
        recording_path_layout = QHBoxLayout(recording_path_row)
        recording_path_layout.setContentsMargins(0, 0, 0, 0)
        recording_path_layout.setSpacing(6)
        recording_path_layout.addWidget(self.recording_path, 1)
        recording_path_layout.addWidget(self.browse_recording_button)
        match_form.addRow("录像目录", recording_path_row)

        self.filter_type = QComboBox()
        self.filter_type.addItem("滑动窗口", "sliding_window")
        self.filter_type.addItem("混合门控卡尔曼", "hybrid_gated_kalman")
        match_form.addRow("位置滤波", self.filter_type)
        self.vulnerability_mode = QComboBox()
        self.vulnerability_mode.addItem("视觉端已关闭（无线电负责）", "disabled")
        self.vulnerability_mode.setEnabled(False)
        self.vulnerability_mode.setToolTip("视觉端双倍易伤决策已关闭，后续由无线电程序负责")
        match_form.addRow("双倍易伤", self.vulnerability_mode)
        settings_layout.addWidget(match_section)

        serial_section, serial_form = self._make_section("裁判系统与设备")
        self.referee_mode = QComboBox()
        self.referee_mode.addItem("无线电 ROS2（比赛默认）", "radio_ros")
        self.referee_mode.addItem("视觉直连串口（legacy）", "legacy_serial")
        serial_form.addRow("裁判通信模式", self.referee_mode)

        self.serial_port = QComboBox()
        self.serial_port.setEditable(True)
        self.serial_port.setInsertPolicy(QComboBox.NoInsert)
        self.refresh_ports_button = QPushButton()
        self.refresh_ports_button.setIcon(self.style().standardIcon(QStyle.SP_BrowserReload))
        self.refresh_ports_button.setIconSize(QSize(18, 18))
        self.refresh_ports_button.setFixedWidth(42)
        self.refresh_ports_button.setToolTip("刷新串口设备")
        port_row = QWidget()
        port_layout = QHBoxLayout(port_row)
        port_layout.setContentsMargins(0, 0, 0, 0)
        port_layout.setSpacing(6)
        port_layout.addWidget(self.serial_port, 1)
        port_layout.addWidget(self.refresh_ports_button)
        serial_form.addRow("串口端口", port_row)
        self.serial_port_row = port_row
        self.serial_port_label = serial_form.labelForField(port_row)

        self.camera_serial = QLineEdit()
        self.camera_serial.setPlaceholderText("留空则使用第一台海康相机")
        serial_form.addRow("相机序列号", self.camera_serial)

        camera_params = QWidget()
        camera_params_layout = QHBoxLayout(camera_params)
        camera_params_layout.setContentsMargins(0, 0, 0, 0)
        camera_params_layout.setSpacing(8)
        self.exposure = QDoubleSpinBox()
        self.exposure.setRange(10.0, 1000000.0)
        self.exposure.setDecimals(1)
        self.exposure.setSingleStep(500.0)
        self.exposure.setSuffix(" us")
        self.gain = QDoubleSpinBox()
        self.gain.setRange(0.0, 48.0)
        self.gain.setDecimals(1)
        self.gain.setSingleStep(0.5)
        self.gain.setSuffix(" dB")
        camera_params_layout.addWidget(self.exposure)
        camera_params_layout.addWidget(self.gain)
        serial_form.addRow("曝光 / 增益", camera_params)
        settings_layout.addWidget(serial_section)

        calibration_section, calibration_form = self._make_section("标定与测试")
        self.calibration_preset = QComboBox()
        self.refresh_presets_button = QPushButton()
        self.refresh_presets_button.setIcon(
            self.style().standardIcon(QStyle.SP_BrowserReload)
        )
        self.refresh_presets_button.setIconSize(QSize(18, 18))
        self.refresh_presets_button.setFixedWidth(42)
        self.refresh_presets_button.setToolTip("刷新地图预制方案")
        preset_row = QWidget()
        preset_layout = QHBoxLayout(preset_row)
        preset_layout.setContentsMargins(0, 0, 0, 0)
        preset_layout.setSpacing(6)
        preset_layout.addWidget(self.calibration_preset, 1)
        preset_layout.addWidget(self.refresh_presets_button)
        calibration_form.addRow("地图预制", preset_row)

        self.points_per_layer = QSpinBox()
        self.points_per_layer.setRange(4, 12)
        self.points_per_layer.setSuffix(" 点 / 层")
        calibration_form.addRow("标定点数", self.points_per_layer)
        self.run_after_save = QCheckBox("保存标定后自动进入比赛")
        calibration_form.addRow("完成后", self.run_after_save)

        self.test_video = QLineEdit()
        self.test_video.setPlaceholderText("留空时使用静态测试图片")
        self.browse_video_button = QPushButton()
        self.browse_video_button.setIcon(self.style().standardIcon(QStyle.SP_DirOpenIcon))
        self.browse_video_button.setIconSize(QSize(18, 18))
        self.browse_video_button.setFixedWidth(42)
        self.browse_video_button.setToolTip("选择测试视频")
        video_row = QWidget()
        video_layout = QHBoxLayout(video_row)
        video_layout.setContentsMargins(0, 0, 0, 0)
        video_layout.setSpacing(6)
        video_layout.addWidget(self.test_video, 1)
        video_layout.addWidget(self.browse_video_button)
        calibration_form.addRow("测试视频", video_row)
        settings_layout.addWidget(calibration_section)
        settings_layout.addStretch()
        self.settings_scroll.setWidget(settings_widget)
        body.addWidget(self.settings_scroll, 5)

        operations = QWidget()
        operations_layout = QVBoxLayout(operations)
        operations_layout.setContentsMargins(0, 0, 0, 0)
        operations_layout.setSpacing(10)

        actions_section = QFrame()
        actions_section.setObjectName("section")
        actions_layout = QVBoxLayout(actions_section)
        actions_layout.setContentsMargins(16, 14, 16, 16)
        actions_layout.setSpacing(9)
        action_title = QLabel("现场操作")
        action_title.setObjectName("sectionTitle")
        actions_layout.addWidget(action_title)

        self.save_button = QPushButton("保存比赛配置")
        self.save_button.setObjectName("saveButton")
        self.save_button.setIcon(self.style().standardIcon(QStyle.SP_DialogSaveButton))
        self.calibrate_button = QPushButton("开始标定")
        self.calibrate_button.setObjectName("calibrateButton")
        self.calibrate_button.setIcon(self.style().standardIcon(QStyle.SP_FileDialogDetailedView))
        self.start_button = QPushButton("启动比赛主程序")
        self.start_button.setObjectName("startButton")
        self.start_button.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        self.stop_button = QPushButton("停止当前程序")
        self.stop_button.setObjectName("stopButton")
        self.stop_button.setIcon(self.style().standardIcon(QStyle.SP_MediaStop))
        self.stop_button.setEnabled(False)
        actions_layout.addWidget(self.save_button)
        actions_layout.addWidget(self.calibrate_button)
        actions_layout.addWidget(self.start_button)
        actions_layout.addWidget(self.stop_button)
        operations_layout.addWidget(actions_section)

        log_title = QLabel("运行日志")
        log_title.setObjectName("sectionTitle")
        operations_layout.addWidget(log_title)
        self.log_output = QPlainTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setMaximumBlockCount(1500)
        operations_layout.addWidget(self.log_output, 1)
        body.addWidget(operations, 4)

    @staticmethod
    def _make_section(title):
        section = QFrame()
        section.setObjectName("section")
        layout = QVBoxLayout(section)
        layout.setContentsMargins(14, 10, 14, 12)
        layout.setSpacing(7)
        heading = QLabel(title)
        heading.setObjectName("sectionTitle")
        layout.addWidget(heading)
        form = QFormLayout()
        form.setContentsMargins(0, 0, 0, 0)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(7)
        form.setLabelAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        layout.addLayout(form)
        return section, form

    def _connect_signals(self):
        self.save_button.clicked.connect(lambda: self._save_configuration(show_message=True))
        self.calibrate_button.clicked.connect(lambda: self._start_process("calibration"))
        self.start_button.clicked.connect(lambda: self._start_process("match"))
        self.stop_button.clicked.connect(self._stop_process)
        self.refresh_ports_button.clicked.connect(lambda: self._refresh_serial_ports())
        self.browse_video_button.clicked.connect(self._browse_test_video)
        self.browse_recording_button.clicked.connect(self._browse_recording_directory)
        self.refresh_presets_button.clicked.connect(self._refresh_calibration_presets)
        self.red_side.toggled.connect(
            lambda checked: self._refresh_calibration_presets() if checked else None
        )
        self.blue_side.toggled.connect(
            lambda checked: self._refresh_calibration_presets() if checked else None
        )
        self.points_per_layer.valueChanged.connect(self._refresh_calibration_presets)
        self.camera_mode.currentIndexChanged.connect(self._update_mode_fields)
        self.algorithm_mode.currentIndexChanged.connect(self._update_algorithm_fields)
        self.referee_mode.currentIndexChanged.connect(self._update_serial_fields)

    def _load_settings(self):
        global_config = self.config.get("global", {})
        state = global_config.get("state", "B")
        self.red_side.setChecked(state == "R")
        self.blue_side.setChecked(state != "R")
        self._select_combo_data(self.camera_mode, global_config.get("camera_mode", "test"))
        self._select_combo_data(
            self.algorithm_mode,
            self.config.get("algorithm", {}).get("mode", "legacy"),
        )
        self.multi_car.setChecked(bool(global_config.get("multi_car_recognition", True)))
        self.record_video_checkbox.setChecked(bool(global_config.get("save_img", False)))
        self._initialize_recording_path()
        self.blind_zone.setChecked(bool(self.config.get("blind_zone", {}).get("enabled", True)))
        configured_transport = self.config.get("referee", {}).get("transport")
        if configured_transport not in ("radio_ros", "legacy_serial"):
            configured_transport = "legacy_serial" if global_config.get("use_serial", False) else "radio_ros"
        self._select_combo_data(self.referee_mode, configured_transport)

        filter_type = self.config.get("filter", {}).get("type", "sliding_window")
        if filter_type == "hybrid":
            filter_type = "hybrid_gated_kalman"
        self._select_combo_data(self.filter_type, filter_type)
        self._select_combo_data(
            self.vulnerability_mode,
            "disabled",
        )

        serial_config = self.config.get("serial", {})
        self._refresh_serial_ports(serial_config.get("port", "/dev/ttyUSB0"))
        camera_config = self.config.get("camera_params", {})
        self.camera_serial.setText(str(camera_config.get("device_serial", "") or ""))
        self.exposure.setValue(float(camera_config.get("exposure_time", 16000.0)))
        self.gain.setValue(float(camera_config.get("gain", 0.0)))

        calibration_config = self.config.get("calibration", {})
        self.points_per_layer.setValue(int(calibration_config.get("points_per_layer", 4)))
        self.run_after_save.setChecked(bool(calibration_config.get("run_after_save", True)))
        self._refresh_calibration_presets()
        test_config = global_config.get("test", {})
        self.test_video.setText(str(test_config.get("video_path", "") or ""))
        self._update_mode_fields()
        self._update_algorithm_fields()
        self._update_serial_fields()

    @staticmethod
    def _select_combo_data(combo, value):
        index = combo.findData(value)
        if index >= 0:
            combo.setCurrentIndex(index)

    def _refresh_serial_ports(self, preferred_port=None):
        current = preferred_port if preferred_port is not None else self.serial_port.currentText().strip()
        ports = sorted(list_ports.comports(), key=lambda item: item.device)
        self.serial_devices = {item.device for item in ports}
        self.serial_port.blockSignals(True)
        self.serial_port.clear()
        for item in ports:
            label = item.device
            if item.description and item.description != "n/a":
                label += f" · {item.description}"
            self.serial_port.addItem(label, item.device)
        if current and current not in self.serial_devices:
            self.serial_port.addItem(current, current)
        target_index = self.serial_port.findData(current)
        if target_index >= 0:
            self.serial_port.setCurrentIndex(target_index)
        elif self.serial_port.count() > 0:
            self.serial_port.setCurrentIndex(0)
        else:
            self.serial_port.setEditText(current or "/dev/ttyUSB0")
        self.serial_port.blockSignals(False)
        self._poll_status()

    def _selected_serial_port(self):
        return self.serial_port.currentText().split(" · ", 1)[0].strip()

    def _refresh_calibration_presets(self):
        selected_name = self.calibration_preset.currentData()
        side = "R" if self.red_side.isChecked() else "B"
        point_count = self.points_per_layer.value()
        self.calibration_preset.blockSignals(True)
        self.calibration_preset.clear()
        self.calibration_preset.addItem("不使用地图预制", "")
        try:
            presets = list_presets(
                DEFAULT_PRESET_PATH,
                side=side,
                points_per_layer=point_count,
            )
        except CalibrationPresetError as error:
            self.calibration_preset.setToolTip(str(error))
            self.calibration_preset.blockSignals(False)
            if hasattr(self, "log_output"):
                self._append_log(f"地图预制方案读取失败: {error}")
            return

        for preset in presets:
            self.calibration_preset.addItem(preset["name"], preset["name"])
        selected_index = self.calibration_preset.findData(selected_name)
        self.calibration_preset.setCurrentIndex(max(0, selected_index))
        own_name = "红方" if side == "R" else "蓝方"
        self.calibration_preset.setToolTip(
            f"仅显示己方{own_name}、每层 {point_count} 点的地图方案；"
            "视频/相机图像点仍需手动标定，方案可在标定窗口的“地图预制”菜单中管理"
        )
        self.calibration_preset.blockSignals(False)

    def _update_mode_fields(self):
        mode = self.camera_mode.currentData()
        self.camera_serial.setEnabled(mode == "hik")
        self.exposure.setEnabled(mode == "hik")
        self.gain.setEnabled(mode == "hik")
        self.test_video.setEnabled(mode == "test")
        self.browse_video_button.setEnabled(mode == "test")

    def _update_serial_fields(self):
        enabled = self.referee_mode.currentData() == "legacy_serial"
        self.serial_port.setEnabled(enabled)
        self.refresh_ports_button.setEnabled(enabled)
        self.serial_port_row.setVisible(enabled)
        self.serial_port_label.setVisible(enabled)
        self._poll_status()

    def _update_algorithm_fields(self):
        legacy = self.algorithm_mode.currentData() == "legacy"
        self.multi_car.setEnabled(legacy)
        self.filter_type.setEnabled(legacy)
        if legacy:
            self.multi_car.setToolTip("")
            self.filter_type.setToolTip("")
        else:
            hint = "新算法始终按车辆轨迹批量处理，并使用独立的真实 dt 卡尔曼滤波"
            self.multi_car.setToolTip(hint)
            self.filter_type.setToolTip(hint)

    def _browse_test_video(self):
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "选择测试视频",
            str(PROJECT_ROOT / "raw"),
            "视频文件 (*.mp4 *.avi *.mkv *.mov);;所有文件 (*)",
        )
        if not filename:
            return
        selected = Path(filename)
        try:
            selected = selected.relative_to(PROJECT_ROOT)
        except ValueError:
            pass
        self.test_video.setText(str(selected))

    def _configured_default_recording_root(self):
        configured = self.config.get("recording", {}).get("default_root", "save_img")
        default_root = Path(str(configured)).expanduser()
        if not default_root.is_absolute():
            default_root = PROJECT_ROOT / default_root
        return default_root.resolve()

    def _initialize_recording_path(self):
        recording_config = self.config.get("recording", {})
        default_root = self._configured_default_recording_root()
        try:
            storage = suggest_recording_storage(
                default_root,
                prefer_external=True,
                external_directory=recording_config.get(
                    "external_directory", "SHARK-radar-recordings"
                ),
            )
        except Exception as error:
            storage = suggest_recording_storage(default_root, prefer_external=False)
            self._append_log(f"外接硬盘扫描失败，已使用默认录像目录: {error}")

        self.recording_path.setText(str(storage.root))
        if storage.is_external:
            self.recording_path.setToolTip(
                f"已检测到外接硬盘 {storage.mount_point}；"
                "每场录像会保存到此目录下带统一 match_run_id 的子目录"
            )
            self._append_log(f"录像目录已初始化为外接硬盘: {storage.root}")
        else:
            self.recording_path.setToolTip(
                "未检测到可写外接硬盘，当前为默认目录；"
                "每场录像会保存到此目录下带统一 match_run_id 的子目录"
            )
            self._append_log(f"未检测到可写外接硬盘，录像目录使用默认路径: {storage.root}")

    def _normalized_recording_path(self):
        text = self.recording_path.text().strip()
        if not text:
            return None
        selected = Path(text).expanduser()
        if not selected.is_absolute():
            selected = PROJECT_ROOT / selected
        return selected.resolve()

    def _browse_recording_directory(self):
        initial = self._normalized_recording_path() or self._configured_default_recording_root()
        directory = QFileDialog.getExistingDirectory(
            self,
            "选择录像存储目录",
            str(initial),
            QFileDialog.ShowDirsOnly,
        )
        if directory:
            self.recording_path.setText(str(Path(directory).resolve()))
            self.recording_path.setToolTip(
                "操作者选择的录像目录；每场录像会保存到其下带统一 match_run_id 的子目录"
            )

    def _save_configuration(self, show_message=False):
        port = self._selected_serial_port()
        mode = str(self.referee_mode.currentData())
        if mode == "legacy_serial" and not port:
            QMessageBox.warning(self, "配置未保存", "启用裁判系统时必须填写串口端口。")
            return False

        recording_path = self._normalized_recording_path()
        if recording_path is None:
            QMessageBox.warning(self, "配置未保存", "请选择录像存储目录。")
            return False
        if recording_path.exists() and not recording_path.is_dir():
            QMessageBox.warning(
                self,
                "配置未保存",
                f"录像存储路径不是目录：\n{recording_path}",
            )
            return False

        updates = {
            "global.state": "R" if self.red_side.isChecked() else "B",
            "global.camera_mode": str(self.camera_mode.currentData()),
            "algorithm.mode": str(self.algorithm_mode.currentData()),
            "global.multi_car_recognition": self.multi_car.isChecked(),
            "global.use_serial": mode == "legacy_serial",
            "referee.transport": mode,
            "global.save_img": self.record_video_checkbox.isChecked(),
            "recording.selected_root": str(recording_path),
            "global.test.video_path": self.test_video.text().strip(),
            "blind_zone.enabled": self.blind_zone.isChecked(),
            "double_vulnerability.enabled": False,
            "double_vulnerability.trigger_mode": str(self.vulnerability_mode.currentData()),
            "calibration.points_per_layer": self.points_per_layer.value(),
            "calibration.run_after_save": self.run_after_save.isChecked(),
            "serial.port": port or "/dev/ttyUSB0",
            "filter.type": str(self.filter_type.currentData()),
            "camera_params.exposure_time": float(self.exposure.value()),
            "camera_params.gain": float(self.gain.value()),
            "camera_params.device_serial": self.camera_serial.text().strip(),
        }
        try:
            self.config = update_config_values(CONFIG_PATH, updates)
        except Exception as error:
            self._append_log(f"配置保存失败: {error}")
            QMessageBox.critical(self, "配置保存失败", str(error))
            return False

        side_name = "红方" if updates["global.state"] == "R" else "蓝方"
        communication_name = "无线电 ROS2" if mode == "radio_ros" else f"视觉串口 {port}"
        self._append_log(
            f"配置已保存：己方{side_name}，输入={updates['global.camera_mode']}，"
            f"算法={updates['algorithm.mode']}，"
            f"录像={'开' if updates['global.save_img'] else '关'}，"
            f"录像目录={updates['recording.selected_root']}，"
            f"裁判通信={communication_name}"
        )
        if show_message:
            QMessageBox.information(self, "配置已保存", "比赛配置已写入 config.yaml。")
        self._poll_status()
        return True

    def _start_process(self, kind):
        if self._process_is_running():
            QMessageBox.warning(self, "程序正在运行", "请先停止当前标定或比赛程序。")
            return
        if not self._save_configuration(show_message=False):
            return

        clear_runtime_status()
        script_name = "calibration.py" if kind == "calibration" else "main.py"
        self.process = QProcess(self)
        self.process_kind = kind
        self.process.setWorkingDirectory(str(PROJECT_ROOT))
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        environment = QProcessEnvironment.systemEnvironment()
        environment.insert("PYTHONUNBUFFERED", "1")
        if kind == "calibration":
            preset_name = str(self.calibration_preset.currentData() or "").strip()
            if preset_name:
                environment.insert(PRESET_ENVIRONMENT_VARIABLE, preset_name)
        self.process.setProcessEnvironment(environment)
        self.process.readyReadStandardOutput.connect(self._read_process_output)
        self.process.started.connect(self._process_started)
        self.process.finished.connect(self._process_finished)
        self.process.errorOccurred.connect(self._process_error)
        self.process.start(sys.executable, ["-u", script_name])
        self._set_action_state(True)
        action = "标定" if kind == "calibration" else "比赛主程序"
        self._append_log(f"正在启动{action}...")
        self._poll_status()

    def _process_started(self):
        action = "标定程序" if self.process_kind == "calibration" else "比赛主程序"
        self._append_log(f"{action}已启动，PID={int(self.process.processId())}")

    def _read_process_output(self):
        if self.process is None:
            return
        output = bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="replace")
        for line in output.rstrip().splitlines():
            self._append_log(line, include_time=False)

    def _process_finished(self, exit_code, exit_status):
        action = "标定程序" if self.process_kind == "calibration" else "比赛主程序"
        self._append_log(f"{action}已结束，退出码={exit_code}")
        clear_runtime_status()
        if self.process is not None:
            self.process.deleteLater()
        self.process = None
        self.process_kind = None
        self._set_action_state(False)
        self._refresh_calibration_presets()
        self._poll_status()

    def _process_error(self, error):
        if self.process is not None:
            self._append_log(f"进程错误: {self.process.errorString()} ({int(error)})")
        if error == QProcess.FailedToStart:
            clear_runtime_status()
            failed_process = self.process
            self.process = None
            self.process_kind = None
            self._set_action_state(False)
            if failed_process is not None:
                failed_process.deleteLater()
            self._poll_status()

    def _stop_process(self):
        if not self._process_is_running():
            return
        self._append_log("正在停止当前程序...")
        self.process.terminate()
        QTimer.singleShot(5000, self._kill_process_if_needed)

    def _kill_process_if_needed(self):
        if self._process_is_running():
            self._append_log("程序未按时退出，执行强制停止")
            self.process.kill()

    def _set_action_state(self, running):
        self.settings_scroll.setEnabled(not running)
        self.save_button.setEnabled(not running)
        self.calibrate_button.setEnabled(not running)
        self.start_button.setEnabled(not running)
        self.stop_button.setEnabled(running)

    def _process_is_running(self):
        return self.process is not None and self.process.state() != QProcess.NotRunning

    def _poll_status(self):
        running = self._process_is_running()
        status = read_runtime_status()
        expected_pid = int(self.process.processId()) if running and self.process is not None else None
        if status and expected_pid and status.get("process_id") not in (None, expected_pid):
            status = {}

        if running:
            if status.get("phase") == "running":
                self.program_status.set_status("比赛运行中", "good")
            elif self.process_kind == "calibration":
                self.program_status.set_status("标定进行中", "warning")
            else:
                self.program_status.set_status("主程序启动中", "blue")
        else:
            self.program_status.set_status("未运行", "neutral")

        mode_name = self.camera_mode.currentText()
        camera_error = status.get("camera_error")
        if camera_error and running:
            self.camera_status.set_status(f"相机错误 · {camera_error}", "bad")
        elif status.get("camera_ready") and running:
            camera_fps = status.get("camera_fps")
            processing_fps = status.get("processing_fps", status.get("fps"))
            if camera_fps is None:
                text = f"{mode_name} · 采集待测"
                if processing_fps is not None:
                    text += f" / 处理 {float(processing_fps):.1f} FPS"
                self.camera_status.set_status(text, "warning")
            else:
                camera_fps = max(float(camera_fps), 0.0)
                processing_fps = max(float(processing_fps or 0.0), 0.0)
                level = "good"
                if camera_fps < 5.0 or processing_fps < 5.0:
                    level = "bad"
                elif camera_fps < 10.0 or processing_fps < 10.0:
                    level = "warning"
                self.camera_status.set_status(
                    f"{mode_name} · 采集 {camera_fps:.1f} / 处理 {processing_fps:.1f} FPS",
                    level,
                )
        elif running:
            self.camera_status.set_status(f"{mode_name} · 等待画面", "warning")
        else:
            self.camera_status.set_status(f"{mode_name} · 待启动", "neutral")

        if self.referee_mode.currentData() == "radio_ros":
            if not running:
                self.referee_status.set_status("无线电 ROS2 · 待启动", "neutral")
                return
            communication = status.get("referee_communication") or {}
            referee_status = communication.get("referee_status") or {}
            if referee_status.get("frame_timed_out"):
                timeout = float(referee_status.get("frame_timeout_sec", REFEREE_FRESH_SECONDS))
                self.referee_status.set_status(f"裁判错误 · {timeout:g} 秒未收到有效帧", "bad")
                return
            communication_error = str(communication.get("last_error") or "").strip()
            if communication_error:
                self.referee_status.set_status(f"裁判错误 · {communication_error}", "bad")
                return
            if status.get("radio_ros_connected"):
                last_packet = status.get("last_referee_packet_at")
                if last_packet and time.time() - float(last_packet) <= REFEREE_FRESH_SECONDS:
                    self.referee_status.set_status("无线电 ROS2 · 裁判在线", "good")
                else:
                    started_at = status.get("started_at")
                    waiting_age = time.time() - float(started_at) if started_at else 0.0
                    if waiting_age >= REFEREE_FRESH_SECONDS:
                        self.referee_status.set_status("裁判错误 · 2 秒未收到有效帧", "bad")
                    else:
                        self.referee_status.set_status("无线电 ROS2 · 等待裁判数据", "warning")
            else:
                self.referee_status.set_status("无线电未运行 · 推理降级", "bad")
            return

        port = self._selected_serial_port()
        port_present = port in self.serial_devices or (port and Path(port).exists())
        if not running:
            if port_present:
                self.referee_status.set_status("串口设备已发现", "warning")
            else:
                self.referee_status.set_status("未发现串口设备", "bad")
            return

        if status and not status.get("serial_open", False):
            error_text = status.get("serial_error")
            self.referee_status.set_status("串口打开失败" if error_text else "串口未打开", "bad")
            return
        if not status:
            self.referee_status.set_status("等待主程序打开串口", "warning")
            return

        last_packet = status.get("last_referee_packet_at")
        if last_packet:
            age = max(0.0, time.time() - float(last_packet))
            if age <= REFEREE_FRESH_SECONDS:
                command = status.get("last_referee_command") or "有效数据"
                self.referee_status.set_status(f"已接入 · {command}", "good")
            else:
                self.referee_status.set_status(f"数据中断 {int(age)} 秒", "bad")
        else:
            started_at = status.get("started_at")
            waiting_age = time.time() - float(started_at) if started_at else 0.0
            if waiting_age >= REFEREE_FRESH_SECONDS:
                self.referee_status.set_status("裁判错误 · 2 秒未收到有效帧", "bad")
            else:
                self.referee_status.set_status("串口已打开 · 等待裁判数据", "warning")

    def _append_log(self, message, include_time=True):
        if not message:
            return
        prefix = datetime.now().strftime("[%H:%M:%S] ") if include_time else ""
        self.log_output.appendPlainText(prefix + message)

    def _update_clock(self):
        self.clock_label.setText(datetime.now().strftime("%Y-%m-%d  %H:%M:%S"))

    def closeEvent(self, event):
        if self._process_is_running():
            choice = QMessageBox.question(
                self,
                "程序仍在运行",
                "关闭启动台会停止当前标定或比赛程序，确定关闭吗？",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if choice != QMessageBox.Yes:
                event.ignore()
                return
            self.process.kill()
            self.process.waitForFinished(1500)
        event.accept()


def _save_screenshot(window, output_path, app):
    app.processEvents()
    screenshot = window.grab()
    screenshot.save(str(output_path))
    app.quit()


def main():
    parser = argparse.ArgumentParser(description="SHARK 雷达比赛启动台")
    parser.add_argument("--smoke-test", action="store_true", help="启动页面后立即退出")
    parser.add_argument("--screenshot", type=Path, help="保存页面截图后退出")
    parser.add_argument("--windowed", action="store_true", help="使用普通窗口而不是最大化")
    arguments = parser.parse_args()

    app = QApplication(sys.argv[:1])
    app.setApplicationName("SHARK Radar Match Console")
    window = MatchLauncher()
    if arguments.windowed or arguments.smoke_test or arguments.screenshot:
        window.show()
    else:
        window.showMaximized()

    if arguments.screenshot:
        QTimer.singleShot(600, lambda: _save_screenshot(window, arguments.screenshot, app))
    elif arguments.smoke_test:
        QTimer.singleShot(300, app.quit)
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
