"""
标定程序依赖 PyQt5 与 OpenCV。若先加载了用户目录里带 Qt 的 opencv-python，
其 cv2/qt/plugins 会抢占 xcb，导致 PyQt5 无法初始化。必须在 import cv2 之前
指定使用 PyQt5 自带的 Qt 平台插件目录。
"""
import importlib.util
import os
import re
import sys


def _force_pyqt5_qt_plugin_path():
    """将 QT_QPA_PLATFORM_PLUGIN_PATH 指向当前环境内 PyQt5/Qt5/plugins。"""
    spec = importlib.util.find_spec("PyQt5")
    if not spec or not spec.origin:
        return
    pyqt_dir = os.path.dirname(spec.origin)
    plugins = os.path.join(pyqt_dir, "Qt5", "plugins")
    if os.path.isdir(plugins):
        os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = plugins
    # 避免外部 QT_PLUGIN_PATH 指向 cv2 等错误插件集
    if "QT_PLUGIN_PATH" in os.environ:
        del os.environ["QT_PLUGIN_PATH"]


_force_pyqt5_qt_plugin_path()

import threading
import time

import cv2

# opencv-python（非 headless）在 import 时会改写 QT_QPA_PLATFORM_PLUGIN_PATH，需再次指回 PyQt5
_force_pyqt5_qt_plugin_path()

import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QImage, QTextCursor
from PyQt5.QtWidgets import (
    QApplication, QButtonGroup, QGridLayout, QHBoxLayout, QInputDialog, QLabel,
    QMenu, QMessageBox, QPushButton, QRadioButton, QTextEdit, QVBoxLayout, QWidget,
)

from calibration_presets import (
    DEFAULT_PRESET_PATH,
    PRESET_ENVIRONMENT_VARIABLE,
    CalibrationPresetError,
    delete_preset,
    get_preset,
    list_presets,
    save_preset,
)

import yaml
with open("config.yaml", "r", encoding="utf-8") as f:  # 指定 UTF-8 编码
    config = yaml.safe_load(f)


def update_config_state(selected_state, config_path="config.yaml"):
    """Update global.state atomically while preserving the existing YAML comments."""
    if selected_state not in ('R', 'B'):
        raise ValueError(f"无效的己方阵营: {selected_state}")

    with open(config_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    global_index = next(
        (index for index, line in enumerate(lines) if re.match(r'^global\s*:', line)),
        None,
    )
    if global_index is None:
        raise ValueError("config.yaml 中缺少 global 配置段")

    state_index = None
    for index in range(global_index + 1, len(lines)):
        line = lines[index]
        if line.strip() and not line.lstrip().startswith('#') and not line.startswith((' ', '\t')):
            break
        if re.match(r'^\s+state\s*:', line):
            state_index = index
            break
    if state_index is None:
        raise ValueError("config.yaml 的 global 配置段中缺少 state")

    original = lines[state_index].rstrip('\r\n')
    newline = '\r\n' if lines[state_index].endswith('\r\n') else '\n'
    indent = re.match(r'^(\s*)', original).group(1)
    comment_at = original.find('#')
    comment = original[comment_at:].strip() if comment_at >= 0 else ''
    lines[state_index] = f"{indent}state: '{selected_state}'"
    if comment:
        lines[state_index] += f"  {comment}"
    lines[state_index] += newline

    updated_text = ''.join(lines)
    parsed = yaml.safe_load(updated_text)
    if parsed.get('global', {}).get('state') != selected_state:
        raise ValueError("config.yaml 阵营配置校验失败")

    temporary_path = f"{config_path}.tmp"
    try:
        with open(temporary_path, "w", encoding="utf-8", newline='') as f:
            f.write(updated_text)
        os.replace(temporary_path, config_path)
    finally:
        if os.path.exists(temporary_path):
            os.remove(temporary_path)

    config.setdefault('global', {})['state'] = selected_state


def _load_hik_sdk():
    """Only load Hikvision SDK when hik camera mode is used."""
    from hik_camera import get_Value, hik_device_serial, image_control, select_hik_device_index, set_Value, \
        start_grab_and_get_data_size

    if sys.platform.startswith("win"):
        from MvImport import MvCameraControl_class as mv
    else:
        from MvImport_Linux import MvCameraControl_class as mv

    globals().update({
        'get_Value': get_Value,
        'hik_device_serial': hik_device_serial,
        'image_control': image_control,
        'select_hik_device_index': select_hik_device_index,
        'set_Value': set_Value,
        'start_grab_and_get_data_size': start_grab_and_get_data_size,
    })
    for name in (
        'MV_CC_DEVICE_INFO_LIST', 'MV_GIGE_DEVICE', 'MV_USB_DEVICE', 'MvCamera',
        'cast', 'POINTER', 'MV_CC_DEVICE_INFO', 'MV_ACCESS_Exclusive',
        'MVCC_INTVALUE_EX', 'memset', 'byref', 'sizeof', 'c_ubyte',
        'MV_FRAME_OUT_INFO_EX',
    ):
        globals()[name] = getattr(mv, name)


def read_test_calibration_image():
    test_cfg = config.get('global', {}).get('test', {})
    video_path = test_cfg.get('video_path') if isinstance(test_cfg, dict) else ''
    if video_path and os.path.exists(video_path):
        cap = cv2.VideoCapture(video_path)
        if cap.isOpened():
            start_frame = int(test_cfg.get('start_frame', 0) or 0)
            if start_frame > 0:
                cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
            ret, frame = cap.read()
            cap.release()
            if ret:
                print(f"标定使用测试视频帧: {video_path}, start_frame={start_frame}, shape={frame.shape}")
                return frame
        else:
            print(f"测试视频无法打开，回退到静态图片: {video_path}")

    image = cv2.imread(config['paths']['test_img'])
    if image is not None:
        print(f"标定使用静态图片: {config['paths']['test_img']}, shape={image.shape}")
    return image


# 海康相机图像获取线程
def hik_camera_get():
    _load_hik_sdk()
    # 获得设备信息
    global camera_image
    deviceList = MV_CC_DEVICE_INFO_LIST()
    tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE

    # ch:枚举设备 | en:Enum device
    # nTLayerType [IN] 枚举传输层 ，pstDevList [OUT] 设备列表
    while 1:
        ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
        if ret != 0:
            print("enum devices fail! ret[0x%x]" % ret)
            # sys.exit()

        if deviceList.nDeviceNum == 0:
            print("find no device!")
            # sys.exit()
        else:
            print("Find %d devices!" % deviceList.nDeviceNum)
            break

    for i in range(0, deviceList.nDeviceNum):
        mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
            print("\ngige device: [%d]" % i)
            # 输出设备名字
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName:
                strModeName = strModeName + chr(per)
            print("device model name: %s" % strModeName)
            print("serial number: %s" % hik_device_serial(mvcc_dev_info))
            # 输出设备ID
            nip1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
            nip2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
            nip3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
            nip4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
            print("current ip: %d.%d.%d.%d\n" % (nip1, nip2, nip3, nip4))
        # 输出USB接口的信息
        elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
            print("\nu3v device: [%d]" % i)
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName:
                if per == 0:
                    break
                strModeName = strModeName + chr(per)
            print("device model name: %s" % strModeName)

            strSerialNumber = hik_device_serial(mvcc_dev_info)
            print("user serial number: %s" % strSerialNumber)
    serial_cfg = config.get('camera_params', {}).get('device_serial')
    nConnectionNum = select_hik_device_index(deviceList, serial_wanted=serial_cfg, default_index=0)

    # ch:创建相机实例 | en:Creat Camera Object
    cam = MvCamera()

    # ch:选择设备并创建句柄 | en:Select device and create handle
    # cast(typ, val)，这个函数是为了检查val变量是typ类型的，但是这个cast函数不做检查，直接返回val
    stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents

    ret = cam.MV_CC_CreateHandle(stDeviceList)
    if ret != 0:
        print("create handle fail! ret[0x%x]" % ret)
        sys.exit()

    # ch:打开设备 | en:Open device
    ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
    if ret != 0:
        print("open device fail! ret[0x%x]" % ret)
        sys.exit()

    print(get_Value(cam, param_type="float_value", node_name="ExposureTime"),
          get_Value(cam, param_type="float_value", node_name="Gain"),
          get_Value(cam, param_type="enum_value", node_name="TriggerMode"),
          get_Value(cam, param_type="float_value", node_name="AcquisitionFrameRate"))

    # 设置设备的一些参数
    set_Value(cam, param_type="float_value", node_name="ExposureTime", node_value=config['camera_params']['exposure_time'])  # 曝光时间
    set_Value(cam, param_type="float_value", node_name="Gain", node_value=config['camera_params']['gain'])  # 增益值
    # 开启设备取流
    start_grab_and_get_data_size(cam)
    # 主动取流方式抓取图像
    stParam = MVCC_INTVALUE_EX()

    memset(byref(stParam), 0, sizeof(MVCC_INTVALUE_EX))
    ret = cam.MV_CC_GetIntValueEx("PayloadSize", stParam)
    if ret != 0:
        print("get payload size fail! ret[0x%x]" % ret)
        sys.exit()
    nDataSize = stParam.nCurValue
    pData = (c_ubyte * nDataSize)()
    stFrameInfo = MV_FRAME_OUT_INFO_EX()

    memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
    while True:
        ret = cam.MV_CC_GetOneFrameTimeout(pData, nDataSize, stFrameInfo, 1000)
        if ret == 0:
            image = np.asarray(pData)
            # 处理海康相机的图像格式为OPENCV处理的格式
            camera_image = image_control(data=image, stFrameInfo=stFrameInfo)
        else:
            print("no data[0x%x]" % ret)


def video_capture_get():
    global camera_image
    cam = cv2.VideoCapture(1)
    while True:
        ret, img = cam.read()
        if ret:
            camera_image = img
            time.sleep(0.016)  # 60fps


color = [(255, 255, 255), (0, 255, 0), (0, 0, 255)]


class MyUI(QWidget):
    def __init__(self):
        super().__init__()
        self.capturing = True
        self.points_per_layer = max(4, int(config.get('calibration', {}).get('points_per_layer', 4)))
        self.preset_path = DEFAULT_PRESET_PATH
        self.dragging_point = None
        self.initUI()
        requested_preset = os.environ.get(PRESET_ENVIRONMENT_VARIABLE, '').strip()
        if requested_preset:
            QTimer.singleShot(0, lambda: self.import_preset(requested_preset, confirm=False))

    def initUI(self):
        # 左上角部分
        self.state = state if state in ('R', 'B') else 'B'
        self.left_top_label = QLabel(self)
        self.left_top_label.setFixedSize(1350, 1000)
        self.left_top_label.setStyleSheet("border: 2px solid black;")
        self.left_top_label.mousePressEvent = self.left_top_clicked
        self.left_top_label.mouseMoveEvent = self.left_top_dragged
        self.left_top_label.mouseReleaseEvent = self.left_top_released
        self.image_points = [[None for _ in range(self.points_per_layer)] for _ in range(2)]
        self.map_points = [[None for _ in range(self.points_per_layer)] for _ in range(2)]
        self.image_counts = [0, 0]
        self.map_counts = [0, 0]
        self.click_history = []
        # 右上角部分
        self.right_top_label = QLabel(self)
        self.right_top_label.setFixedSize(550, 900)
        self.right_top_label.setStyleSheet("border: 2px solid black;")
        self.right_top_label.mousePressEvent = self.right_top_clicked
        self.right_top_label.mouseMoveEvent = self.right_top_dragged
        self.right_top_label.mouseReleaseEvent = self.right_top_released

        # 左下角部分
        self.left_bottom_text = QTextEdit(self)
        self.left_bottom_text.setFixedSize(300, 60)

        # 右下角部分
        self.button1 = QPushButton('开始标定', self)
        self.button1.setFixedSize(100, 30)
        self.button1.clicked.connect(self.button1_clicked)

        self.button2 = QPushButton('切换高度', self)
        self.button2.setFixedSize(100, 30)
        self.button2.clicked.connect(self.button2_clicked)

        self.button3 = QPushButton('地图预制', self)
        self.button3.setFixedSize(100, 30)
        self.preset_menu = QMenu(self.button3)
        self.import_preset_action = self.preset_menu.addAction('导入地图预制方案')
        self.save_preset_action = self.preset_menu.addAction('保存当前地图点为预制')
        self.delete_preset_action = self.preset_menu.addAction('删除地图预制方案')
        self.import_preset_action.triggered.connect(self.choose_and_import_preset)
        self.save_preset_action.triggered.connect(self.save_current_preset)
        self.delete_preset_action.triggered.connect(self.choose_and_delete_preset)
        self.button3.setMenu(self.preset_menu)

        self.button4 = QPushButton('保存计算', self)
        self.button4.setFixedSize(100, 30)
        self.button4.clicked.connect(self.button4_clicked)

        self.button5 = QPushButton('撤回', self)
        self.button5.setFixedSize(100, 30)
        self.button5.clicked.connect(self.undo_last_point)

        self.opponent_label = QLabel('对方', self)
        self.opponent_red = QRadioButton('红方', self)
        self.opponent_blue = QRadioButton('蓝方', self)
        self.opponent_group = QButtonGroup(self)
        self.opponent_group.addButton(self.opponent_red)
        self.opponent_group.addButton(self.opponent_blue)
        self.opponent_red.setChecked(self.state == 'B')
        self.opponent_blue.setChecked(self.state == 'R')
        self.opponent_red.clicked.connect(lambda: self.select_opponent('R'))
        self.opponent_blue.clicked.connect(lambda: self.select_opponent('B'))

        opponent_widget = QWidget(self)
        opponent_layout = QHBoxLayout(opponent_widget)
        opponent_layout.setContentsMargins(0, 0, 0, 0)
        opponent_layout.addWidget(self.opponent_label)
        opponent_layout.addWidget(self.opponent_red)
        opponent_layout.addWidget(self.opponent_blue)
        self.height = 0
        self.T = []

        # _,left_image = self.camera_capture.read()
        left_image = camera_image

        # 记录缩放比例
        self.left_scale_x = left_image.shape[1] / 1350.0
        self.left_scale_y = left_image.shape[0] / 1000.0

        left_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2RGB)
        self.left_image = cv2.resize(left_image, (1350, 1000))
        self.left_base_image = self.left_image.copy()
        self.load_side_assets()
        # 缩放图像
        self.update_images()

        self.camera_timer = QTimer(self)
        self.camera_timer.timeout.connect(self.update_camera)
        self.camera_timer.start(50)  # 50毫秒更新一次相机
        # 设置按钮样式
        self.set_button_style(self.button1)
        self.set_button_style(self.button2)
        self.set_button_style(self.button3)
        self.set_button_style(self.button4)
        self.set_button_style(self.button5)
        self.opponent_label.setStyleSheet("font-size: 16px;")
        self.opponent_red.setStyleSheet("font-size: 16px;")
        self.opponent_blue.setStyleSheet("font-size: 16px;")

        grid_layout = QGridLayout()
        grid_layout.addWidget(self.button1, 0, 0)
        grid_layout.addWidget(self.button2, 0, 1)
        grid_layout.addWidget(self.button3, 1, 0)
        grid_layout.addWidget(self.button4, 1, 1)
        grid_layout.addWidget(self.button5, 2, 0)
        grid_layout.addWidget(opponent_widget, 2, 1)

        buttons_and_text_widget = QWidget()

        hbox_buttons_and_text = QHBoxLayout(buttons_and_text_widget)
        hbox_buttons_and_text.addLayout(grid_layout)
        hbox_buttons_and_text.addWidget(self.left_bottom_text)

        vbox_left = QVBoxLayout()
        vbox_left.addWidget(self.left_top_label)

        vbox_right = QVBoxLayout()
        vbox_right.addWidget(self.right_top_label)
        vbox_right.addWidget(buttons_and_text_widget)

        hbox = QHBoxLayout()
        hbox.addLayout(vbox_left)
        hbox.addLayout(vbox_right)

        self.setLayout(hbox)
        self.setGeometry(0, 0, 1900, 1000)
        self.setWindowTitle('Calibration UI')
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.show()

    def load_side_assets(self):
        side_key = 'red' if self.state == 'R' else 'blue'
        self.save_path = config['paths']['calibration'][side_key]
        right_image_path = config['paths']['map_images'][side_key]
        right_image = cv2.imread(right_image_path)
        if right_image is None:
            raise FileNotFoundError(f"无法读取标定地图: {right_image_path}")

        self.right_scale_x = right_image.shape[1] / 550.0
        self.right_scale_y = right_image.shape[0] / 900.0
        right_image = cv2.cvtColor(right_image, cv2.COLOR_BGR2RGB)
        self.right_image = cv2.resize(right_image, (550, 900))
        self.right_base_image = self.right_image.copy()

    def select_opponent(self, opponent_state):
        selected_state = 'B' if opponent_state == 'R' else 'R'
        if not self.capturing:
            self.append_text('已开始标定，不能切换对方阵营')
            self.opponent_red.setChecked(self.state == 'B')
            self.opponent_blue.setChecked(self.state == 'R')
            return
        self.opponent_red.setChecked(opponent_state == 'R')
        self.opponent_blue.setChecked(opponent_state == 'B')
        if selected_state == self.state:
            return

        self.state = selected_state
        self.load_side_assets()
        update_config_state(self.state)
        self.redraw_points()
        opponent_name = '红方' if opponent_state == 'R' else '蓝方'
        own_name = '蓝方' if self.state == 'B' else '红方'
        self.append_text(f'对方已选{opponent_name}，己方为{own_name}')

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()
        elif event.key() in (Qt.Key_Backspace, Qt.Key_Z) and not self.capturing:
            self.undo_last_point()
        elif event.key() == Qt.Key_Space and sys.platform.startswith("linux"):
            os.system("xdotool key Escape")

    def update_images(self):

        left_pixmap = self.convert_cvimage_to_pixmap(self.left_image)
        self.left_top_label.setPixmap(left_pixmap)


        right_pixmap = self.convert_cvimage_to_pixmap(self.right_image)
        self.right_top_label.setPixmap(right_pixmap)

    def update_camera(self):
        if self.capturing:
            img0 = camera_image
            left_image = cv2.cvtColor(img0, cv2.COLOR_BGR2RGB)
            self.left_image = cv2.resize(left_image, (1350, 1000))
            self.left_base_image = self.left_image.copy()
            self.update_images()

    def redraw_points(self):
        self.left_image = self.left_base_image.copy()
        self.right_image = self.right_base_image.copy()
        for height in range(2):
            for index, point in enumerate(self.image_points[height]):
                if point is None:
                    continue
                x, y = point
                px = int(x / self.left_scale_x)
                py = int(y / self.left_scale_y)
                cv2.circle(self.left_image, (px, py), 4, color[height], -1)
                cv2.putText(self.left_image, str(index), (px, py), cv2.FONT_HERSHEY_SIMPLEX, 1, color[height], 3)
            for index, point in enumerate(self.map_points[height]):
                if point is None:
                    continue
                x, y = point
                px = int(x / self.right_scale_x)
                py = int(y / self.right_scale_y)
                cv2.circle(self.right_image, (px, py), 4, color[height], -1)
                cv2.putText(self.right_image, str(index), (px, py), cv2.FONT_HERSHEY_SIMPLEX, 1, color[height], 2)
        self.update_images()

    def _point_storage(self, kind):
        if kind == 'image':
            return self.image_points, self.image_counts
        return self.map_points, self.map_counts

    def _record_point(self, kind, height, index, point):
        points, counts = self._point_storage(kind)
        previous_point = points[height][index]
        points[height][index] = point
        if previous_point is None:
            counts[height] += 1
        self.click_history.append((kind, height, index, previous_point))
        return previous_point

    def _nearest_point_index(self, kind, height, display_x, display_y, threshold=20):
        points, _ = self._point_storage(kind)
        scale_x = self.left_scale_x if kind == 'image' else self.right_scale_x
        scale_y = self.left_scale_y if kind == 'image' else self.right_scale_y
        candidates = []
        for index, point in enumerate(points[height]):
            if point is None:
                continue
            point_x = point[0] / scale_x
            point_y = point[1] / scale_y
            distance_squared = (point_x - display_x) ** 2 + (point_y - display_y) ** 2
            candidates.append((distance_squared, index))
        if not candidates:
            return None
        distance_squared, index = min(candidates)
        return index if distance_squared <= threshold ** 2 else None

    def _begin_point_drag(self, kind, event):
        if self.capturing or event.button() != Qt.LeftButton:
            return False
        index = self._nearest_point_index(
            kind,
            self.height,
            event.pos().x(),
            event.pos().y(),
        )
        if index is None:
            return False
        points, _ = self._point_storage(kind)
        self.dragging_point = (
            kind,
            self.height,
            index,
            tuple(points[self.height][index]),
        )
        return True

    def _drag_point(self, kind, event):
        if self.dragging_point is None or self.dragging_point[0] != kind:
            return
        if hasattr(event, 'buttons') and not (event.buttons() & Qt.LeftButton):
            return
        _, height, index, _ = self.dragging_point
        scale_x = self.left_scale_x if kind == 'image' else self.right_scale_x
        scale_y = self.left_scale_y if kind == 'image' else self.right_scale_y
        label = self.left_top_label if kind == 'image' else self.right_top_label
        display_x = min(max(0, event.pos().x()), label.width() - 1)
        display_y = min(max(0, event.pos().y()), label.height() - 1)
        point = (
            int(display_x * scale_x),
            int(display_y * scale_y),
        )
        points, _ = self._point_storage(kind)
        points[height][index] = point
        self.redraw_points()

    def _finish_point_drag(self, kind, event):
        if self.dragging_point is None or self.dragging_point[0] != kind:
            return
        drag = self.dragging_point
        self._drag_point(kind, event)
        self.dragging_point = None
        _, height, index, original_point = drag
        points, _ = self._point_storage(kind)
        current_point = tuple(points[height][index])
        if current_point == original_point:
            return
        self.click_history.append((kind, height, index, original_point))
        point_name = '图像点' if kind == 'image' else '地图点'
        self.append_text(
            f'修改{point_name} {height}-{index}：'
            f'({current_point[0]}, {current_point[1]})'
        )

    def undo_last_point(self):
        if self.capturing:
            return
        if not self.click_history:
            self.append_text('没有可撤回的标定点')
            return
        kind, height, index, previous_point = self.click_history.pop()
        points, counts = self._point_storage(kind)
        if previous_point is None:
            points[height][index] = None
            counts[height] = max(0, counts[height] - 1)
        else:
            points[height][index] = previous_point
        point_name = '图像点' if kind == 'image' else '地图点'
        action_name = '撤回' if previous_point is None else '还原'
        self.append_text(f'{action_name}{point_name} {height}-{index}')
        self.height = height
        self.redraw_points()

    def left_top_clicked(self, event):
        # 图像点击事件
        if not self.capturing:
            if self._begin_point_drag('image', event):
                return
            if self.image_counts[self.height] >= self.points_per_layer:
                self.append_text('图像点已满；可拖动已有编号点进行修改')
                return
            x = int(event.pos().x() * self.left_scale_x)
            y = int(event.pos().y() * self.left_scale_y)
            index = self.image_points[self.height].index(None)
            self._record_point('image', self.height, index, (x, y))
            self.redraw_points()
            self.append_text(f'图像点 {self.height}-{index}：({x}, {y})')

    def left_top_dragged(self, event):
        self._drag_point('image', event)

    def left_top_released(self, event):
        self._finish_point_drag('image', event)

    def right_top_clicked(self, event):
        # 地图点击事件
        if not self.capturing:
            if self._begin_point_drag('map', event):
                return
            if self.map_counts[self.height] >= self.points_per_layer:
                self.append_text('地图点已满；可拖动已有编号点进行修改')
                return
            x = int(event.pos().x() * self.right_scale_x)
            y = int(event.pos().y() * self.right_scale_y)
            index = self.map_points[self.height].index(None)
            self._record_point('map', self.height, index, (x, y))
            self.redraw_points()
            self.append_text(f'地图点 {self.height}-{index}：({x}, {y})')

    def right_top_dragged(self, event):
        self._drag_point('map', event)

    def right_top_released(self, event):
        self._finish_point_drag('map', event)

    def button1_clicked(self):
        # 按钮1点击事件
        if not self.capturing:
            return
        self.append_text('开始标定')
        self.capturing = False
        self.button1.setEnabled(False)
        for button in self.opponent_group.buttons():
            button.setEnabled(False)

        print('开始标定')

    def button2_clicked(self):
        # 按钮2点击事件
        self.append_text('切换高度')
        self.height = (self.height + 1) % 2
        print('切换高度')

    def button3_clicked(self):
        self.choose_and_import_preset()

    def _matching_presets(self, require_current_point_count=True):
        point_count = self.points_per_layer if require_current_point_count else None
        return list_presets(
            self.preset_path,
            side=self.state,
            points_per_layer=point_count,
        )

    def _choose_preset(self, title, require_current_point_count=True):
        try:
            presets = self._matching_presets(require_current_point_count)
        except CalibrationPresetError as error:
            QMessageBox.critical(self, '地图预制方案读取失败', str(error))
            return None
        if not presets:
            own_name = '红方' if self.state == 'R' else '蓝方'
            count_hint = (
                f'、每层 {self.points_per_layer} 点'
                if require_current_point_count else ''
            )
            QMessageBox.information(
                self,
                title,
                f'暂无适用于己方{own_name}{count_hint}的地图预制方案。',
            )
            return None

        labels = [
            (
                preset['name']
                if require_current_point_count
                else f"{preset['name']}（{preset['points_per_layer']} 点/层）"
            )
            for preset in presets
        ]
        selected_label, accepted = QInputDialog.getItem(
            self,
            title,
            '选择方案：',
            labels,
            0,
            False,
        )
        if not accepted:
            return None
        return presets[labels.index(selected_label)]

    def choose_and_import_preset(self):
        preset = self._choose_preset('导入地图预制方案')
        if preset is not None:
            self.import_preset(preset['name'])

    def import_preset(self, name, confirm=True):
        try:
            preset = get_preset(name, self.state, self.preset_path)
        except CalibrationPresetError as error:
            QMessageBox.critical(self, '导入地图预制方案失败', str(error))
            self.append_text(f'导入地图预制方案失败：{error}')
            return False
        if preset['points_per_layer'] != self.points_per_layer:
            message = (
                f"方案“{preset['name']}”为每层 {preset['points_per_layer']} 点，"
                f'当前设置为每层 {self.points_per_layer} 点，不能导入。'
            )
            QMessageBox.warning(self, '导入地图预制方案失败', message)
            self.append_text(message)
            return False

        has_current_map_points = any(self.map_counts)
        if confirm and has_current_map_points:
            choice = QMessageBox.question(
                self,
                '替换当前地图点',
                f"导入“{preset['name']}”会替换当前已选地图点，是否继续？",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if choice != QMessageBox.Yes:
                return False

        self.map_points = [
            [tuple(point) for point in layer]
            for layer in preset['map_points']
        ]
        self.map_counts = [len(layer) for layer in self.map_points]
        self.click_history = [
            action for action in self.click_history if action[0] != 'map'
        ]
        for height in range(2):
            for index in range(self.points_per_layer):
                self.click_history.append(('map', height, index, None))
        self.dragging_point = None
        self.height = 0
        if self.capturing:
            self.button1_clicked()
        self.redraw_points()
        message = (
            f"已导入地图预制方案“{preset['name']}”；"
            '图像点仍需手动标定，地图点可拖动修改或按原方式撤回重选'
        )
        self.append_text(message)
        print(message)
        return True

    def save_current_preset(self):
        for height in range(2):
            if self.map_counts[height] != self.points_per_layer:
                QMessageBox.warning(
                    self,
                    '无法保存地图预制方案',
                    (
                        '请先完成两个高度层的全部地图点。\n'
                        f'高度 {height}：地图点 '
                        f'{self.map_counts[height]}/{self.points_per_layer}'
                    ),
                )
                return

        name, accepted = QInputDialog.getText(
            self,
            '保存地图预制方案',
            '方案名称：',
        )
        if not accepted:
            return
        normalized_name = name.strip()
        try:
            existing_names = {
                preset['name']
                for preset in self._matching_presets(require_current_point_count=False)
            }
        except CalibrationPresetError as error:
            QMessageBox.critical(self, '地图预制方案读取失败', str(error))
            return

        overwrite = normalized_name in existing_names
        if overwrite:
            choice = QMessageBox.question(
                self,
                '覆盖地图预制方案',
                f'己方当前阵营已存在“{normalized_name}”，是否覆盖？',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if choice != QMessageBox.Yes:
                return
        try:
            preset = save_preset(
                normalized_name,
                self.state,
                self.map_points,
                path=self.preset_path,
                overwrite=overwrite,
            )
        except (CalibrationPresetError, OSError) as error:
            QMessageBox.critical(self, '保存地图预制方案失败', str(error))
            return
        message = f"已保存地图预制方案“{preset['name']}”"
        self.append_text(message)
        QMessageBox.information(self, '地图预制方案已保存', message)

    def choose_and_delete_preset(self):
        preset = self._choose_preset(
            '删除地图预制方案',
            require_current_point_count=False,
        )
        if preset is None:
            return
        choice = QMessageBox.question(
            self,
            '删除地图预制方案',
            f"确定删除“{preset['name']}”吗？此操作无法撤回。",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if choice != QMessageBox.Yes:
            return
        try:
            delete_preset(preset['name'], self.state, self.preset_path)
        except (CalibrationPresetError, OSError) as error:
            QMessageBox.critical(self, '删除地图预制方案失败', str(error))
            return
        self.append_text(f"已删除地图预制方案“{preset['name']}”")

    def button4_clicked(self):
        # 按钮4点击事件
        print(self.image_points)
        print(self.map_points)
        matrices = []
        for i in range(0, 2):
            if self.image_counts[i] != self.points_per_layer or self.map_counts[i] != self.points_per_layer:
                msg = (
                    f"高度 {i} 点数不足：图像点 {self.image_counts[i]}/{self.points_per_layer}, "
                    f"地图点 {self.map_counts[i]}/{self.points_per_layer}"
                )
                self.append_text(msg)
                print(msg)
                return
            image_point = np.array(self.image_points[i], dtype=np.float32)
            map_point = np.array(self.map_points[i], dtype=np.float32)
            if self.points_per_layer == 4:
                matrix = cv2.getPerspectiveTransform(image_point, map_point)
            else:
                matrix, inliers = cv2.findHomography(image_point, map_point, method=0)
                if matrix is None:
                    msg = f"高度 {i} 多点拟合失败，请检查点位是否共线或顺序错误"
                    self.append_text(msg)
                    print(msg)
                    return
            self.T.append(matrix)
            matrices.append(matrix)

            projected = cv2.perspectiveTransform(image_point.reshape(-1, 1, 2), matrix).reshape(-1, 2)
            errors = np.linalg.norm(projected - map_point, axis=1)
            print(
                f"height={i} reprojection error px: "
                f"mean={errors.mean():.2f}, max={errors.max():.2f}, all={np.round(errors, 2).tolist()}"
            )

        update_config_state(self.state)
        np.save(self.save_path, self.T)
        points_save_path = self.save_path.replace('.npy', '_points.npz')
        np.savez(
            points_save_path,
            matrices=np.array(matrices),
            image_points=np.array(self.image_points, dtype=np.float32),
            map_points=np.array(self.map_points, dtype=np.float32),
        )

        self.append_text('保存计算')
        print('保存计算', self.save_path)
        print('保存标定点', points_save_path)
        time.sleep(1)
        if bool(config.get('calibration', {}).get('run_after_save', False)):
            print('标定完成，自动启动 main.py')
            sys.stdout.flush()
            os.execv(sys.executable, [sys.executable, '-u', 'main.py'])
        sys.exit()

    def convert_cvimage_to_pixmap(self, cvimage):
        height, width, channel = cvimage.shape
        bytes_per_line = 3 * width
        qimage = QImage(cvimage.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimage)
        return pixmap

    def set_button_style(self, button):
        button.setStyleSheet("QPushButton { font-size: 18px; }")

    def append_text(self, text):
        # 在文本组件中追加文本
        current_text = self.left_bottom_text.toPlainText()
        self.left_bottom_text.setPlainText(current_text + '\n' + text)
        # 自动向下滚动文本组件
        cursor = self.left_bottom_text.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.left_bottom_text.setTextCursor(cursor)


if __name__ == '__main__':
    camera_mode = config['global']['camera_mode']  # 'test':测试模式,'hik':海康相机,'video':USB相机（videocapture）
    camera_image = None
    state = config['global']['state']  # R:红方/B:蓝方

    if camera_mode == 'test':
        camera_image = read_test_calibration_image()
    elif camera_mode == 'hik':
        # 海康相机图像获取线程
        thread_camera = threading.Thread(target=hik_camera_get, daemon=True)
        thread_camera.start()
    elif camera_mode == 'video':
        # USB相机图像获取线程
        thread_camera = threading.Thread(target=video_capture_get, daemon=True)
        thread_camera.start()

    while camera_image is None:
        print("等待图像。。。")
        time.sleep(0.5)
    app = QApplication(sys.argv)
    myui = MyUI()
    sys.exit(app.exec_())
