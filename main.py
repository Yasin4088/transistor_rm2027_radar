# 2026赛季新地图
# 两层仿射变换 0和300
# 盲区预测
# 门控卡尔曼滤波
# config
import atexit
import json
import os
import re
import signal
import sys
import threading
import time
import struct
import random
import string
from collections import deque
from pathlib import Path
import serial
# 项目目录规整后：业务模块在 src/，海康 SDK 绑定在 tools/，统一加入导入路径
_HERE = Path(__file__).resolve().parent
for _sub in ("src", "tools"):
    if str(_HERE / _sub) not in sys.path:
        sys.path.insert(0, str(_HERE / _sub))
from capture.camera_availability import wait_for_initial_camera_frame
from ui.information_ui import draw_information_ui
import cv2
import numpy as np
import open3d as o3d
from detect.detect_function import YOLOv5Detector
from capture.frame_rate import RecentFrameRate
from RM_serial_py.ser_api import build_send_packet, receive_packet, Radar_decision, \
    build_data_decision, build_data_radar_all, generate_random_password
from output.runtime_status import initialize_runtime_status, update_runtime_status
from output.referee_transport import RefereeTransport, normalize_transport_mode
from output.recording_storage import prepare_match_recording_directory
from output.vision_telemetry import build_vision_telemetry, classify_legacy_target_names
from output.video_recorder import AsyncMatchVideoRecorder
from detect.vehicle_color import VehicleColorMemory, analyze_armor_light_color
import yaml


def _load_hik_sdk():
    """Load Hikvision SDK only when hik camera mode is actually used."""
    from capture.hik_camera import get_Value, hik_device_serial, image_control, select_hik_device_index, set_Value, \
        start_grab_and_get_data_size

    if sys.platform.startswith("linux"):
        from MvImport_Linux import MvCameraControl_class as mv
    elif sys.platform.startswith("win"):
        from MvImport import MvCameraControl_class as mv
    else:
        raise RuntimeError(f"Unsupported platform for MvCameraControl_class: {sys.platform}")

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

# 切换到脚本所在目录，保证从任意目录启动（如 test1 别名）时，config.yaml 及其中的相对路径（模型、图片、视频）都能被正确找到。（方便调试）
os.chdir(Path(__file__).resolve().parent)

with open("config/config.yaml", "r", encoding="utf-8") as f:  # 指定 UTF-8 编码
    config = yaml.safe_load(f)

_runtime_status_error_reported = False


def report_runtime_status(**values):
    """Report launcher status without allowing monitoring failures to stop the match."""
    global _runtime_status_error_reported
    try:
        update_runtime_status(**values)
    except OSError as error:
        if not _runtime_status_error_reported:
            print(f"运行状态写入失败，比赛主程序继续运行: {error}")
            _runtime_status_error_reported = True


stop_requested = False


def request_stop(signum, frame):
    global stop_requested
    stop_requested = True
    print(f"收到退出信号 {signum}，正在关闭录像与比赛程序...")


for handled_signal in (signal.SIGINT, signal.SIGTERM):
    signal.signal(handled_signal, request_stop)


referee_mode = normalize_transport_mode(config.get('referee', {}).get('transport', 'radio_ros'))
telemetry_rate_hz = min(max(float(config.get('referee', {}).get('telemetry_rate_hz', 20.0)), 1.0), 30.0)
serial_requested = referee_mode == 'legacy_serial'
referee_transport = None
match_run_id = str(os.environ.get('TRANSISTOR_MATCH_RUN_ID', '') or '').strip()
if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,87}', match_run_id):
    match_run_id = f"vision_{time.strftime('%Y%m%d_%H%M%S')}_{os.getpid()}"
try:
    initialize_runtime_status(
        process_id=os.getpid(),
        match_run_id=match_run_id,
        started_at=time.time(),
        heartbeat_at=time.time(),
        phase="starting",
        state=config.get('global', {}).get('state'),
        camera_mode=config.get('global', {}).get('camera_mode'),
        serial_enabled=serial_requested,
        serial_port=config.get('serial', {}).get('port'),
        referee_transport=referee_mode,
    )
except OSError as error:
    print(f"运行状态初始化失败，比赛主程序继续运行: {error}")
    _runtime_status_error_reported = True

# 本脚本作为入口运行时的时间原点（用于双倍易伤「启动窗口」等）
main_program_start_time = time.time()

state = config['global']['state']  # R:红方/B:蓝方
algorithm_mode = str(config.get('algorithm', {}).get('mode', 'legacy')).lower()
if algorithm_mode not in ('hkust_tracker', 'legacy'):
    print(f"未知算法模式 {algorithm_mode}，已回退到 legacy（2131aef 视觉链路）")
    algorithm_mode = 'legacy'
if algorithm_mode == 'hkust_tracker':
    from detect.tracking import TrackedRadarPipeline
    from detect.tracking.types import TargetState
tracked_targets_lock = threading.Lock()
tracked_targets = {}
multi_car_recognition = bool(config.get('global', {}).get('multi_car_recognition', True))
assert config['filter']['type'] in ['sliding_window', 'hybrid_gated_kalman', 'hybrid'], \
    "滤波器类型必须为sliding_window/hybrid_gated_kalman/hybrid"
if config['filter']['type'] == 'sliding_window':
    assert 1 <= config['filter']['sliding_window']['window_size'] <= 20, "滑动窗口大小应在1-20之间"
# 图像测试模式
camera_mode = config['global']['camera_mode']  # 'test':测试模式,'hik':海康相机,'video':USB相机（videocapture）
serial_error = None
if referee_mode == 'legacy_serial':
    try:
        ser1 = serial.Serial(
            config['serial']['port'],
            config['serial']['baudrate'],
            timeout=config['serial']['timeout']
        )
        print(f"串口已连接：{config['serial']['port']}")
    except Exception as e:
        print(f"串口连接失败：{str(e)}")
        serial_error = str(e)
        ser1 = None
        config['global']['use_serial'] = False  # 自动禁用旧串口功能
else:
    ser1 = None
    print("裁判通信模式: radio_ros（本项目不会打开裁判串口）")
report_runtime_status(
    serial_enabled=serial_requested,
    serial_open=ser1 is not None,
    serial_error=serial_error,
    referee_transport=referee_mode,
)
record_video = bool(config.get('global', {}).get('save_img', False))
recording_config = config.get('recording', {})
recording_fps = float(recording_config.get('fps', 20.0))
recording_codec = str(recording_config.get('codec', 'h264_nvenc'))
recording_max_catchup_seconds = float(recording_config.get('max_catchup_seconds', 1.0))
recording_queue_size = int(recording_config.get('queue_size', 4))
recording_map_max_width = int(recording_config.get('map_max_width', 1920))
recording_camera_max_width = int(recording_config.get('camera_max_width', 2560))
recording_default_root = Path(recording_config.get('default_root', 'save_img'))
if not recording_default_root.is_absolute():
    recording_default_root = Path(__file__).resolve().parent / recording_default_root
recording_selected_root_text = str(recording_config.get('selected_root', '') or '').strip()
recording_selected_root = Path(recording_selected_root_text) if recording_selected_root_text else None
if recording_selected_root is not None and not recording_selected_root.is_absolute():
    recording_selected_root = Path(__file__).resolve().parent / recording_selected_root
recording_prefer_external = bool(recording_config.get('prefer_external', True))
recording_external_directory = str(
    recording_config.get('external_directory', 'Transistor-radar-recordings')
)
report_runtime_status(recording_requested=record_video, recording=False)


compare_cfg = config.get('filter_compare', {})
compare_enabled = bool(compare_cfg.get('enabled', False))
compare_draw = bool(compare_cfg.get('draw_on_map', True))
compare_text = bool(compare_cfg.get('show_delta_text', True))
compare_type = str(compare_cfg.get('compare_type', 'hybrid_gated_kalman'))

ui_cfg = config.get('ui', {})
show_armor_canvas = bool(ui_cfg.get('show_armor_canvas', False))

projection_cfg = config.get('projection', {})
projection_point_source = str(projection_cfg.get('point_source', 'car_bottom'))
projection_car_x_ratio = float(projection_cfg.get('car_x_ratio', 0.5))
projection_car_y_ratio = float(projection_cfg.get('car_y_ratio', 0.95))
projection_armor_x_ratio = float(projection_cfg.get('armor_x_ratio', 0.5))
projection_armor_y_ratio = float(projection_cfg.get('armor_y_ratio', 1.5))

inference_cfg = config.get('inference', {})
car_img_size = tuple(int(value) for value in inference_cfg.get('car_img_size', [640, 640]))
armor_img_size = tuple(int(value) for value in inference_cfg.get('armor_img_size', [640, 640]))
armor_canvas_size = tuple(int(value) for value in inference_cfg.get('armor_canvas_size', armor_img_size[::-1]))
armor_conf_threshold = min(max(float(inference_cfg.get('armor_conf_threshold', 0.4)), 0.0), 1.0)
vehicle_tracker_cfg = config.get('algorithm', {}).get('vehicle_tracker', {})
vehicle_low_confidence = min(
    max(float(vehicle_tracker_cfg.get('low_confidence', 0.10)), 0.0),
    1.0,
)
vehicle_high_confidence = min(
    max(float(vehicle_tracker_cfg.get('high_confidence', 0.30)), 0.0),
    1.0,
)
if vehicle_low_confidence > vehicle_high_confidence:
    raise ValueError(
        'algorithm.vehicle_tracker.low_confidence 不能高于 high_confidence'
    )
if len(car_img_size) != 2 or len(armor_img_size) != 2 or len(armor_canvas_size) != 2:
    raise ValueError('inference 中的图像尺寸必须是由两个整数组成的 [高, 宽] 或 [宽, 高]')

armor_selection_cfg = config.get('armor_selection', {})
armor_selection_mode = str(armor_selection_cfg.get('mode', 'center')).lower()
if armor_selection_mode not in ('center', 'confidence'):
    print(f"未知装甲板选择模式 {armor_selection_mode}，已回退到 center")
    armor_selection_mode = 'center'

vehicle_color_cfg = config.get('vehicle_color_hold', {})
vehicle_color_hold_enabled = bool(vehicle_color_cfg.get('enabled', True))
vehicle_color_min_saturation = int(vehicle_color_cfg.get('min_saturation', 70))
vehicle_color_min_value = int(vehicle_color_cfg.get('min_value', 55))
vehicle_color_min_pixels = int(vehicle_color_cfg.get('min_pixels', 4))
vehicle_color_dominance_ratio = float(vehicle_color_cfg.get('dominance_ratio', 1.5))
vehicle_color_side_strip_ratio = float(vehicle_color_cfg.get('side_strip_ratio', 0.28))
vehicle_color_yellow_as_red = bool(vehicle_color_cfg.get('yellow_as_red', True))
vehicle_color_yellow_hue_min = int(vehicle_color_cfg.get('yellow_hue_min', 16))
vehicle_color_yellow_hue_max = int(vehicle_color_cfg.get('yellow_hue_max', 40))
vehicle_color_require_pair = bool(vehicle_color_cfg.get('require_light_bar_pair', True))
vehicle_color_bar_min_aspect = float(vehicle_color_cfg.get('bar_min_aspect_ratio', 1.3))
vehicle_color_bar_min_height = float(vehicle_color_cfg.get('bar_min_height_ratio', 0.12))
vehicle_color_bar_min_similarity = float(vehicle_color_cfg.get('bar_min_height_similarity', 0.4))
vehicle_color_bar_max_y_offset = float(vehicle_color_cfg.get('bar_max_y_offset_ratio', 0.8))
vehicle_color_bar_max_angle_diff = float(vehicle_color_cfg.get('bar_max_angle_difference', 35.0))
vehicle_color_allow_single_bar = bool(vehicle_color_cfg.get('allow_single_light_bar', True))
vehicle_color_single_bar_confidence_scale = float(
    vehicle_color_cfg.get('single_bar_confidence_scale', 0.55)
)
vehicle_color_allow_compact_pair = bool(vehicle_color_cfg.get('allow_compact_light_pair', True))
vehicle_color_compact_pair_max_height = int(vehicle_color_cfg.get('compact_pair_max_armor_height', 16))
vehicle_color_compact_pair_min_aspect = float(vehicle_color_cfg.get('compact_pair_min_aspect_ratio', 1.0))
vehicle_color_compact_pair_confidence_scale = float(
    vehicle_color_cfg.get('compact_pair_confidence_scale', 0.7)
)
vehicle_color_debug_overlay = bool(vehicle_color_cfg.get('debug_overlay', False))
vehicle_color_memory = VehicleColorMemory(
    max_center_distance=float(vehicle_color_cfg.get('max_center_distance', 120.0)),
    max_size_change_ratio=float(vehicle_color_cfg.get('max_size_change_ratio', 0.5)),
    min_box_iou=float(vehicle_color_cfg.get('min_box_iou', 0.3)),
    max_missed_frames=int(vehicle_color_cfg.get('max_missed_frames', 1)),
    confirmation_count=int(vehicle_color_cfg.get('confirmation_count', 3)),
    switch_confirmation_count=int(vehicle_color_cfg.get('switch_confirmation_count', 5)),
    single_bar_confirmation_count=int(vehicle_color_cfg.get('single_bar_confirmation_count', 5)),
    compact_pair_confirmation_count=int(vehicle_color_cfg.get('compact_pair_confirmation_count', 5)),
    model_only_confirmation_count=int(vehicle_color_cfg.get('model_only_confirmation_count', 5)),
    model_only_min_confidence=float(vehicle_color_cfg.get('model_only_min_confidence', 0.75)),
    model_only_max_conflicting_light_confidence=float(
        vehicle_color_cfg.get('model_only_max_conflicting_light_confidence', 0.4)
    ),
    require_model_agreement=bool(vehicle_color_cfg.get('require_model_agreement', True)),
)

armor_confirmation_cfg = config.get('armor_confirmation', {})
armor_confirmation_enabled = bool(armor_confirmation_cfg.get('enabled', True))
armor_confirmation_required_count = max(1, int(armor_confirmation_cfg.get('required_count', 2)))
armor_confirmation_max_dist = float(armor_confirmation_cfg.get('max_center_distance', 120.0))
armor_confirmation_max_missed = max(0, int(armor_confirmation_cfg.get('max_missed_frames', 3)))
armor_confirmation_allowed_numbers = {
    str(number) for number in armor_confirmation_cfg.get('allowed_numbers', [1, 2, 3, 4, 7])
}
armor_confirmation_tracks = []
frame_index = 0

armor_duplicate_cfg = config.get('armor_duplicate', {})
armor_duplicate_enabled = bool(armor_duplicate_cfg.get('enabled', True))
armor_duplicate_scope = str(armor_duplicate_cfg.get('scope', 'number')).lower()
if armor_duplicate_scope not in ('number', 'class'):
    print(f"未知重复装甲板去重范围 {armor_duplicate_scope}，已回退到 number")
    armor_duplicate_scope = 'number'

occlusion_hold_cfg = config.get('occlusion_hold', {})
occlusion_hold_enabled = bool(occlusion_hold_cfg.get('enabled', True))
occlusion_hold_max_time = float(occlusion_hold_cfg.get('max_hold_time', 0.0) or 0.0)
occlusion_hold_cache = {}

temporary_death_cfg = config.get('temporary_death_hold', {})
temporary_death_enabled = bool(temporary_death_cfg.get('enabled', True))
temporary_death_max_dist = float(temporary_death_cfg.get('max_center_distance', 80.0))
temporary_death_max_size_change = float(temporary_death_cfg.get('max_size_change_ratio', 0.35))
temporary_death_update_position = bool(temporary_death_cfg.get('update_position_from_car_box', True))
temporary_death_box_lost_timeout = float(temporary_death_cfg.get('box_lost_timeout', 3.0))

referee_send_cfg = config.get('referee_send', {})
referee_send_correct_only = bool(referee_send_cfg.get('correct_only', True))
referee_valid_names = set()
referee_occlusion_names = set()
referee_valid_names_lock = threading.Lock()


def publish_referee_target_states(valid_names, occlusion_names):
    global referee_valid_names, referee_occlusion_names
    with referee_valid_names_lock:
        referee_valid_names = set(valid_names)
        referee_occlusion_names = set(occlusion_names)


def get_referee_target_states():
    with referee_valid_names_lock:
        return set(referee_valid_names), set(referee_occlusion_names)


def _armor_number(armor_cls):
    match = re.search(r'(\d+)$', str(armor_cls))
    return match.group(1) if match else None


def _armor_duplicate_key(armor_cls):
    if armor_duplicate_scope == 'class':
        return str(armor_cls)
    return _armor_number(armor_cls)


def confirm_armor_detection(armor_cls, car_box, current_frame):
    """Return (confirmed_cls, streak_count); confirmed_cls is None until the class is stable."""
    armor_number = _armor_number(armor_cls)
    if armor_number not in armor_confirmation_allowed_numbers:
        return None, 0
    if not armor_confirmation_enabled:
        return armor_cls, armor_confirmation_required_count

    left, top, car_w, car_h = car_box
    center_x = left + car_w * 0.5
    center_y = top + car_h * 0.5
    max_dist2 = armor_confirmation_max_dist ** 2

    armor_confirmation_tracks[:] = [
        track for track in armor_confirmation_tracks
        if current_frame - track['last_seen_frame'] <= armor_confirmation_max_missed
    ]

    best_track = None
    best_dist2 = max_dist2
    for track in armor_confirmation_tracks:
        if track.get('updated_frame') == current_frame:
            continue
        dx = center_x - track['center'][0]
        dy = center_y - track['center'][1]
        dist2 = dx * dx + dy * dy
        if dist2 <= best_dist2:
            best_dist2 = dist2
            best_track = track

    if best_track is None:
        best_track = {
            'center': (center_x, center_y),
            'cls': armor_cls,
            'count': 1,
            'last_seen_frame': current_frame,
            'updated_frame': current_frame,
        }
        armor_confirmation_tracks.append(best_track)
    else:
        was_previous_frame = best_track['last_seen_frame'] == current_frame - 1
        if best_track['cls'] == armor_cls and was_previous_frame:
            best_track['count'] += 1
        else:
            best_track['cls'] = armor_cls
            best_track['count'] = 1
        best_track['center'] = (center_x, center_y)
        best_track['last_seen_frame'] = current_frame
        best_track['updated_frame'] = current_frame

    if best_track['count'] >= armor_confirmation_required_count:
        return armor_cls, best_track['count']
    return None, best_track['count']


def _car_box_signature(car_box):
    left, top, car_w, car_h = car_box
    return {
        'center': (float(left + car_w * 0.5), float(top + car_h * 0.5)),
        'size': (float(car_w), float(car_h)),
    }


def _same_car_box(car_box, cached_box):
    if cached_box is None:
        return False
    current = _car_box_signature(car_box)
    dx = current['center'][0] - cached_box['center'][0]
    dy = current['center'][1] - cached_box['center'][1]
    if dx * dx + dy * dy > temporary_death_max_dist * temporary_death_max_dist:
        return False
    cw, ch = current['size']
    ow, oh = cached_box['size']
    if ow <= 1 or oh <= 1:
        return False
    width_change = abs(cw - ow) / ow
    height_change = abs(ch - oh) / oh
    return width_change <= temporary_death_max_size_change and height_change <= temporary_death_max_size_change


_raycast_miss_count = 0  # 3D 射线未命中计数（排查用）


def project_image_point_to_map(point_x, point_y):
    """像素 → 场地坐标。按 config 的 projection.mode 走 2D 仿射或 3D 射线
    3D 模式下未命中 mesh 返回 None（上层跳过，不参与上报）"""
    global _raycast_miss_count
    point_x = min(max(point_x, 0), img_x)
    point_y = min(max(point_y, 0), img_y)

    # --- 2D 仿射路径（M_ground/M_height_r 由改动二加载） ---
    if projection_mode == 'affine' and M_ground is not None:
        camera_point = np.array([[[point_x, point_y]]], dtype=np.float32)
        mapped_point = cv2.perspectiveTransform(camera_point.reshape(1, 1, 2), M_ground)
        x_c = max(int(mapped_point[0][0][0]), 0)
        y_c = max(int(mapped_point[0][0][1]), 0)
        x_c = min(x_c, width)
        y_c = min(y_c, height)
        color = mask_image[y_c, x_c]
        if color[0] == color[1] == color[2] == 0:
            return x_c, y_c
        mapped_point = cv2.perspectiveTransform(camera_point.reshape(1, 1, 2), M_height_r)
        x_c = max(int(mapped_point[0][0][0]), 0)
        y_c = max(int(mapped_point[0][0][1]), 0)
        x_c = min(x_c, width)
        y_c = min(y_c, height)
        return x_c, y_c

    # --- 3D 射线路径 ---
    if pixel_to_world is not None:
        world = pixel_to_world((point_x, point_y))
        if world is not None:
            wx, wz = world[0], world[2]
            # 场地尺寸从 config 读取（半长，米制：mesh 是 ±14m/±7.5m 体系）
            field_long_m = field_long_half_cm / 100.0   # 长边半长（14.0m）
            field_short_m = field_short_half_cm / 100.0  # 短边半长（7.5m）
            if state == 'R':
                map_x = (-wz + field_long_m) * 100.0
                map_y = (-wx + field_short_m) * 100.0
            else:
                map_x = (2.0 * field_long_m - (-wz + field_long_m)) * 100.0
                map_y = (2.0 * field_short_m - (-wx + field_short_m)) * 100.0
            return map_y, map_x
        # 未命中：显式暴露（计数+节流日志），不静默降级
        global _raycast_miss_count
        _raycast_miss_count += 1
        if _raycast_miss_count % 100 == 1:
            print(f"[射线] 未命中 mesh ({int(point_x)},{int(point_y)}) 累计{_raycast_miss_count}次")
        return None

    # --- 射线模式未初始化（启动失败兜底） ---
    return point_x, point_y


def convert_projected_map_point(xy):
    """Convert the internal projected point to display and referee coordinates."""
    if state == 'R':
        display_xy = (2800.0 - float(xy[1]), float(xy[0]))
    else:
        display_xy = (float(xy[1]), 1500.0 - float(xy[0]))
    referee_xy = (display_xy[0], 1500.0 - display_xy[1])
    return display_xy, referee_xy


def add_position_measurement(name, x, y, car_box=None, temporary_dead=False):
    update_occlusion_hold(name, x, y, car_box, temporary_dead)
    if isinstance(filter, SlidingWindowFilter):#这里两种端口一样
        filter.add_data(name, x, y)
    else:
        filter.add_data(name, x, y)
    if compare_filter is not None: #影子寄存器，对比滑动窗口与卡尔曼滤波
        compare_filter.add_data(name, x, y)


def update_occlusion_hold(name, x, y, car_box=None, temporary_dead=False):
    if not occlusion_hold_enabled:
        return
    occlusion_hold_cache[name] = {
        'xy': (float(x), float(y)),
        'last_seen_time': time.time(),
        'last_seen_frame': frame_index,
        'car_box': _car_box_signature(car_box) if car_box is not None else occlusion_hold_cache.get(name, {}).get('car_box'),
        'temporary_dead': bool(temporary_dead),
    }


def refresh_temporary_death_hold(car_box, excluded_names=None):
    if not occlusion_hold_enabled or not temporary_death_enabled:
        return None
    excluded_names = excluded_names or set()
    best_name = None
    best_dist2 = temporary_death_max_dist * temporary_death_max_dist
    current = _car_box_signature(car_box)
    for name, item in occlusion_hold_cache.items():
        if name in excluded_names:
            continue
        cached_box = item.get('car_box')
        if not _same_car_box(car_box, cached_box):
            continue
        dx = current['center'][0] - cached_box['center'][0]
        dy = current['center'][1] - cached_box['center'][1]
        dist2 = dx * dx + dy * dy
        if dist2 <= best_dist2:
            best_dist2 = dist2
            best_name = name
    if best_name is None:
        return None
    occlusion_hold_cache[best_name]['last_seen_time'] = time.time()
    occlusion_hold_cache[best_name]['last_seen_frame'] = frame_index
    occlusion_hold_cache[best_name]['temporary_dead'] = True
    guess_list[best_name] = False
    return best_name


def apply_occlusion_hold(filtered_data):
    if not occlusion_hold_enabled:
        return filtered_data
    now = time.time()
    merged = dict(filtered_data)
    stale_names = []
    for name, item in occlusion_hold_cache.items():
        age = now - item['last_seen_time']
        timeout = temporary_death_box_lost_timeout if item.get('temporary_dead') else occlusion_hold_max_time
        if timeout > 0 and age > timeout:
            stale_names.append(name)
            continue
        if name not in merged:
            merged[name] = item['xy']
            guess_list[name] = False
    for name in stale_names:
        occlusion_hold_cache.pop(name, None)
    return merged

# 文件路径配置（地图/UI 背景，两个模式都需要）
if state == 'R':
    map_image = cv2.imread(config['paths']['map_images']['red'])
else:
    map_image = cv2.imread(config['paths']['map_images']['blue'])
mask_image = cv2.imread(config['paths']['map_images']['mask'])   # 仿射选层用，保留
map_backup = cv2.imread(config['paths']['map_images']['backup'])

# 投影模式初始化（config 切换 2D 仿射 / 3D 射线）
projection_mode = config.get('projection', {}).get('mode', 'raycast')

M_ground = None
M_height_r = None
pixel_to_world = None

if projection_mode == 'affine':
    # --- 2D 仿射模式（shark 原版） ---
    if state == 'R':
        loaded_arrays = np.load(config['paths']['calibration']['red'])
    else:
        loaded_arrays = np.load(config['paths']['calibration']['blue'])
    M_ground = loaded_arrays[0]      # 地面层
    M_height_r = loaded_arrays[1]    # 高地层
    print("投影模式: 2D 双层仿射")
else:
    # --- 3D 射线模式（raycast） ---
    try:
        from raycast import PixelToWorld, build_pixel_to_world_from_npz
        _mesh = o3d.io.read_triangle_mesh(config['paths']['mesh_path'])
        # 优先加载已标定的外参（extrinsics.npz），否则用默认假外参
        import os as _os
        _ext_path = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), 'extrinsics.npz')
        if _os.path.isfile(_ext_path):
            _ext = np.load(_ext_path)
            _R, _T = _ext['R'], _ext['t']
            pixel_to_world, _calibrated = build_pixel_to_world_from_npz(
                config['paths']['intrinsics_path'], _mesh, R=_R, T=_T)
            print(f"投影模式: 3D 射线定位（已标定外参）, mesh: {config['paths']['mesh_path']}")
        else:
            pixel_to_world, _calibrated = build_pixel_to_world_from_npz(
                config['paths']['intrinsics_path'], _mesh)
            print(f"投影模式: 3D 射线定位, mesh: {config['paths']['mesh_path']}")
            print("⚠️ 警告: 3D 射线定位使用【假外参】(R=I, T=俯视10m)——"
                  "坐标不可用于比赛！需 6 点 PnP 标定后传入真实 R/T")
    except Exception as e:
        print(f"3D 射线定位初始化失败，降级为像素坐标: {e}")

# 确定地图画面像素
height, width = mask_image.shape[:2]
height -= 1
width -= 1

# 场地尺寸（cm）——从 config 读取，3D 射线坐标转换用
# 注意：mesh 坐标是"半长"体系（长边 ±14m = ±1400cm），需除以 2 得半长
field_map_size = tuple(config.get('ui', {}).get('map_size', [2800, 1500])) \
    if config.get('ui', {}).get('map_size') else tuple(config['global'].get('map_size', [2800, 1500]))
field_long_cm = max(field_map_size)    # 长边全长（2800cm）
field_short_cm = min(field_map_size)   # 短边全长（1500cm）
field_long_half_cm = field_long_cm / 2.0   # 长边半长（1400cm = 14m）
field_short_half_cm = field_short_cm / 2.0  # 短边半长（750cm = 7.5m）

# 初始化战场信息UI（易伤情况、双倍易伤次数、双倍易伤触发状态）
information_ui = np.zeros((config['ui']['info_panel_size'][1],
                           config['ui']['info_panel_size'][0], 3), dtype=np.uint8) * 255
information_ui_show = information_ui.copy()
double_vulnerability_chance = -1  # 双倍易伤机会数
opponent_double_vulnerability = -1  # 是否正在触发双倍易伤
target = -1  # 飞镖当前瞄准目标（用于触发双倍易伤）
# 己方加密等级（即对方干扰波难度等级），开局为1，最高为3；来自0x020E 的 bit3～4，仅解析不入决策
encryption_level = 1
key_modifiable = -1  # 密钥是否可修改
chances_flag = 0  # radar_cmd 当前值；只允许单调递增，禁止旧版 0→1→2→0 循环
vulnerability = [-1, -1, -1, -1, -1, -1]  # 易伤情况
mark_progress_bit_cache = {i: -1 for i in range(6)}

# 密钥相关全局变量
current_password = generate_random_password()  # 开局生成随机密钥
password_sent = False  # 开局密钥是否已发送
# 上一帧 0x020E bit5（key_modifiable），用于边沿检测；-1 表示尚未收到有效报文
last_key_modifiable = -1
latest_vision_fps = 0.0
latest_inference_ms = 0.0
camera_frame_rate = RecentFrameRate(window_seconds=2.0)


def set_camera_error(message):
    """Publish a changed camera error without making camera hardware mandatory."""
    global camera_error
    normalized = None if message is None else str(message)
    if globals().get('camera_error') == normalized:
        return
    camera_error = normalized
    if normalized:
        print(normalized)
    report_runtime_status(camera_ready=False, camera_error=normalized)


def publish_camera_frame(image):
    """Publish one newly delivered frame and count it exactly once."""
    global camera_image
    if image is None:
        return False
    set_camera_error(None)
    camera_image = image
    camera_frame_rate.mark()
    return True

# 双倍易伤触发策略配置
_dv_cfg = config['double_vulnerability']
double_vulnerability_enabled = bool(_dv_cfg.get('enabled', False))
trigger_mode = _dv_cfg['trigger_mode']  # 视觉端关闭时为 'disabled'
double_vuln_stable_duration_s = float(_dv_cfg.get('stable_duration_s', 2.0))
double_vuln_startup_window_s = float(_dv_cfg.get('startup_limit_window_s', 120.0))
double_vuln_startup_max_uses = int(_dv_cfg.get('startup_limit_max_uses', 1))
double_vuln_startup_uses = 0  # main 启动窗口内已成功请求双倍易伤的次数
if double_vulnerability_enabled:
    print(
        f"双倍易伤触发模式: {trigger_mode}，条件稳定时长: {double_vuln_stable_duration_s}s，"
        f"主程序启动后 {double_vuln_startup_window_s}s 内至多 {double_vuln_startup_max_uses} 次双倍易伤"
    )
else:
    print("视觉端双倍易伤自主决策已关闭，由无线电程序负责")

# 盲区预测配置
blind_zone_enabled = bool(config.get('blind_zone', {}).get('enabled', True))
blind_zone_roles = {'1', '2', '7'}

# 距离/运动条件持续计时（需 global 在 ser_send 内更新）
distance_condition_true_since = None
motion_condition_true_since = None

# 距离触发相关
if double_vulnerability_enabled and trigger_mode in ['distance', 'both']:
    distance_threshold = _dv_cfg['distance']['threshold']

# 运动趋势触发相关
if double_vulnerability_enabled and trigger_mode in ['motion', 'both']:
    # 记录每个机器人的位置历史（time, (x, y)）
    robot_position_history = {}
    for robot in ['R1', 'R2', 'R3', 'R4', 'R5', 'R6', 'R7', 'B1', 'B2', 'B3', 'B4', 'B5', 'B6', 'B7']:
        robot_position_history[robot] = deque(maxlen=5)  # 保存最近5帧的位置和时间戳
    # 读取运动趋势相关参数
    motion_params = _dv_cfg['motion']

# 盲区预测用：位置历史（无条件初始化，独立于双倍易伤开关）
if 'robot_position_history' not in globals():
    robot_position_history = {}
    for robot in ['R1', 'R2', 'R3', 'R4', 'R5', 'R6', 'R7', 'B1', 'B2', 'B3', 'B4', 'B5', 'B6', 'B7']:
        robot_position_history[robot] = deque(maxlen=5)

# 加载战场地图
map = map_backup.copy()

# 初始化盲区预测列表
guess_list = {
    "B1": True,
    "B2": True,
    "B3": True,
    "B4": True,
    "B5": True,
    "B6": True,
    "B7": True,
    "R1": True,
    "R2": True,
    "R3": True,
    "R4": True,
    "R5": True,
    "R6": True,
    "R7": True
}
# 上次盲区预测时的标记进度
guess_value = {
    "B1": 0,
    "B2": 0,
    "B7": 0,
    "R1": 0,
    "R2": 0,
    "R7": 0
}
# 当前标记进度（用于判断是否预测正确正确）
guess_value_now = {
    "B1": 0,
    "B2": 0,
    "B7": 0,
    "R1": 0,
    "R2": 0,
    "R7": 0
}

# 机器人名字对应ID
mapping_table = {
    "R1": 1,
    "R2": 2,
    "R3": 3,
    "R4": 4,
    "R5": 5,
    "R6": 6,
    "R7": 7,
    "B1": 101,
    "B2": 102,
    "B3": 103,
    "B4": 104,
    "B5": 105,
    "B6": 106,
    "B7": 107
}

# 盲区预测点位
guess_table = {}
for robot, points in config['blind_zone']['points'].items():
    guess_table[robot] = [tuple(point) for point in points]

# ---- 多源坐标融合（信息波 UWB 为主 + 视觉回退 + 盲区兜底）----
fusion_enabled = bool(config.get('fusion', {}).get('enabled', False))
fusion_buffer = None
if fusion_enabled:
    try:
        from fusion.position_fusion import PositionFusionBuffer
        fusion_buffer = PositionFusionBuffer()
        print("多源坐标融合已启用（信息波 UWB 为主 + 视觉回退）")
    except Exception as e:
        print(f"多源坐标融合初始化失败: {e}")


def feed_info_wave_positions(positions_cm):
    """喂入信息波 0x0A01 坐标（{机器人名: (x_cm, y_cm)}，裁判坐标）"""
    if fusion_buffer is not None and positions_cm:
        fusion_buffer.update_info_wave(positions_cm)


def feed_vision_positions(vision_map):
    """喂入视觉坐标（{机器人名: ((x,y)或None, (盲区x,y)或None)}）"""
    if fusion_buffer is not None and vision_map:
        fusion_buffer.update_vision_tracks(vision_map)

# 盲区预测打分参数（照 HKUST guess_pts.py）
guess_velocity_scoring = bool(config.get('blind_zone', {}).get('velocity_scoring', True))
guess_cos_factor = float(config.get('blind_zone', {}).get('cos_factor', 0.003))
guess_d_factor = float(config.get('blind_zone', {}).get('d_factor', 0.01))


class MotionKalman2D:
    def __init__(self, process_noise=1e-5, measurement_noise=1e-1):
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]], dtype=np.float32)
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * float(process_noise)
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * float(measurement_noise)
        self.kf.errorCovPost = np.eye(4, dtype=np.float32)
        self.kf.statePost = np.array([[0], [0], [0], [0]], dtype=np.float32)
        self.last_measurement = None
        self.last_update_time = time.time()

    def update(self, measurement):
        current_time = time.time()
        dt = max(current_time - self.last_update_time, 1e-3)
        self.last_update_time = current_time

        self.kf.transitionMatrix = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]], dtype=np.float32)

        if self.last_measurement is not None:
            dx = float(measurement[0]) - float(self.last_measurement[0])
            dy = float(measurement[1]) - float(self.last_measurement[1])
            self.kf.processNoiseCov[2, 2] = min(0.5, 0.1 + abs(dx) / 10)
            self.kf.processNoiseCov[3, 3] = min(0.5, 0.1 + abs(dy) / 10)

        self.kf.predict()
        z = np.array([[np.float32(measurement[0])], [np.float32(measurement[1])]])
        self.kf.correct(z)
        self.last_measurement = (float(measurement[0]), float(measurement[1]))

    def get_estimate(self):
        if self.last_measurement is None:
            return None
        s = self.kf.statePost
        return float(s[0, 0]), float(s[1, 0])


# 添加滑动窗口滤波器
class SlidingWindowFilter:
    def __init__(self, window_size=5, max_inactive_time=2.0, threshold=100000.0, update_guess=True):
        self.window_size = window_size
        self.max_inactive_time = max_inactive_time
        self.threshold = threshold
        self.update_guess = update_guess
        self.windows = {}
        self.last_update = {}

    def add_data(self, name, x, y):
        if name not in self.windows:
            self.windows[name] = deque(maxlen=self.window_size)

        # 异常值检测
        if len(self.windows[name]) > 0:
            last_x, last_y = self.windows[name][-1]
            if (x - last_x) ** 2 + (y - last_y) ** 2 > self.threshold:
                return

        self.windows[name].append((x, y))
        self.last_update[name] = time.time()

    def get_all_data(self):
        current_time = time.time()
        filtered = {}

        # 清理过期数据
        to_remove = []
        for name in self.windows:
            if current_time - self.last_update.get(name, 0) > self.max_inactive_time:
                to_remove.append(name)
                if self.update_guess:
                    guess_list[name] = True

        for name in to_remove:
            del self.windows[name]
            del self.last_update[name]

        # 计算窗口均值
        for name, window in self.windows.items():
            if len(window) >= self.window_size:
                x_avg = sum(p[0] for p in window) / len(window)
                y_avg = sum(p[1] for p in window) / len(window)
                filtered[name] = (x_avg, y_avg)
                if self.update_guess:
                    guess_list[name] = False

        return filtered


class HybridGatedKalmanFilter:
    def __init__(self,
                 prefilter_window=3,
                 prefilter_mode='median',
                 max_inactive_time=2.0,
                 raw_jump_threshold=120.0,
                 pred_jump_threshold=100.0,
                 process_noise=1e-5,
                 measurement_noise=1e-1,
                 update_guess=True):
        self.prefilter_window = max(1, int(prefilter_window))
        self.prefilter_mode = prefilter_mode
        self.max_inactive_time = float(max_inactive_time)
        self.raw_jump_threshold = float(raw_jump_threshold)
        self.pred_jump_threshold = float(pred_jump_threshold)
        self.update_guess = update_guess

        self.raw_windows = {}
        self.kf_filters = {}
        self.last_update = {}
        self.last_raw = {}
        self.process_noise = float(process_noise)
        self.measurement_noise = float(measurement_noise)

    def _ensure_target(self, name):
        if name not in self.raw_windows:
            self.raw_windows[name] = deque(maxlen=self.prefilter_window)
        if name not in self.kf_filters:
            self.kf_filters[name] = MotionKalman2D(
                process_noise=self.process_noise,
                measurement_noise=self.measurement_noise
            )

    def _prefilter_point(self, points):
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        if self.prefilter_mode == 'mean':
            return float(sum(xs) / len(xs)), float(sum(ys) / len(ys))
        return float(np.median(xs)), float(np.median(ys))

    def add_data(self, name, x, y):
        now = time.time()
        self._ensure_target(name)

        # 原始观测门控：抑制明显跳点
        if name in self.last_raw:
            dx = x - self.last_raw[name][0]
            dy = y - self.last_raw[name][1]
            if dx * dx + dy * dy > self.raw_jump_threshold * self.raw_jump_threshold:
                return

        self.raw_windows[name].append((x, y))
        self.last_raw[name] = (x, y)
        fx, fy = self._prefilter_point(self.raw_windows[name])

        # 卡尔曼预测门控：观测偏离预测过大则丢弃该观测
        kf_obj = self.kf_filters[name]
        if kf_obj.last_measurement is not None:
            pred = kf_obj.get_estimate()
            if pred is not None:
                pdx = fx - pred[0]
                pdy = fy - pred[1]
                if pdx * pdx + pdy * pdy > self.pred_jump_threshold * self.pred_jump_threshold:
                    return

        kf_obj.update((fx, fy))
        self.last_update[name] = now
        if self.update_guess:
            guess_list[name] = False

    def get_all_data(self):
        current_time = time.time()
        filtered = {}
        to_remove = []

        for name in list(self.kf_filters.keys()):
            if current_time - self.last_update.get(name, 0) > self.max_inactive_time:
                to_remove.append(name)
                if self.update_guess:
                    guess_list[name] = True
                continue

            estimate = self.kf_filters[name].get_estimate()
            if estimate is not None:
                filtered[name] = estimate
            if self.update_guess:
                guess_list[name] = False

        for name in to_remove:
            self.raw_windows.pop(name, None)
            self.kf_filters.pop(name, None)
            self.last_update.pop(name, None)
            self.last_raw.pop(name, None)

        return filtered


def create_filter_by_type(config, filter_type, update_guess=True):
    if filter_type == "hybrid":
        filter_type = "hybrid_gated_kalman"
    if filter_type == "sliding_window":
        return SlidingWindowFilter(
            window_size=int(config['filter']['sliding_window']['window_size']),
            max_inactive_time=float(config['filter']['sliding_window']['max_inactive_time']),
            threshold=float(config['filter']['sliding_window']['threshold']),
            update_guess=update_guess
        )
    if filter_type == "hybrid_gated_kalman":
        hybrid_cfg = config.get('filter', {}).get('hybrid_gated_kalman', {})
        return HybridGatedKalmanFilter(
            prefilter_window=int(hybrid_cfg.get('prefilter_window', 3)),
            prefilter_mode=str(hybrid_cfg.get('prefilter_mode', 'median')),
            max_inactive_time=float(hybrid_cfg.get('max_inactive_time', 2.0)),
            raw_jump_threshold=float(hybrid_cfg.get('raw_jump_threshold', 120.0)),
            pred_jump_threshold=float(hybrid_cfg.get('pred_jump_threshold', 100.0)),
            process_noise=float(hybrid_cfg.get('process_noise', 1e-5)),
            measurement_noise=float(hybrid_cfg.get('measurement_noise', 1e-1)),
            update_guess=update_guess
        )
    raise ValueError(f"Unsupported filter type: {filter_type}")


# 滤波器选择
def create_filter(config):
    return create_filter_by_type(config, config['filter']['type'], update_guess=True)


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
            set_camera_error("海康相机枚举失败 ret[0x%x]，等待重试" % ret)
            time.sleep(1.0)
            continue

        if deviceList.nDeviceNum == 0:
            set_camera_error("未发现海康相机，等待设备连接")
            time.sleep(1.0)
        else:
            set_camera_error(None)
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
    # 按 config 中的序列号选择设备；未配置则默认第一台
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

    # 海康相机线程中的硬编码参数
    set_Value(cam, param_type="float_value", node_name="ExposureTime",
              node_value=config['camera_params']['exposure_time'])
    set_Value(cam, param_type="float_value", node_name="Gain", node_value=config['camera_params']['gain'])
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
    while not stop_requested:
        cycle_started_at = time.monotonic()
        ret = cam.MV_CC_GetOneFrameTimeout(pData, nDataSize, stFrameInfo, 1000)
        if ret == 0:
            image = np.asarray(pData)
            # 处理海康相机的图像格式为OPENCV处理的格式
            publish_camera_frame(image_control(data=image, stFrameInfo=stFrameInfo))
        else:
            print("no data[0x%x]" % ret)


def video_capture_get():
    global camera_image
    cam = None
    try:
        while not stop_requested:
            cam = cv2.VideoCapture(0)
            if not cam.isOpened():
                set_camera_error("未发现可用的 USB 相机，等待设备连接")
                cam.release()
                cam = None
                time.sleep(1.0)
                continue

            set_camera_error(None)
            print("USB 相机已连接")
            while not stop_requested:
                ret, img = cam.read()
                if not ret:
                    set_camera_error("USB 相机读取失败，等待重新连接")
                    break
                publish_camera_frame(img)
                time.sleep(0.016)  # 60fps

            cam.release()
            cam = None
            if not stop_requested:
                time.sleep(1.0)
    except Exception as e:
        set_camera_error(f"USB 相机捕获异常：{e}")
    finally:
        if cam is not None and cam.isOpened():
            cam.release()
            print("摄像头资源已释放")

# 测试视频
def _test_video_capture_get_opencv(video_path, loop=True, start_frame=0, playback_speed=1.0):
    global camera_image
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"OpenCV 无法打开测试视频：{video_path}")
        return

    if start_frame and start_frame > 0:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(start_frame))

    fps = cap.get(cv2.CAP_PROP_FPS)
    if not fps or fps <= 1e-6:
        fps = 30.0
    delay = max(1.0 / fps / max(playback_speed, 1e-6), 0.0)

    try:
        while not stop_requested:
            ret, img = cap.read()
            if not ret:
                if loop:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, int(start_frame or 0))
                    continue
                break
            publish_camera_frame(img)
            if delay > 0:
                time.sleep(delay)
    finally:
        cap.release()
        print("测试视频资源已释放（OpenCV）")


def test_video_capture_get(video_path, loop=True, start_frame=0, playback_speed=1.0):
    global camera_image
    try:
        import av
        # Windows 下可用 d3d11va；Linux/其他平台默认软件解码，避免硬件解码参数不兼容。
        av_options = {'hwaccel': 'd3d11va'} if sys.platform.startswith("win") else {}
        container = av.open(video_path, mode='r', options=av_options)
        stream = next((s for s in container.streams if s.type == 'video'), None)
        if stream is None:
            container.close()
            print(f"未找到视频流：{video_path}")
            return
        # 起始帧
        if start_frame and start_frame > 0:
            try:
                container.seek(int(start_frame))
            except Exception:
                pass
        # 延时以接近原始帧率
        fps = float(stream.average_rate) if stream and stream.average_rate else 30.0
        delay = max(1.0 / fps / max(playback_speed, 1e-6), 0.0)
        while not stop_requested:
            got_frame = False
            for packet in container.demux(stream):
                for frame in packet.decode():
                    img = frame.to_ndarray(format='bgr24')
                    publish_camera_frame(img)
                    got_frame = True
                    if delay > 0:
                        time.sleep(delay)
                if got_frame:
                    break
            if not got_frame:
                if loop:
                    try:
                        container.seek(0)
                        continue
                    except Exception:
                        break
                else:
                    break
        container.close()
        print("测试视频资源已释放（PyAV）")
    except Exception as e:
        print(f"测试视频捕获异常（PyAV）：{e}，尝试使用 OpenCV 读取")
        _test_video_capture_get_opencv(video_path, loop, start_frame, playback_speed)


# 串口发送线程
def ser_send():
    if referee_mode == 'legacy_serial' and not ser1:  # 旧串口模式必须有可用端口
        print("串口未启用，发送线程退出")
        return
    print(
        "裁判系统坐标发送: "
        + ("仅已确认标定车辆" if referee_send_correct_only else "所有滤波/预测坐标")
    )
    seq = 0
    global chances_flag
    global guess_value
    global encryption_level
    global current_password
    global password_sent
    global last_key_modifiable
    global distance_condition_true_since
    global motion_condition_true_since
    # 单点预测时间
    guess_time = {
        'B1': 0,
        'B2': 0,
        'B7': 0,
        'R1': 0,
        'R2': 0,
        'R7': 0,
    }
    # 预测点索引
    guess_index = {
        'B1': 0,
        'B2': 0,
        'B7': 0,
        'R1': 0,
        'R2': 0,
        'R7': 0,
    }

    # 发送蓝方机器人坐标
    def send_point_B(send_name, all_filter_data):
        # front_time = time.time()
        # 同一帧内所有机器人使用统一坐标系（由当前己方阵营state决定）
        if state == 'R':
            filtered_xyz = (2800 - all_filter_data[send_name][1], all_filter_data[send_name][0])
        else:
            filtered_xyz = (all_filter_data[send_name][1], 1500 - all_filter_data[send_name][0])
        # 转换为裁判系统单位CM
        ser_x = int(filtered_xyz[0]) * 10 / 10
        ser_y = int(1500 - filtered_xyz[1]) * 10 / 10
        return ser_x, ser_y

    # 发送红方机器人坐标
    def send_point_R(send_name, all_filter_data):
        # front_time = time.time()
        # 同一帧内所有机器人使用统一坐标系（由当前己方阵营state决定）
        if state == 'R':
            filtered_xyz = (2800 - all_filter_data[send_name][1], all_filter_data[send_name][0])
        else:
            filtered_xyz = (all_filter_data[send_name][1], 1500 - all_filter_data[send_name][0])
        # 转换为裁判系统单位CM
        ser_x = int(filtered_xyz[0]) * 10 / 10
        ser_y = int(1500 - filtered_xyz[1]) * 10 / 10
        return ser_x, ser_y

    # 发送盲区预测点坐标
    def score_guess_points(send_name, last_pos, vel):
        """按速度方向+距离给预设点打分（照 HKUST guess_pts.py predict_points）
        score = cos_factor*余弦相似度(速度方向,指向点方向) + (1-cos_factor)*exp(-距离*d_factor)
        """
        cos_factor = guess_cos_factor
        d_factor = guess_d_factor
        best_point, best_score = None, -1e9
        for point in guess_table.get(send_name, []):
            d_vec = (point[0] - last_pos[0], point[1] - last_pos[1])
            dot = vel[0] * d_vec[0] + vel[1] * d_vec[1]
            v_norm = np.sqrt(vel[0] ** 2 + vel[1] ** 2) + 1e-8
            d_norm = np.sqrt(d_vec[0] ** 2 + d_vec[1] ** 2) + 1e-8
            cos_sim = dot / (v_norm * d_norm)
            d_score = np.exp(-d_norm * d_factor)
            score = cos_factor * cos_sim + (1 - cos_factor) * d_score
            if score > best_score:
                best_score, best_point = score, point
        return best_point

    def send_point_guess(send_name, guess_time_limit):
        # 有速度信息：按速度方向打分选点（HKUST 方式）
        if guess_velocity_scoring and send_name in robot_position_history and len(robot_position_history[send_name]) >= 2:
            last_pos = robot_position_history[send_name][-1][1]
            vel = calculate_velocity(robot_position_history[send_name])
            if vel is not None:
                scored = score_guess_points(send_name, last_pos, vel)
                if scored is not None:
                    return scored[0], scored[1]
        # 无速度/关闭打分：原双点轮换兜底（进度反馈切换）
        # 进度未满 and 预测进度没有涨 and 超过单点预测时间上限，同时满足则切换另一个点预测
        if guess_value_now.get(send_name) < 120 and guess_value_now.get(send_name) - guess_value.get(
                send_name) <= 0 and time.time() - guess_time.get(send_name) >= guess_time_limit:
            guess_index[send_name] = 1 - guess_index[send_name]  # 每个ID不一样
            guess_time[send_name] = time.time()
        if guess_value_now.get(send_name) - guess_value.get(send_name) > 0:
            guess_time[send_name] = time.time()
        return guess_table.get(send_name)[guess_index.get(send_name)][0], \
            guess_table.get(send_name)[guess_index.get(send_name)][1]

    # 定义距离触发检查函数
    def check_distance_trigger():
        """【策略1】检查己方有输出的机器人与任意敌方机器人的距离是否满足触发条件"""
        # 己方有输出的机器人：英雄 + 步兵 + 哨兵
        our_output_robots = ['R1', 'R2', 'R3', 'R4', 'R5', 'R7'] if state == 'R' else ['B1', 'B2', 'B3', 'B4', 'B5', 'B7']
        # 敌方所有机器人
        enemy_robots = ['B1', 'B2', 'B3', 'B4', 'B5', 'B6', 'B7'] if state == 'R' else ['R1', 'R2', 'R3', 'R4', 'R5', 'R6', 'R7']
        
        # 遍历己方所有输出机器人
        for our_robot in our_output_robots:
            # 排除完全未检测到的机器人（send_map为(0,0)）
            if send_map[our_robot] == (0, 0):
                continue
            # 排除正在使用盲区预测点的机器人
            if guess_list.get(our_robot, True):
                continue
            
            # 遍历所有敌方机器人
            for enemy in enemy_robots:
                # 排除完全未检测到的敌方机器人
                if send_map[enemy] == (0, 0):
                    continue
                # 排除正在使用盲区预测点的敌方机器人
                if guess_list.get(enemy, True):
                    continue
                
                # 计算距离（坐标单位是cm，转换为米）
                dx = (send_map[our_robot][0] - send_map[enemy][0]) / 100.0
                dy = (send_map[our_robot][1] - send_map[enemy][1]) / 100.0
                distance = np.sqrt(dx**2 + dy**2)
                
                # 使用配置的距离阈值
                if distance < distance_threshold:
                    print(f"[距离触发] {our_robot} vs {enemy}, 距离={distance:.2f}米")
                    return True
        
        return False
    
    def calculate_velocity(position_history):
        """计算机器人的平均速度向量（单位：米/秒）
        
        Args:
            position_history: deque of (time, (x, y))
        
        Returns:
            (vx, vy) 或 None（如果数据不足）
        """
        if len(position_history) < 2:
            return None
        
        # 使用最近2帧计算瞬时速度
        t1, (x1, y1) = position_history[-2]
        t2, (x2, y2) = position_history[-1]
        
        dt = t2 - t1
        if dt < 0.01:  # 时间间隔太小，避免除零
            return None
        
        # 计算速度（坐标单位是cm，转换为m/s）
        vx = (x2 - x1) / 100.0 / dt
        vy = (y2 - y1) / 100.0 / dt
        
        return (vx, vy)
    
    def is_approaching(our_pos, enemy_pos, our_vel, enemy_vel):
        """判断两个机器人是否有相互接近的趋势
        
        Args:
            our_pos: (x, y) 己方位置（cm）
            enemy_pos: (x, y) 敌方位置（cm）
            our_vel: (vx, vy) 己方速度（m/s）
            enemy_vel: (vx, vy) 敌方速度（m/s）
        
        Returns:
            bool: 是否满足触发条件
        """
        # 计算当前距离（米）
        dx = (enemy_pos[0] - our_pos[0]) / 100.0
        dy = (enemy_pos[1] - our_pos[1]) / 100.0
        distance = np.sqrt(dx**2 + dy**2)
        
        # 使用配置的距离参数
        close_dist = motion_params['close_distance']
        far_dist = motion_params['far_distance']
        approach_spd = motion_params['approach_speed']
        active_spd = motion_params['active_speed']
        min_spd = motion_params['min_speed']
        
        # 如果距离已经很近，直接触发
        if distance < close_dist:
            return True
        
        # 如果距离太远，不触发
        if distance > far_dist:
            return False
        
        # 中距离情况，检查运动趋势
        # 计算位置向量（从己方指向敌方）
        if distance < 0.01:
            return False
        
        direction_x = dx / distance
        direction_y = dy / distance
        
        # 计算相对速度（敌方相对于己方的速度）
        rel_vx = enemy_vel[0] - our_vel[0]
        rel_vy = enemy_vel[1] - our_vel[1]
        rel_speed = np.sqrt(rel_vx**2 + rel_vy**2)
        
        if rel_speed < 0.1:  # 相对速度太小，认为没有明显运动趋势
            return False
        
        # 计算相对速度在位置向量上的投影（负值表示正在接近）
        approach_velocity = -(rel_vx * direction_x + rel_vy * direction_y)
        
        # 如果接近速度超过阈值，认为有交战趋势
        if approach_velocity > approach_spd:
            print(f"接近速度={approach_velocity:.2f}m/s, 距离={distance:.2f}m")
            return True
        
        # 也检查己方是否主动向敌方移动
        our_speed = np.sqrt(our_vel[0]**2 + our_vel[1]**2)
        if our_speed > min_spd:  # 己方在移动
            # 己方速度在敌方方向上的投影
            our_approach = our_vel[0] * direction_x + our_vel[1] * direction_y
            if our_approach > active_spd:  # 己方正在快速接近敌方
                print(f"己方接近速度={our_approach:.2f}m/s, 距离={distance:.2f}m")
                return True
        
        return False
    
    def check_motion_trend_trigger():
        """【策略2】检测双方机器人运动趋势，判断是否有交战趋势"""
        # 己方有输出的机器人：英雄 + 步兵 + 哨兵
        our_output_robots = ['R1', 'R2', 'R3', 'R4', 'R5', 'R7'] if state == 'R' else ['B1', 'B2', 'B3', 'B4', 'B5', 'B7']
        # 敌方所有机器人
        enemy_robots = ['B1', 'B2', 'B3', 'B4', 'B5', 'B6', 'B7'] if state == 'R' else ['R1', 'R2', 'R3', 'R4', 'R5', 'R6', 'R7']
        
        # 遍历己方所有输出机器人
        for our_robot in our_output_robots:
            # 需要至少2帧数据才能计算速度
            if len(robot_position_history[our_robot]) < 2:
                continue
            # 排除正在使用盲区预测点的机器人
            if guess_list.get(our_robot, True):
                continue
            
            # 计算己方机器人的速度向量
            our_vel = calculate_velocity(robot_position_history[our_robot])
            if our_vel is None:
                continue
            
            # 获取己方当前位置
            our_pos = robot_position_history[our_robot][-1][1]  # (time, (x, y))
            
            # 遍历所有敌方机器人
            for enemy in enemy_robots:
                # 需要至少2帧数据才能计算速度
                if len(robot_position_history[enemy]) < 2:
                    continue
                # 排除正在使用盲区预测点的敌方机器人
                if guess_list.get(enemy, True):
                    continue
                
                # 计算敌方机器人的速度向量
                enemy_vel = calculate_velocity(robot_position_history[enemy])
                if enemy_vel is None:
                    continue
                
                # 获取敌方当前位置
                enemy_pos = robot_position_history[enemy][-1][1]
                
                # 判断是否有接近趋势
                if is_approaching(our_pos, enemy_pos, our_vel, enemy_vel):
                    print(f"运动趋势满足条件: {our_robot} vs {enemy}")
                    return True
        
        return False
    
    time_s = time.time()
    target_last = 0  # 上一帧的飞镖目标
    update_time = 0  # 上次预测点更新时间
    send_count = 0  # 信道占用数，上限为4
    last_telemetry_log = 0.0
    send_map = {
        "R1": (0, 0),
        "R2": (0, 0),
        "R3": (0, 0),
        "R4": (0, 0),
        "R5": (0, 0),
        "R6": (0, 0),
        "R7": (0, 0),
        "B1": (0, 0),
        "B2": (0, 0),
        "B3": (0, 0),
        "B4": (0, 0),
        "B5": (0, 0),
        "B6": (0, 0),
        "B7": (0, 0)
    }
    # 开局发送初始密钥
    key_update_time = 0  # 密钥更新发送时间戳
    def double_vuln_startup_gate_ok():
        if double_vuln_startup_window_s <= 0:
            return True
        if time.time() - main_program_start_time >= double_vuln_startup_window_s:
            return True
        return double_vuln_startup_uses < double_vuln_startup_max_uses

    def note_double_vuln_startup_use():
        global double_vuln_startup_uses
        if double_vuln_startup_window_s > 0 and time.time() - main_program_start_time < double_vuln_startup_window_s:
            double_vuln_startup_uses += 1

    while not stop_requested:
        cycle_started_at = time.monotonic()

        if referee_mode == 'radio_ros' and referee_transport is not None:
            for ack in referee_transport.consume_successful_requests():
                chances_flag = max(chances_flag, int(ack.get('radar_cmd', chances_flag)))
                time_s = time.time()
                note_double_vuln_startup_use()
                print(
                    f"双倍易伤串口 ACK 成功: request_id={ack.get('request_id')} "
                    f"radar_cmd={chances_flag}"
                )
                report_runtime_status(last_double_vulnerability_ack=ack)

        guess_time_limit = config['blind_zone']['base_time'] + config['blind_zone'][
            'offset_time']  # 单位：秒，根据上一帧的信道占用数动态调整单点预测时间
        # print(guess_time_limit)
        send_count = 0  # 重置信道占用数

        # 0x0121 雷达自主决策：双倍易伤 + 密钥更新 整合发送
        # 每帧先分别判断各自触发条件，最后合并成一个数据包发送
        need_send_decision = False       # 本帧是否需要发送0x0121
        decision_triggers_vulnerability = False
        decision_radar_cmd = chances_flag  # radar_cmd: 默认为当前值（不触发双倍易伤时保持不变）
        decision_password_cmd = 0        # password_cmd: 0=无密钥操作, 1=更新己方密钥
        decision_password = [0] * 6      # 密钥值：默认全0，仅password_cmd=1时填充实际密钥

        # 开局发送初始密钥
        if referee_mode == 'legacy_serial' and not password_sent:
            if key_modifiable == 1 or key_modifiable == -1:
                print(f"开局发送初始密钥: {[chr(c) for c in current_password]}")
                decision_password_cmd = 1
                decision_password = current_password
                need_send_decision = True
                password_sent = True
                last_key_modifiable = key_modifiable  # 与本帧边沿逻辑对齐，避免同帧再发更新
        elif referee_mode == 'legacy_serial' and password_sent and key_modifiable == 1 and last_key_modifiable != 1:
            current_password = generate_random_password()
            print(f"bit5=1 更新密钥: {[chr(c) for c in current_password]}")
            decision_password_cmd = 1
            decision_password = current_password
            need_send_decision = True

        try:
            if state == 'R':
                opponent_prefix = 'B'
                ally_prefix = 'R'
            else:
                opponent_prefix = 'R'
                ally_prefix = 'B'
            if algorithm_mode == 'hkust_tracker':
                with tracked_targets_lock:
                    target_snapshot = dict(tracked_targets)
                valid_send_names = {
                    name for name, target_output in target_snapshot.items()
                    if target_output.valid
                }
                occlusion_names = {
                    name for name, target_output in target_snapshot.items()
                    if target_output.state == TargetState.OCCLUSION_HOLD
                }
                for robot_name in send_map:
                    target_output = target_snapshot.get(robot_name)
                    if target_output is None or not target_output.valid:
                        send_map[robot_name] = (0, 0)
                        guess_list[robot_name] = False
                        continue
                    send_map[robot_name] = (
                        int(target_output.position_cm[0]),
                        int(target_output.position_cm[1]),
                    )
                    guess_list[robot_name] = (
                        target_output.state == TargetState.BLIND_PREDICTION
                    )
            else:
                all_filter_data = apply_occlusion_hold(filter.get_all_data())
                valid_send_names, occlusion_names = get_referee_target_states()

                # 喂视觉坐标给融合缓冲（转裁判坐标：内部(短边,长边) → 裁判(长边,短边)）
                if fusion_buffer is not None:
                    vision_map = {}
                    for robot_name in send_map:
                        ground = all_filter_data.get(robot_name)
                        blind = None
                        if guess_list.get(robot_name):
                            blind = send_point_guess(robot_name, guess_time_limit)
                        # 内部坐标 → 裁判坐标（x:0-2800长边, y:0-1500短边）
                        def _to_referee(xy):
                            if xy is None:
                                return None
                            if state == 'R':
                                return (2800 - xy[1], xy[0])
                            return (xy[1], 1500 - xy[0])
                        vision_map[robot_name] = (_to_referee(ground), _to_referee(blind))
                    feed_vision_positions(vision_map)

                def update_send_map_entry(robot_name, allow_guess=False):
                    # 融合仲裁优先（信息波 UWB > 视觉 > 旧信息波 > 盲区）
                    if fusion_buffer is not None:
                        resolved = fusion_buffer.resolve([robot_name]).get(robot_name)
                        if resolved is not None and resolved.coordinate is not None:
                            send_map[robot_name] = (resolved.coordinate.x, resolved.coordinate.y)
                            return

                    if referee_send_correct_only and robot_name not in valid_send_names:
                        send_map[robot_name] = (0, 0)
                        return

                    # 仅敌方哨兵保留盲区预测，其他机器人无识别时统一置0
                    if allow_guess and guess_list.get(robot_name):
                        send_map[robot_name] = send_point_guess(robot_name, guess_time_limit)
                        return

                    if guess_list.get(robot_name):
                        send_map[robot_name] = (0, 0)
                        return

                    if all_filter_data.get(robot_name, False):
                        if robot_name[0] == 'B':
                            send_map[robot_name] = send_point_B(robot_name, all_filter_data)
                        else:
                            send_map[robot_name] = send_point_R(robot_name, all_filter_data)
                    else:
                        send_map[robot_name] = (0, 0)

                for role in ['1', '2', '3', '4', '5', '6', '7']:
                    enemy_name = f"{opponent_prefix}{role}"
                    ally_name = f"{ally_prefix}{role}"
                    update_send_map_entry(
                        enemy_name,
                        allow_guess=(
                            blind_zone_enabled
                            and role in blind_zone_roles
                            and enemy_name in guess_table
                        )
                    )
                    update_send_map_entry(ally_name, allow_guess=False)

            if referee_mode == 'radio_ros':
                if referee_transport is not None and referee_transport.snapshot().get('connected'):
                    telemetry = build_vision_telemetry(
                        side=state,
                        send_map=send_map,
                        valid_names=set(valid_send_names),
                        guess_list=guess_list,
                        occlusion_names=occlusion_names,
                        camera_ready=globals().get('camera_image') is not None,
                        fps=latest_vision_fps,
                        camera_fps=camera_frame_rate.snapshot(),
                        inference_ms=latest_inference_ms,
                        filter_type=(algorithm_mode if algorithm_mode == 'hkust_tracker'
                                     else config['filter']['type']),
                    )
                    referee_transport.publish_telemetry(telemetry)
            else:
                ser_data = build_data_radar_all(send_map, state)
                packet, seq = build_send_packet(ser_data, seq, [0x03, 0x05])
                ser1.write(packet)
            if time.time() - last_telemetry_log >= 1.0:
                print(f"坐标遥测 {telemetry_rate_hz:g}Hz" if referee_mode == 'radio_ros' else send_map, seq)
                last_telemetry_log = time.time()
            # 超过单点预测时间上限，更新上次预测的进度
            if time.time() - update_time > guess_time_limit:
                update_time = time.time()
                if state == 'R':
                    guess_value['B1'] = guess_value_now.get('B1')
                    guess_value['B2'] = guess_value_now.get('B2')
                    guess_value['B7'] = guess_value_now.get('B7')
                else:
                    guess_value['R1'] = guess_value_now.get('R1')
                    guess_value['R2'] = guess_value_now.get('R2')
                    guess_value['R7'] = guess_value_now.get('R7')

            # 双倍易伤判断：飞镖目标切换触发 
            if double_vulnerability_enabled and target != target_last and target != 0:
                target_last = target
                # 有双倍易伤机会，并且当前没有在双倍易伤
                if double_vulnerability_chance > 0 and opponent_double_vulnerability == 0:
                    time_e = time.time()
                    # 发送时间间隔为10秒；启动窗口内双倍次数上限
                    if time_e - time_s > 10 and double_vuln_startup_gate_ok():
                        if referee_mode == 'radio_ros':
                            request_id = referee_transport.request_double_vulnerability() if referee_transport else None
                            if request_id:
                                print(f"飞镖触发：请求双倍易伤 request_id={request_id}，等待串口 ACK")
                                report_runtime_status(last_double_vulnerability_request={
                                    'request_id': request_id, 'reason': 'dart', 'requested_at': time.time()
                                })
                        elif chances_flag < 0xFF:
                            print("飞镖触发：请求双倍易伤")
                            decision_radar_cmd = chances_flag + 1
                            decision_triggers_vulnerability = True
                            need_send_decision = True
            
            # 双倍易伤判断：自动触发逻辑
            trigger_now = False
            trigger_method = ""
            
            # 策略1：基于距离的触发
            if double_vulnerability_enabled and trigger_mode in ['distance', 'both']:
                distance_trigger_now = check_distance_trigger()
                now_stable = time.time()
                if distance_trigger_now:
                    if distance_condition_true_since is None:
                        distance_condition_true_since = now_stable
                    elif now_stable - distance_condition_true_since >= double_vuln_stable_duration_s:
                        trigger_now = True
                        trigger_method = "距离触发"
                        distance_condition_true_since = None
                else:
                    distance_condition_true_since = None
            
            # 更新机器人位置历史（盲区预测打分用；独立于双倍易伤开关）
            current_time = time.time()
            for robot_id in send_map:
                pos = send_map[robot_id]
                # 只记录真实检测到的位置
                if pos != (0, 0) and not guess_list.get(robot_id, True):
                    robot_position_history[robot_id].append((current_time, pos))

            # 策略2：基于运动趋势的触发
            if double_vulnerability_enabled and trigger_mode in ['motion', 'both']:
                motion_trigger_now = check_motion_trend_trigger()
                now_stable = time.time()
                if motion_trigger_now:
                    if motion_condition_true_since is None:
                        motion_condition_true_since = now_stable
                    elif now_stable - motion_condition_true_since >= double_vuln_stable_duration_s:
                        trigger_now = True
                        trigger_method = "运动趋势触发" if trigger_method == "" else "距离+运动趋势触发"
                        motion_condition_true_since = None
                else:
                    motion_condition_true_since = None
            
            # 执行双倍易伤自动触发
            if trigger_now:
                # 有双倍易伤机会，并且当前没有在双倍易伤
                if double_vulnerability_chance > 0 and opponent_double_vulnerability == 0:
                    time_e = time.time()
                    # 共享10秒防抖
                    if time_e - time_s > 10:
                        if decision_radar_cmd == chances_flag and double_vuln_startup_gate_ok():
                            if referee_mode == 'radio_ros':
                                request_id = referee_transport.request_double_vulnerability() if referee_transport else None
                                if request_id:
                                    print(f"[{trigger_method}] 请求双倍易伤 request_id={request_id}，等待串口 ACK")
                                    report_runtime_status(last_double_vulnerability_request={
                                        'request_id': request_id,
                                        'reason': trigger_method,
                                        'requested_at': time.time(),
                                    })
                            elif chances_flag < 0xFF:
                                print(f"[{trigger_method}] 请求双倍易伤")
                                decision_radar_cmd = chances_flag + 1
                                decision_triggers_vulnerability = True
                                need_send_decision = True

            if referee_mode == 'legacy_serial' and need_send_decision:
                data = build_data_decision(decision_radar_cmd, state,
                                           password_cmd=decision_password_cmd,
                                           password=decision_password)
                packet, seq = build_send_packet(data, seq, [0x03, 0x01])
                ser1.write(packet)
                if decision_radar_cmd > chances_flag:
                    chances_flag = decision_radar_cmd
                if decision_triggers_vulnerability:
                    note_double_vuln_startup_use()
                    time_s = time.time()
                parts = []
                if decision_radar_cmd > 0:
                    parts.append(f"双倍易伤(radar_cmd={decision_radar_cmd})")
                if decision_password_cmd == 1:
                    parts.append(f"密钥更新({[chr(c) for c in decision_password]})")
                print(f"[0x0121] 已发送: {' + '.join(parts)}")
        except Exception as r:
            print('未知错误 %s' % (r))

        last_key_modifiable = key_modifiable
        cycle_period = 1.0 / telemetry_rate_hz if referee_mode == 'radio_ros' else 0.2
        time.sleep(max(0.0, cycle_period - (time.monotonic() - cycle_started_at)))

# 裁判系统串口接收线程
def ser_receive():
    if not ser1:  # 检查串口是否可用
        print("串口未启用，接收线程退出")
        return
    global vulnerability  # 标记进度列表
    global double_vulnerability_chance  # 拥有双倍易伤次数
    global opponent_double_vulnerability  # 双倍易伤触发状态
    global encryption_level  # 己方加密等级
    global key_modifiable  # 密钥是否可修改
    global target  # 飞镖当前目标
    global mark_progress_bit_cache  # 0x020C 
    progress_cmd_id = [0x02, 0x0C]  # 雷达标记进度的命令码0x020C
    vulnerability_cmd_id = [0x02, 0x0E]  # 双倍易伤次数和触发状态
    target_cmd_id = [0x01, 0x05]  # 飞镖目标
    buffer = b''  # 初始化缓冲区
    while True:
        # 从串口读取数据
        received_data = ser1.read_all()  # 读取一秒内收到的所有串口数据
        # 将读取到的数据添加到缓冲区中
        buffer += received_data

        # 查找帧头（SOF）的位置
        sof_index = buffer.find(b'\xA5')

        while sof_index != -1:
            # 如果找到帧头，尝试解析数据包
            if len(buffer) >= sof_index + 5:  # 至少需要5字节才能解析帧头
                # 从帧头开始解析数据包
                packet_data = buffer[sof_index:]

                # 查找下一个帧头的位置
                next_sof_index = packet_data.find(b'\xA5', 1)

                if next_sof_index != -1:
                    # 如果找到下一个帧头，说明当前帧头到下一个帧头之间是一个完整的数据包
                    packet_data = packet_data[:next_sof_index]
                    # print(packet_data)
                else:
                    # 如果没找到下一个帧头，说明当前帧头到末尾不是一个完整的数据包
                    break

                # 解析数据包
                progress_result = receive_packet(packet_data, progress_cmd_id,
                                                 info=False)  # 解析单个数据包，cmd_id为0x020C
                vulnerability_result = receive_packet(packet_data, vulnerability_cmd_id, info=False)
                target_result = receive_packet(packet_data, target_cmd_id, info=False)
                received_commands = []
                # 更新裁判系统数据，标记进度、易伤、飞镖目标
                if progress_result is not None:
                    received_commands.append("0x020C")
                    received_cmd_id1, received_data1, received_seq1 = progress_result
                    mark_progress = struct.unpack('H', received_data1[:2])[0]
                    vulnerability = [((mark_progress >> i) & 0x01) * 120 for i in range(6)]
                    for bit_idx in range(6):
                        mark_progress_bit_cache[bit_idx] = vulnerability[bit_idx]

                    if state == 'R':
                        guess_value_now['B1'] = vulnerability[0]
                        guess_value_now['B2'] = vulnerability[1]
                        guess_value_now['B7'] = vulnerability[5]
                    else:
                        guess_value_now['R1'] = vulnerability[0]
                        guess_value_now['R2'] = vulnerability[1]
                        guess_value_now['R7'] = vulnerability[5]
                if vulnerability_result is not None:
                    received_commands.append("0x020E")
                    received_cmd_id2, received_data2, received_seq2 = vulnerability_result
                    received_data2 = list(received_data2)[0]
                    double_vulnerability_chance, opponent_double_vulnerability, encryption_level, key_modifiable = Radar_decision(received_data2)
                if target_result is not None:
                    received_commands.append("0x0105")
                    received_cmd_id3, received_data3, received_seq3 = target_result
                    target = (list(received_data3)[1] & 0b1100000) >> 5
                if received_commands:
                    report_runtime_status(
                        last_referee_packet_at=time.time(),
                        last_referee_command="/".join(received_commands),
                    )

                # 从缓冲区中移除已解析的数据包
                buffer = buffer[sof_index + len(packet_data):]

                # 继续寻找下一个帧头的位置
                sof_index = buffer.find(b'\xA5')

            else:
                # 缓冲区中的数据不足以解析帧头，继续读取串口数据
                break
        time.sleep(0.5)


def handle_radio_referee_message(message_type, payload):
    """Apply radio-owned referee parsing to the existing visual strategy state."""
    global vulnerability
    global double_vulnerability_chance
    global opponent_double_vulnerability
    global encryption_level
    global key_modifiable
    global target
    if message_type == 'RadarMarkProgress':
        enemy = payload.get('enemy') or {}
        names = (
            'opponent_hero', 'opponent_engineer', 'opponent_infantry_3',
            'opponent_infantry_4', 'opponent_aerial', 'opponent_sentry',
        )
        vulnerability = [120 if bool(enemy.get(name)) else 0 for name in names]
        for bit_idx, value in enumerate(vulnerability):
            mark_progress_bit_cache[bit_idx] = value
        opponent_prefix = 'B' if state == 'R' else 'R'
        guess_value_now[f'{opponent_prefix}1'] = vulnerability[0]
        guess_value_now[f'{opponent_prefix}2'] = vulnerability[1]
        guess_value_now[f'{opponent_prefix}7'] = vulnerability[5]
    elif message_type == 'RadarDecisionSync':
        double_vulnerability_chance = int(payload.get('double_vulnerability_count', -1))
        opponent_double_vulnerability = int(bool(payload.get('is_double_vulnerability', False)))
        encryption_level = int(payload.get('own_encryption_level', 0))
        key_modifiable = int(bool(payload.get('can_change_password', False)))
    elif message_type == 'DartStatus':
        target = int(payload.get('selected_target', 0))
    report_runtime_status(
        last_referee_packet_at=time.time(),
        last_referee_command=message_type or 'unknown',
        referee_transport='radio_ros',
    )


if referee_mode == 'radio_ros':
    referee_transport = RefereeTransport.create(
        referee_mode, state, on_referee_message=handle_radio_referee_message
    )
    try:
        referee_transport.start()
        print('radio_ros 本地桥接已启动，等待 transistor-radar-radio 状态心跳；裁判串口由无线电侧独占')
        report_runtime_status(radio_ros_connected=False, radio_ros_error=None)
    except Exception as error:
        print(f'radio_ros 裁判通信降级，视觉推理继续运行: {error}')
        report_runtime_status(radio_ros_connected=False, radio_ros_error=str(error))


# 创建机器人坐标滤波器
filter = create_filter(config)
print(f"已启用滤波器类型: {config['filter']['type']}")

compare_filter = None
if compare_enabled:
    main_type = config['filter']['type']
    if compare_type == 'auto':
        compare_type = 'hybrid_gated_kalman' if main_type == 'sliding_window' else 'sliding_window'
    if compare_type == main_type:
        print(f"对比滤波器与主滤波器相同({main_type})，已自动关闭对比")
    else:
        try:
            compare_filter = create_filter_by_type(config, compare_type, update_guess=False)
            print(f"已启用影子对比滤波器: {compare_type}")
        except Exception as e:
            compare_filter = None
            print(f"影子对比滤波器创建失败: {e}")

# 加载模型，实例化机器人检测器和装甲板检测器 yolov5
weights_path = config['paths']['models']['car']
weights_path_next = config['paths']['models']['armor']
detector = YOLOv5Detector(weights_path, img_size=car_img_size, data='config/car.yaml',
                          conf_thres=vehicle_low_confidence,
                          iou_thres=0.5, max_det=14, ui=True)
detector_next = None
armor_batch_active = False
armor_batch_path = config.get('paths', {}).get('models', {}).get('armor_batch')
if algorithm_mode == 'hkust_tracker' and armor_batch_path and os.path.isfile(armor_batch_path):
    try:
        detector_next = YOLOv5Detector(
            armor_batch_path,
            data='config/armor.yaml',
            conf_thres=armor_conf_threshold,
            iou_thres=0.2,
            img_size=(320, 320),
            max_det=8,
            ui=False,
        )
        if not detector_next.supports_dynamic_batch:
            raise RuntimeError('engine 没有动态 batch profile')
        armor_batch_active = True
        print(f"动态装甲板 batch 引擎已加载: {armor_batch_path} (1..16, 320x320)")
    except Exception as error:
        detector_next = None
        print(f"动态装甲板 batch 引擎加载失败，使用静态 4x4 拼图: {error}")
if detector_next is None:
    detector_next = YOLOv5Detector(weights_path_next, data='config/armor.yaml',
                                   conf_thres=armor_conf_threshold, iou_thres=0.2,
                                   img_size=armor_img_size,
                                   max_det=16 if algorithm_mode == 'hkust_tracker' else 10,
                                   ui=algorithm_mode == 'legacy')
print(f"推理尺寸: 车辆={car_img_size[1]}x{car_img_size[0]}, "
      f"装甲板={'320x320 dynamic batch' if armor_batch_active else f'{armor_img_size[1]}x{armor_img_size[0]} static tile'}")
print(
    f"车辆检测阈值: 新建={vehicle_high_confidence:.2f}, "
    f"遮挡找回={vehicle_low_confidence:.2f}"
)
print(f"装甲板检测置信度阈值: {armor_conf_threshold:.2f}")
print(f"算法模式: {algorithm_mode}")
if algorithm_mode == 'legacy':
    print(f"多车同时识别: {'开启' if multi_car_recognition else '关闭（仅处理置信度最高的单车 ROI）'}")

tracked_pipeline = None
if algorithm_mode == 'hkust_tracker':
    tracked_pipeline = TrackedRadarPipeline(
        config=config,
        car_detector=detector,
        armor_detector=detector_next,
        project_point=lambda point: (lambda r: r if r is not None else (0.0, 0.0))(
            project_image_point_to_map(point[0], point[1])),
        convert_map_point=convert_projected_map_point,
        side=state,
    )
report_runtime_status(phase="models_ready")

# 仅 legacy_serial 自己读取裁判串口；radio_ros 的裁判状态由回调更新。
if referee_mode == 'legacy_serial' and ser1 is not None:
    thread_receive = threading.Thread(target=ser_receive, daemon=True)
    thread_receive.start()
else:
    print("跳过视觉项目串口接收线程")

# 两种模式都运行坐标/策略线程；radio_ros 只发布结构化遥测与语义请求。
if referee_mode == 'radio_ros' or ser1 is not None:
    thread_list = threading.Thread(target=ser_send, daemon=True)
    thread_list.start()
else:
    print("裁判通信不可用；视觉推理继续运行")

camera_image = None
camera_error = None
cap = None


def run_camera_worker(target, *args):
    try:
        target(*args)
    except (Exception, SystemExit) as error:
        detail = str(error) or error.__class__.__name__
        set_camera_error(f"图像输入启动失败: {detail}")

if camera_mode == 'test':
    # 优先使用视频测试配置，其次回落到静态图片
    test_cfg = config.get('global', {}).get('test', {}) if isinstance(config.get('global', {}).get('test', {}), dict) else {}
    video_path = test_cfg.get('video_path') or config.get('paths', {}).get('test_video')
    if video_path and isinstance(video_path, str) and len(video_path) > 0:
        if not os.path.exists(video_path):
            print(f"测试视频路径不存在：{video_path}，回退到静态测试图片")
            camera_image = cv2.imread(config['paths']['test_img'])
        else:
            loop = bool(test_cfg.get('loop', True))
            start_frame = int(test_cfg.get('start_frame', 0) or 0)
            playback_speed = float(test_cfg.get('playback_speed', 1.0) or 1.0)
            print(f"使用测试视频：{video_path}，loop={loop}, start_frame={start_frame}, speed={playback_speed}")
            thread_camera = threading.Thread(
                target=run_camera_worker,
                args=(test_video_capture_get, video_path, loop, start_frame, playback_speed),
                daemon=True,
            )
            thread_camera.start()
    else:
        camera_image = cv2.imread(config['paths']['test_img'])
elif camera_mode == 'hik':
    # 海康相机图像获取线程
    thread_camera = threading.Thread(target=run_camera_worker, args=(hik_camera_get,), daemon=True)
    thread_camera.start()
elif camera_mode == 'video':
    # USB相机图像获取线程
    thread_camera = threading.Thread(target=run_camera_worker, args=(video_capture_get,), daemon=True)
    thread_camera.start()

initial_camera_frame = wait_for_initial_camera_frame(
    get_frame=lambda: camera_image,
    get_error=lambda: camera_error,
    should_stop=lambda: stop_requested,
    report_status=report_runtime_status,
)

if initial_camera_frame is None:
    report_runtime_status(
        phase="stopped",
        stopped_at=time.time(),
        camera_ready=False,
        recording=False,
    )
    sys.exit(0)

# 获取相机图像的画幅，限制点不超限
img0 = initial_camera_frame.copy()
img_y = img0.shape[0]
img_x = img0.shape[1]
print(img0.shape)
report_runtime_status(
    phase="running",
    camera_ready=True,
    camera_error=None,
    camera_fps=camera_frame_rate.snapshot(),
    heartbeat_at=time.time(),
    recording_suspended_reason=None,
)

# 勾选录像后才检测存储。检测只读取已挂载块设备信息，不枚举 USB/视觉设备。
video_recorder = None
recording_paths_announced = False
recording_error_announced = False
recording_storage = None
recording_directory = None
video_recorder_closed = False
if record_video:
    try:
        recording_directory, recording_storage = prepare_match_recording_directory(
            recording_default_root,
            prefer_external=recording_prefer_external,
            external_directory=recording_external_directory,
            selected_root=recording_selected_root,
            session_name=f"{match_run_id}_vision",
        )
        video_recorder = AsyncMatchVideoRecorder(
            recording_directory,
            fps=recording_fps,
            codec=recording_codec,
            max_catchup_seconds=recording_max_catchup_seconds,
            queue_size=recording_queue_size,
            map_max_width=recording_map_max_width,
            camera_max_width=recording_camera_max_width,
        )
        video_recorder.prepare(map, img0)
        if recording_storage.is_external:
            print(
                f"检测到外接硬盘 {recording_storage.mount_point} "
                f"({recording_storage.source})，录像优先写入外接硬盘"
            )
            storage_kind = "external"
        else:
            selected_root_used = (
                recording_selected_root is not None
                and recording_storage.root == recording_selected_root.resolve()
            )
            if selected_root_used:
                print(f"录像写入操作者选择的目录: {recording_storage.root}")
            elif recording_prefer_external:
                print("未检测到已挂载且可写的外接硬盘，录像写入默认目录")
            else:
                print("配置已关闭外接硬盘优先，录像写入默认目录")
            storage_kind = "default"
        print(
            f"比赛录像目录: {recording_directory}，目标={recording_fps:g} FPS，"
            f"编码器={recording_codec}，后台队列={recording_queue_size}"
        )
        recording_manifest = {
            "schema": "transistor.radar.vision_recording.v1",
            "match_run_id": match_run_id,
            "component_session_id": recording_directory.name,
            "started_at": time.time(),
            "recording_directory": str(recording_directory),
            "storage": storage_kind,
            "storage_root": str(recording_storage.root),
            "storage_mount_point": (
                str(recording_storage.mount_point)
                if recording_storage.mount_point is not None
                else None
            ),
            "requested_codec": recording_codec,
            "requested_fps": recording_fps,
        }
        manifest_path = recording_directory / "recording_manifest.json"
        temporary_manifest_path = recording_directory / ".recording_manifest.json.tmp"
        with temporary_manifest_path.open("w", encoding="utf-8") as manifest_file:
            json.dump(recording_manifest, manifest_file, ensure_ascii=False, indent=2)
            manifest_file.write("\n")
            manifest_file.flush()
            os.fsync(manifest_file.fileno())
        os.replace(temporary_manifest_path, manifest_path)
        report_runtime_status(
            recording_requested=True,
            recording=False,
            match_run_id=match_run_id,
            component_session_id=recording_directory.name,
            recording_storage=storage_kind,
            recording_directory=str(recording_directory),
        )
    except Exception as error:
        print(f"比赛录像初始化失败，主程序继续运行: {error}")
        record_video = False
        report_runtime_status(recording=False, recording_error=str(error))


def close_video_recorder():
    global video_recorder_closed
    if video_recorder_closed:
        return
    video_recorder_closed = True
    if video_recorder is not None:
        finalized = video_recorder.release(timeout=15.0)
        status = video_recorder.snapshot()
        if not finalized:
            print("录像后台线程 15 秒内未结束；主程序继续退出")
        print(
            f"录像已停止：处理画面 {status['input_frames']} 帧，"
            f"按 {recording_fps:g} FPS 采样 {status['submitted_frames']} 帧，"
            f"视频写入 {status['frames_written']} 帧，"
            f"编码队列拥塞丢弃 {status['dropped_frames']} 帧，"
            f"平均双路写入 {status['average_write_ms']:.1f} ms/组"
        )


atexit.register(close_video_recorder)


def queue_recording_frame(map_frame, camera_frame):
    """Submit one frame pair without performing codec or disk I/O here."""
    global recording_paths_announced, recording_error_announced, record_video
    if not record_video or video_recorder is None:
        return

    accepted = video_recorder.submit(map_frame, camera_frame)
    status = video_recorder.snapshot()
    if status["error"]:
        if not recording_error_announced:
            print(f"比赛录像后台写入失败，主程序继续运行: {status['error']}")
            recording_error_announced = True
        record_video = False
        report_runtime_status(recording=False, recording_error=status["error"])
        return
    if not accepted:
        return
    if not recording_paths_announced and status["map_path"] and status["camera_path"]:
        print(f"地图录像: {status['map_path']}")
        print(f"相机录像: {status['camera_path']}")
        print(
            f"录像分辨率: 地图={status['map_size'][0]}x{status['map_size'][1]}，"
            f"相机={status['camera_size'][0]}x{status['camera_size'][1]}"
        )
        print(
            f"录像实际编码器: {status['selected_codec']} "
            f"({status['selected_backend']})"
        )
        report_runtime_status(
            recording=True,
            recording_fps=recording_fps,
            recording_codec=status["selected_codec"],
            map_video=str(status["map_path"]),
            camera_video=str(status["camera_path"]),
            map_video_size=list(status["map_size"]),
            camera_video_size=list(status["camera_size"]),
        )
        recording_paths_announced = True


def present_tracked_frame(map_frame, camera_frame, information_frame, started_at, diagnostics):
    """Present and record one frame from the track-centric pipeline."""
    global latest_vision_fps, latest_inference_ms, last_runtime_status_update
    global stop_requested

    finished_at = time.time()
    elapsed = finished_at - started_at
    latest_vision_fps = 1 / elapsed if elapsed > 0 else 0.0
    latest_inference_ms = max(elapsed, 0.0) * 1000.0
    if finished_at - last_runtime_status_update >= 1.0:
        measured_camera_fps = camera_frame_rate.snapshot()
        camera_fps_text = "--" if measured_camera_fps is None else f"{measured_camera_fps:.1f}"
        print(
            f"camera_fps: {camera_fps_text} | processing_fps: {latest_vision_fps:.1f} "
            f"| tracks={diagnostics.get('active_tracks', 0)} "
            f"armor={diagnostics.get('armor_detections', 0)}"
        )
        transport_status = referee_transport.snapshot() if referee_transport is not None else {
            'mode': referee_mode, 'connected': ser1 is not None
        }
        report_runtime_status(
            heartbeat_at=finished_at,
            fps=round(latest_vision_fps, 1),
            processing_fps=round(latest_vision_fps, 1),
            camera_fps=(
                None
                if measured_camera_fps is None
                else round(measured_camera_fps, 1)
            ),
            inference_ms=round(latest_inference_ms, 1),
            algorithm_mode=algorithm_mode,
            active_tracks=int(diagnostics.get('active_tracks', 0)),
            armor_batch=bool(diagnostics.get('dynamic_armor_batch', False)),
            referee_transport=referee_mode,
            referee_communication=transport_status,
            radio_ros_connected=bool(transport_status.get('radio_online', False)),
        )
        last_runtime_status_update = finished_at

    _ = draw_information_ui(vulnerability, state, information_frame)
    cv2.putText(information_frame, "algorithm: hkust_tracker", (10, 330),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
    cv2.putText(information_frame, "vulnerability_chances: " + str(double_vulnerability_chance),
                (10, 370), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(information_frame, "vulnerability_Triggering: " + str(opponent_double_vulnerability),
                (10, 410), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.imshow('information_ui', information_frame)
    cv2.imshow('map_main', cv2.resize(map_frame, tuple(config['ui']['map_display_size'])))
    cv2.imshow('img', cv2.resize(camera_frame, tuple(config['ui']['img_display_size'])))

    queue_recording_frame(map_frame, camera_frame)

    key = cv2.waitKey(1) & 0xFF
    if key in (27, ord('q')):
        stop_requested = True


last_runtime_status_update = 0.0
while not stop_requested:
    frame_index += 1
    # 刷新裁判系统信息UI图像
    information_ui_show = information_ui.copy()
    map = map_backup.copy()
    img0 = camera_image.copy()
    ts = time.time()
    if tracked_pipeline is not None:
        tracked_pipeline.set_blind_progress(guess_value_now)
        tracked_result = tracked_pipeline.process(img0, ts)
        img0 = tracked_result.annotated_image
        with tracked_targets_lock:
            tracked_targets = dict(tracked_result.targets)
        valid_names = {
            name for name, target_output in tracked_result.targets.items()
            if target_output.valid
        }
        tracked_occlusion_names = {
            name for name, target_output in tracked_result.targets.items()
            if target_output.state == TargetState.OCCLUSION_HOLD
        }
        publish_referee_target_states(valid_names, tracked_occlusion_names)
        for name, target_output in tracked_result.targets.items():
            if not target_output.valid or target_output.display_xy is None:
                continue
            color_m = (0, 0, 255) if name.startswith('R') else (255, 0, 0)
            point = (int(target_output.display_xy[0]), int(target_output.display_xy[1]))
            thickness = -1 if name[0] != state else 3
            if target_output.state == TargetState.BLIND_PREDICTION:
                color_m = (0, 215, 255)
            elif target_output.state == TargetState.OCCLUSION_HOLD:
                color_m = (255, 255, 0)
            cv2.circle(map, point, 15, color_m, thickness)
            cv2.putText(map, name, (point[0] - 5, point[1] + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 5)
            position = target_output.position_cm
            cv2.putText(
                map,
                f"({int(position[0])},{int(position[1])}) {target_output.state.value}",
                (point[0] - 100, point[1] + 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (255, 255, 255),
                3,
            )
        present_tracked_frame(
            map, img0, information_ui_show, ts, tracked_result.diagnostics
        )
        continue
    # 第一层神经网络识别
    result0 = detector.predict(img0)
    roi_list = []
    roi_pos = []
    car_entries = []
    confirmed_armor_candidates = []
    for detection in result0:
        cls, xywh, conf = detection
        if cls == 'car':
            left, top, w, h = xywh
            left, top, w, h = int(left), int(top), int(w), int(h)
            cropped = camera_image[top:top + h, left:left + w]
            cropped_img = np.ascontiguousarray(cropped)
            car_entries.append((float(conf), cropped_img, (left, top, w, h)))
    if not multi_car_recognition and len(car_entries) > 1:
        car_entries.sort(key=lambda x: x[0], reverse=True)
        car_entries = car_entries[:1]
    for _, cropped_img, pos in car_entries:
        roi_list.append(cropped_img)
        roi_pos.append(pos)

    # 拼接ROI并批量识别
    def pack_rois_to_canvas(roi_list, roi_pos, canvas_size=armor_canvas_size, padding=10):
        """
        将roi_list拼接到若干张canvas_size的画布上，返回画布列表和每个roi在画布中的位置
        每个ROI四周加padding，避免目标贴边
        """
        canvases = []
        canvas_rois = []  # 每个画布上的roi信息: [(roi_idx, x_with_pad, y_with_pad, w, h, x_raw, y_raw)]
        cur_canvas = np.zeros((canvas_size[1], canvas_size[0], 3), dtype=np.uint8)
        cur_rois = []
        x, y, row_h = 0, 0, 0
        for idx, roi in enumerate(roi_list):
            h, w = roi.shape[:2]
            ph, pw = h + 2 * padding, w + 2 * padding
            if pw > canvas_size[0] or ph > canvas_size[1]:
                continue  # 跳过过大的roi
            if x + pw > canvas_size[0]:
                x = 0
                y += row_h
                row_h = 0
            if y + ph > canvas_size[1]:
                canvases.append(cur_canvas)
                canvas_rois.append(cur_rois)
                cur_canvas = np.zeros((canvas_size[1], canvas_size[0], 3), dtype=np.uint8)
                cur_rois = []
                x, y, row_h = 0, 0, 0
            # 先填充黑色，再放ROI
            cur_canvas[y:y + ph, x:x + pw] = 0
            cur_canvas[y + padding:y + padding + h, x + padding:x + padding + w] = roi
            cur_rois.append((idx, x + padding, y + padding, w, h, x, y))  # 记录带padding和原始canvas坐标
            x += pw
            row_h = max(row_h, ph)
        if cur_rois:
            canvases.append(cur_canvas)
            canvas_rois.append(cur_rois)
        return canvases, canvas_rois


    if roi_list:
        canvases, canvas_rois = pack_rois_to_canvas(roi_list, roi_pos, padding=10)
        for i, (canvas, rois) in enumerate(zip(canvases, canvas_rois)):
            result_n = detector_next.predict(canvas)
            if show_armor_canvas:
                canvas_show = canvas.copy()
                for _, rx, ry, rw, rh, _, _ in rois:
                    cv2.rectangle(canvas_show, (rx, ry), (rx + rw, ry + rh), (0, 0, 255), 2)
                cv2.imshow(f'canvas_result_{i}', canvas_show)

            # 复原检测框并画到img0上（每个ROI只保留一个装甲板）
            for idx, rx, ry, rw, rh, ox, oy in rois:
                best_det = None
                best_conf = -1
                best_center_dist2 = float('inf')
                best_xywh = None
                best_cls = None
                roi_center_x = rx + rw * 0.5
                roi_center_y = ry + rh * 0.5
                # 遍历所有检测框，找属于该ROI的
                for detection1 in result_n:
                    cls, xywh, conf = detection1
                    x, y, w, h = xywh
                    cx, cy = x + w * 0.5, y + h * 0.5
                    if rx <= cx < rx + rw and ry <= cy < ry + rh:
                        center_dist2 = (cx - roi_center_x) ** 2 + (cy - roi_center_y) ** 2
                        if armor_selection_mode == 'center':
                            is_better = (
                                center_dist2 < best_center_dist2
                                or (center_dist2 == best_center_dist2 and conf > best_conf)
                            )
                        else:
                            is_better = conf > best_conf
                        if is_better:
                            best_conf = conf
                            best_center_dist2 = center_dist2
                            best_det = detection1
                            best_xywh = xywh
                            best_cls = cls
                # 只处理选中的装甲板
                if best_det is not None:
                    x, y, w, h = best_xywh
                    left, top, car_w, car_h = roi_pos[idx]
                    x0 = x - rx + left
                    y0 = y - ry + top
                    color_held = False
                    color_committed = True
                    color_debug_text = None
                    if vehicle_color_hold_enabled:
                        crop_left = max(0, int(x))
                        crop_top = max(0, int(y))
                        crop_right = min(canvas.shape[1], int(x + w))
                        crop_bottom = min(canvas.shape[0], int(y + h))
                        armor_crop = canvas[crop_top:crop_bottom, crop_left:crop_right]
                        model_cls = best_cls
                        color_evidence = analyze_armor_light_color(
                            armor_crop,
                            min_saturation=vehicle_color_min_saturation,
                            min_value=vehicle_color_min_value,
                            min_pixels=vehicle_color_min_pixels,
                            dominance_ratio=vehicle_color_dominance_ratio,
                            side_strip_ratio=vehicle_color_side_strip_ratio,
                            yellow_as_red=vehicle_color_yellow_as_red,
                            yellow_hue_min=vehicle_color_yellow_hue_min,
                            yellow_hue_max=vehicle_color_yellow_hue_max,
                            require_light_bar_pair=vehicle_color_require_pair,
                            bar_min_aspect_ratio=vehicle_color_bar_min_aspect,
                            bar_min_height_ratio=vehicle_color_bar_min_height,
                            bar_min_height_similarity=vehicle_color_bar_min_similarity,
                            bar_max_y_offset_ratio=vehicle_color_bar_max_y_offset,
                            bar_max_angle_difference=vehicle_color_bar_max_angle_diff,
                            allow_single_light_bar=vehicle_color_allow_single_bar,
                            single_bar_confidence_scale=vehicle_color_single_bar_confidence_scale,
                            allow_compact_light_pair=vehicle_color_allow_compact_pair,
                            compact_pair_max_armor_height=vehicle_color_compact_pair_max_height,
                            compact_pair_min_aspect_ratio=vehicle_color_compact_pair_min_aspect,
                            compact_pair_confidence_scale=vehicle_color_compact_pair_confidence_scale,
                        )
                        detected_light_color = color_evidence['color']
                        detected_light_confidence = color_evidence['confidence']
                        detected_light_bars = (
                            color_evidence['red_bars']
                            if detected_light_color == 'R'
                            else color_evidence['blue_bars'] if detected_light_color == 'B' else 0
                        )
                        best_cls, color_held, color_committed = vehicle_color_memory.resolve(
                            best_cls,
                            detected_light_color,
                            (left, top, car_w, car_h),
                            frame_index,
                            model_confidence=best_conf,
                            detected_confidence=detected_light_confidence,
                            detected_bars=detected_light_bars,
                            detected_kind=color_evidence['kind'],
                            return_state=True,
                        )
                        if vehicle_color_debug_overlay:
                            color_debug_text = (
                                f"M={model_cls[0]}:{best_conf:.2f} "
                                f"L={detected_light_color or '?'}:{detected_light_confidence:.2f} "
                                f"bars={color_evidence['red_bars']}/{color_evidence['blue_bars']} "
                                f"kind={color_evidence['kind'] or '-'}"
                            )
                    # Color and class are independent gates; mature both streaks
                    # in parallel, but release only after both have confirmed.
                    confirmed_cls, confirm_count = confirm_armor_detection(
                        best_cls, (left, top, car_w, car_h), frame_index
                    )
                    if not color_committed:
                        cv2.rectangle(img0, (int(x0), int(y0)), (int(x0 + w), int(y0 + h)),
                                      (0, 165, 255), 2)
                        cv2.putText(img0, f'{best_cls} {best_conf:.2f}', (int(x0), int(y0) - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                        status = f'color pending | class {confirm_count}/{armor_confirmation_required_count}'
                        cv2.putText(img0, status, (int(x0), int(y0 + h + 22)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)
                        if color_debug_text:
                            cv2.putText(img0, color_debug_text, (int(x0), int(y0 + h + 44)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                        continue
                    if confirmed_cls is None:
                        cv2.rectangle(img0, (int(x0), int(y0)), (int(x0 + w), int(y0 + h)), (0, 165, 255), 2)
                        cv2.putText(img0, f'{best_cls} {best_conf:.2f}', (int(x0), int(y0) - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                        status = "invalid" if confirm_count == 0 else f"pending {confirm_count}/{armor_confirmation_required_count}"
                        if color_held:
                            status += " | color hold"
                        cv2.putText(img0, status, (int(x0), int(y0 + h + 22)), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.65, (0, 165, 255), 2)
                        if color_debug_text:
                            cv2.putText(img0, color_debug_text, (int(x0), int(y0 + h + 44)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                        continue
                    confirmed_armor_candidates.append({
                        'cls': confirmed_cls,
                        'conf': best_conf,
                        'armor_xywh': (x0, y0, w, h),
                        'car_box': (left, top, car_w, car_h),
                        'roi_idx': idx,
                        'color_held': color_held,
                        'color_debug_text': color_debug_text,
                    })

    confirmed_roi_indices = {candidate['roi_idx'] for candidate in confirmed_armor_candidates}
    recognized_names = set()
    measured_names = set()
    if confirmed_armor_candidates:
        winners = set()
        duplicate_groups = {}
        for candidate_index, candidate in enumerate(confirmed_armor_candidates):
            key = _armor_duplicate_key(candidate['cls'])
            if not armor_duplicate_enabled or key is None:
                winners.add(candidate_index)
                continue
            duplicate_groups.setdefault(key, []).append(candidate_index)

        for group_indices in duplicate_groups.values():
            winner_index = max(group_indices, key=lambda i: confirmed_armor_candidates[i]['conf'])
            winners.add(winner_index)

        for candidate_index, candidate in enumerate(confirmed_armor_candidates):
            best_cls = candidate['cls']
            best_conf = candidate['conf']
            x0, y0, w, h = candidate['armor_xywh']
            left, top, car_w, car_h = candidate['car_box']
            color_held = candidate.get('color_held', False)
            color_debug_text = candidate.get('color_debug_text')

            if candidate_index not in winners:
                cv2.rectangle(img0, (int(x0), int(y0)), (int(x0 + w), int(y0 + h)), (128, 128, 128), 2)
                cv2.putText(img0, f'{best_cls} {best_conf:.2f}', (int(x0), int(y0) - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)
                cv2.putText(img0, 'duplicate lower conf', (int(x0), int(y0 + h + 22)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (128, 128, 128), 2)
                continue

            cv2.rectangle(img0, (int(x0), int(y0)), (int(x0 + w), int(y0 + h)), (255, 0, 255), 2)
            cv2.putText(img0, f'{best_cls} {best_conf:.2f}', (int(x0), int(y0) - 5), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 0, 255), 2)
            if color_held:
                cv2.putText(img0, 'color hold', (int(x0), int(y0 + h + 22)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, (0, 255, 255), 2)
            if color_debug_text:
                cv2.putText(img0, color_debug_text, (int(x0), int(y0 + h + 44)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 255), 1)
            if projection_point_source == 'armor_bottom':
                point_x = x0 + projection_armor_x_ratio * w
                point_y = y0 + projection_armor_y_ratio * h
            else:
                point_x = left + projection_car_x_ratio * car_w
                point_y = top + projection_car_y_ratio * car_h
            point_x = min(max(point_x, 0), img_x)
            point_y = min(max(point_y, 0), img_y)
            cv2.circle(img0, (int(point_x), int(point_y)), 6, (0, 255, 255), -1)
            # 原图中的车体落点/装甲板锚点作为待仿射变化的点
            X_M, Y_M = project_image_point_to_map(point_x, point_y)
            if X_M is None:  # 3D 射线未命中 mesh，跳过该候选
                continue
            recognized_names.add(best_cls)
            measured_names.add(best_cls)
            add_position_measurement(best_cls, X_M, Y_M, (left, top, car_w, car_h))

    for roi_idx, car_box in enumerate(roi_pos):
        if roi_idx in confirmed_roi_indices:
            continue
        held_name = refresh_temporary_death_hold(car_box, recognized_names)
        if held_name is None:
            continue
        left, top, car_w, car_h = car_box
        if temporary_death_update_position:
            point_x = left + projection_car_x_ratio * car_w
            point_y = top + projection_car_y_ratio * car_h
            X_M, Y_M = project_image_point_to_map(point_x, point_y)
            if X_M is None:  # 3D 射线未命中 mesh，跳过保持
                continue
            add_position_measurement(held_name, X_M, Y_M, car_box, temporary_dead=True)
            recognized_names.add(held_name)
            cv2.circle(img0, (int(point_x), int(point_y)), 6, (255, 255, 0), -1)
        cv2.rectangle(img0, (int(left), int(top)), (int(left + car_w), int(top + car_h)), (255, 255, 0), 2)
        cv2.putText(img0, f'{held_name} temp dead hold', (int(left), int(top) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    # 获取所有识别到的机器人坐标
    def project_to_map_and_serial(robot_name, xy):
        """按当前阵营state统一换算坐标，避免同帧出现两套坐标系。"""
        if state == 'R':
            map_xy = (2800 - xy[1], xy[0])
        else:                                           
            map_xy = (xy[1], 1500 - xy[0])
        ser_x = int(map_xy[0]) * 10 / 10
        ser_y = int(1500 - map_xy[1]) * 10 / 10
        return map_xy, (ser_x, ser_y)

    all_filter_data = apply_occlusion_hold(filter.get_all_data())
    hold_cache_names = set(occlusion_hold_cache) if occlusion_hold_enabled else set()
    valid_position_names, current_occlusion_names = classify_legacy_target_names(
        measured_names,
        hold_cache_names,
    )
    publish_referee_target_states(valid_position_names, current_occlusion_names)
    compare_filter_data = compare_filter.get_all_data() if compare_filter is not None else {}
    # print(all_filter_data_name)
    if all_filter_data != {}:
        for name, xyxy in all_filter_data.items():
            # print(name, xyxy)
            if xyxy is not None:
                if name[0] == "R":
                    color_m = (0, 0, 255)
                else:
                    color_m = (255, 0, 0)
                filtered_xyz, (ser_x, ser_y) = project_to_map_and_serial(name, xyxy)
                point = (int(filtered_xyz[0]), int(filtered_xyz[1]))

                # 双方同时绘制：敌方实心圆，己方空心圆，便于快速区分。
                thickness = -1 if name[0] != state else 3
                cv2.circle(map, point, 15, color_m, thickness)
                cv2.putText(map, str(name),
                            (point[0] - 5, point[1] + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 5)
                cv2.putText(map, "(" + str(ser_x) + "," + str(ser_y) + ")",
                            (point[0] - 100, point[1] + 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4)

    compare_map = map_backup.copy() if compare_filter is not None and compare_draw else None
    if compare_filter is not None and compare_draw and compare_filter_data:
        for name, xyxy in compare_filter_data.items():
            if xyxy is None:
                continue
            cmp_xyz, (cmp_ser_x, cmp_ser_y) = project_to_map_and_serial(name, xyxy)
            cmp_point = (int(cmp_xyz[0]), int(cmp_xyz[1]))
            cmp_thickness = -1 if name[0] != state else 3
            cv2.circle(compare_map, cmp_point, 15, (0, 255, 255), cmp_thickness)
            cv2.putText(compare_map, str(name), (cmp_point[0] - 5, cmp_point[1] + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 5)
            cv2.putText(compare_map, f"({cmp_ser_x},{cmp_ser_y})", (cmp_point[0] - 100, cmp_point[1] + 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4)

            if compare_text and name in all_filter_data and all_filter_data[name] is not None:
                main_xy = all_filter_data[name]
                main_xyz, _ = project_to_map_and_serial(name, main_xy)
                delta = np.sqrt((main_xyz[0] - cmp_xyz[0]) ** 2 + (main_xyz[1] - cmp_xyz[1]) ** 2)
                cv2.putText(compare_map, f"d={delta:.1f}", (cmp_point[0] + 12, cmp_point[1] + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

    te = time.time()
    t_p = te - ts
    latest_vision_fps = 1 / t_p if t_p > 0 else 0.0
    latest_inference_ms = max(t_p, 0.0) * 1000.0
    if te - last_runtime_status_update >= 1.0:
        measured_camera_fps = camera_frame_rate.snapshot()
        camera_fps_text = "--" if measured_camera_fps is None else f"{measured_camera_fps:.1f}"
        print(
            f"camera_fps: {camera_fps_text} | "
            f"processing_fps: {latest_vision_fps:.1f}"
        )
        transport_status = referee_transport.snapshot() if referee_transport is not None else {
            'mode': referee_mode, 'connected': ser1 is not None
        }
        report_runtime_status(
            heartbeat_at=te,
            fps=round(latest_vision_fps, 1),
            processing_fps=round(latest_vision_fps, 1),
            camera_fps=(
                None
                if measured_camera_fps is None
                else round(measured_camera_fps, 1)
            ),
            inference_ms=round(latest_inference_ms, 1),
            referee_transport=referee_mode,
            referee_communication=transport_status,
            radio_ros_connected=bool(transport_status.get('radio_online', False)),
        )
        last_runtime_status_update = te
    # 绘制UI
    _ = draw_information_ui(vulnerability, state, information_ui_show)
    cv2.putText(information_ui_show, "vulnerability_chances: " + str(double_vulnerability_chance),
                (10, 350),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(information_ui_show, "vulnerability_Triggering: " + str(opponent_double_vulnerability),
                (10, 400),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.imshow('information_ui', information_ui_show)
    map_show = cv2.resize(map, tuple(config['ui']['map_display_size']))
    # cv2.putText(map_show, "Main Filter", (20, 40),
    #             cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    cv2.imshow('map_main', map_show)
    if compare_map is not None:
        map_compare_show = cv2.resize(compare_map, tuple(config['ui']['map_display_size']))
        cv2.putText(map_compare_show, f"Compare Filter: {compare_type}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        cv2.imshow('map_compare', map_compare_show)
    img_show = cv2.resize(img0, tuple(config['ui']['img_display_size']))
    cv2.imshow('img', img_show)
    # 提交完整处理画面；后台按 recording 最大宽度等比缩放，屏幕窗口尺寸不参与录像。
    queue_recording_frame(map, img0)
    key = cv2.waitKey(1) & 0xFF
    if key in (27, ord('q')):
        stop_requested = True

close_video_recorder()
if ser1 is not None and ser1.is_open:
    ser1.close()
if referee_transport is not None:
    referee_transport.close()
cv2.destroyAllWindows()
report_runtime_status(phase="stopped", stopped_at=time.time(), recording=False)
