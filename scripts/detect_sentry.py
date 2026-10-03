#!/usr/bin/env python3
"""Live sentry detection diagnostic without projection, radio, or referee I/O.

The first-stage detector exposes both ``car`` and ``watcher`` classes.  The
match pipeline currently consumes only ``car``; this diagnostic deliberately
keeps both classes so that a real sentry can reveal which label the model uses.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from collections import Counter
from ctypes import POINTER, byref, c_ubyte, cast, memset, sizeof
from pathlib import Path

import cv2
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
for relative in (".", "src", "tools"):
    path = str(ROOT / relative)
    if path not in sys.path:
        sys.path.insert(0, path)

from capture.mvs_runtime import configure_mvs_runtime  # noqa: E402
from detect.detect_function import YOLOv5Detector  # noqa: E402


VEHICLE_CLASSES = {"car", "watcher"}
SENTRY_CLASSES = {"R7", "B7"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="只测试车体/哨兵装甲识别，不定位、不连接 SDR 或裁判系统"
    )
    parser.add_argument(
        "--source",
        default="hik",
        help="hik（海康相机）或视频文件路径，默认 hik",
    )
    parser.add_argument("--start-frame", type=int, default=0, help="视频测试起始帧")
    parser.add_argument("--max-frames", type=int, default=0, help="0 表示持续运行")
    parser.add_argument("--no-display", action="store_true", help="不显示窗口，用于录像冒烟测试")
    parser.add_argument("--exposure", type=float, help="临时曝光时间（微秒），仅海康相机")
    parser.add_argument("--gain", type=float, help="临时模拟增益，仅海康相机")
    parser.add_argument(
        "--output-dir",
        default="/tmp/transistor_sentry_detection",
        help="截图目录（默认位于 /tmp，不写入仓库）",
    )
    return parser.parse_args()


class VideoSource:
    def __init__(self, path: str, start_frame: int = 0):
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开视频：{path}")
        if start_frame > 0:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

    def read(self) -> np.ndarray | None:
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self) -> None:
        self.cap.release()


class HikSource:
    def __init__(self, camera_config: dict):
        configure_mvs_runtime()
        from capture.hik_camera import (
            MV_ACCESS_Exclusive,
            MV_CC_DEVICE_INFO,
            MV_CC_DEVICE_INFO_LIST,
            MV_FRAME_OUT_INFO_EX,
            MV_GIGE_DEVICE,
            MV_USB_DEVICE,
            MVCC_INTVALUE_EX,
            MvCamera,
            hik_device_serial,
            image_control,
            select_hik_device_index,
            set_Value,
        )

        self._image_control = image_control
        self.cam = MvCamera()
        self.started = False
        self.opened = False
        self.handle_created = False

        devices = MV_CC_DEVICE_INFO_LIST()
        ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, devices)
        if ret != 0:
            raise RuntimeError(f"海康相机枚举失败：0x{ret:x}")
        if devices.nDeviceNum == 0:
            raise RuntimeError("未发现海康相机")

        print(f"枚举到 {devices.nDeviceNum} 台海康相机：")
        for index in range(devices.nDeviceNum):
            info = cast(devices.pDeviceInfo[index], POINTER(MV_CC_DEVICE_INFO)).contents
            print(f"  [{index}] serial={hik_device_serial(info) or '(未知)'}")

        serial = str(camera_config.get("device_serial", "") or "").strip()
        index = select_hik_device_index(devices, serial_wanted=serial, default_index=0)
        info = cast(devices.pDeviceInfo[index], POINTER(MV_CC_DEVICE_INFO)).contents

        ret = self.cam.MV_CC_CreateHandle(info)
        if ret != 0:
            raise RuntimeError(f"创建相机句柄失败：0x{ret:x}")
        self.handle_created = True
        ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            self.close()
            raise RuntimeError(f"打开相机失败：0x{ret:x}")
        self.opened = True

        set_Value(
            self.cam,
            param_type="float_value",
            node_name="ExposureTime",
            node_value=float(camera_config.get("exposure_time", 11000.0)),
        )
        set_Value(
            self.cam,
            param_type="float_value",
            node_name="Gain",
            node_value=float(camera_config.get("gain", 15.0)),
        )
        ret = self.cam.MV_CC_StartGrabbing()
        if ret != 0:
            self.close()
            raise RuntimeError(f"开始取流失败：0x{ret:x}")
        self.started = True

        payload = MVCC_INTVALUE_EX()
        memset(byref(payload), 0, sizeof(MVCC_INTVALUE_EX))
        ret = self.cam.MV_CC_GetIntValueEx("PayloadSize", payload)
        if ret != 0:
            self.close()
            raise RuntimeError(f"获取 PayloadSize 失败：0x{ret:x}")
        self.buffer = (c_ubyte * int(payload.nCurValue))()
        self.frame_info = MV_FRAME_OUT_INFO_EX()

    def read(self) -> np.ndarray | None:
        memset(byref(self.frame_info), 0, sizeof(type(self.frame_info)))
        ret = self.cam.MV_CC_GetOneFrameTimeout(
            self.buffer, len(self.buffer), self.frame_info, 1000
        )
        if ret != 0:
            print(f"相机暂时无帧：0x{ret:x}")
            return None
        raw = np.asarray(self.buffer)
        return self._image_control(raw, self.frame_info)

    def close(self) -> None:
        if self.started:
            self.cam.MV_CC_StopGrabbing()
            self.started = False
        if self.opened:
            self.cam.MV_CC_CloseDevice()
            self.opened = False
        if self.handle_created:
            self.cam.MV_CC_DestroyHandle()
            self.handle_created = False


def clamp_roi(x: int, y: int, w: int, h: int, shape: tuple[int, ...]):
    height, width = shape[:2]
    x1 = max(0, min(x, width - 1))
    y1 = max(0, min(y, height - 1))
    x2 = max(x1 + 1, min(x + w, width))
    y2 = max(y1 + 1, min(y + h, height))
    return x1, y1, x2, y2


def draw_label(image: np.ndarray, text: str, point: tuple[int, int], color):
    x, y = point
    cv2.putText(
        image,
        text,
        (x, max(24, y)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        color,
        2,
        cv2.LINE_AA,
    )


def display_frame(image: np.ndarray, max_width: int = 1700) -> np.ndarray:
    if image.shape[1] <= max_width:
        return image
    scale = max_width / image.shape[1]
    return cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)


def save_frame(output_dir: Path, frame: np.ndarray, prefix: str, frame_index: int) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    path = output_dir / f"{prefix}_{timestamp}_{frame_index:06d}.jpg"
    cv2.imwrite(str(path), frame)
    print(f"已保存截图：{path}")
    return path


def main() -> int:
    args = parse_args()
    os.chdir(ROOT)
    with open(ROOT / "config/config.yaml", "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    source = None
    try:
        if args.source.lower() == "hik":
            camera_config = dict(config.get("camera_params", {}) or {})
            if args.exposure is not None:
                camera_config["exposure_time"] = args.exposure
            if args.gain is not None:
                camera_config["gain"] = args.gain
            source = HikSource(camera_config)
        else:
            source = VideoSource(args.source, args.start_frame)

        inference = config.get("inference", {})
        car_detector = YOLOv5Detector(
            config["paths"]["models"]["car"],
            img_size=tuple(inference.get("car_img_size", [640, 640])),
            data="config/car.yaml",
            conf_thres=float(config.get("algorithm", {}).get("vehicle_tracker", {}).get("low_confidence", 0.10)),
            iou_thres=0.5,
            max_det=20,
            ui=False,
        )
        armor_detector = YOLOv5Detector(
            config["paths"]["models"]["armor"],
            img_size=tuple(inference.get("armor_img_size", [1280, 1280])),
            data="config/armor.yaml",
            conf_thres=float(inference.get("armor_conf_threshold", 0.4)),
            iou_thres=0.2,
            max_det=10,
            ui=False,
        )

        output_dir = Path(args.output_dir)
        frame_index = 0
        started_at = time.monotonic()
        class_counts: Counter[str] = Counter()
        armor_counts: Counter[str] = Counter()
        last_auto_save = -1000

        print("诊断已启动：绿色=car，橙色=watcher，青色=装甲板；Q/Esc退出，S保存截图")
        try:
            while args.max_frames <= 0 or frame_index < args.max_frames:
                frame = source.read()
                if frame is None:
                    if args.source.lower() != "hik":
                        break
                    continue
                frame_index += 1
                annotated = frame.copy()
                vehicles = car_detector.predict(frame)
                sentry_seen = False

                for class_name, xywh, confidence in vehicles:
                    class_name = str(class_name)
                    class_counts[class_name] += 1
                    x, y, w, h = map(int, xywh)
                    color = (0, 200, 0) if class_name == "car" else (0, 140, 255)
                    cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 3)
                    draw_label(annotated, f"{class_name} {confidence:.2f}", (x, y - 8), color)
                    if class_name not in VEHICLE_CLASSES:
                        continue

                    x1, y1, x2, y2 = clamp_roi(x, y, w, h, frame.shape)
                    crop = frame[y1:y2, x1:x2]
                    if crop.size == 0:
                        continue
                    for armor_name, armor_xywh, armor_confidence in armor_detector.predict(crop):
                        armor_name = str(armor_name)
                        armor_counts[armor_name] += 1
                        ax, ay, aw, ah = map(int, armor_xywh)
                        ax += x1
                        ay += y1
                        armor_color = (255, 255, 0) if armor_name in SENTRY_CLASSES else (255, 180, 40)
                        cv2.rectangle(
                            annotated, (ax, ay), (ax + aw, ay + ah), armor_color, 3
                        )
                        draw_label(
                            annotated,
                            f"{armor_name} {armor_confidence:.2f}",
                            (ax, ay - 8),
                            armor_color,
                        )
                        sentry_seen |= armor_name in SENTRY_CLASSES

                elapsed = max(time.monotonic() - started_at, 1e-6)
                summary = (
                    f"frame={frame_index} avg_fps={frame_index / elapsed:.1f} "
                    f"car={class_counts['car']} watcher={class_counts['watcher']} "
                    f"R7={armor_counts['R7']} B7={armor_counts['B7']}"
                )
                cv2.rectangle(annotated, (0, 0), (min(annotated.shape[1], 1250), 52), (0, 0, 0), -1)
                draw_label(annotated, summary, (12, 36), (255, 255, 255))

                if sentry_seen and frame_index - last_auto_save >= 30:
                    save_frame(output_dir, annotated, "sentry", frame_index)
                    last_auto_save = frame_index

                key = -1
                if not args.no_display:
                    cv2.imshow("Sentry detection diagnostic", display_frame(annotated))
                    key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("s"):
                    save_frame(output_dir, annotated, "manual", frame_index)
        except KeyboardInterrupt:
            print("\n收到停止请求，正在释放相机")

        print("\n=== 诊断统计 ===")
        print("车辆类别：", dict(class_counts))
        print("装甲类别：", dict(armor_counts))
        print("哨兵装甲：", {name: armor_counts[name] for name in sorted(SENTRY_CLASSES)})
        return 0
    finally:
        if source is not None:
            source.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
