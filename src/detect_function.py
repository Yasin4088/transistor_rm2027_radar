# 导入需要的库
import os
import sys
import platform
import time
from pathlib import Path, PurePosixPath, WindowsPath

# Windows下将PosixPath映射到WindowsPath，解决Linux保存的.pt模型在Windows加载报错的问题
import pathlib
if platform.system() == 'Windows':
    pathlib.PosixPath = pathlib.WindowsPath

# 初始化目录
import cv2

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # 定义YOLOv5的根目录
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # 将YOLOv5的根目录添加到环境变量中（程序结束后删除）
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from utils.general import (LOGGER, check_img_size, scale_boxes)

# 导入letterbox

import random
import torch
import numpy as np
from utils.general import non_max_suppression, xyxy2xywh
from utils.torch_utils import select_device
from utils.plots import Annotator
from models.common import DetectMultiBackend
from utils.augmentations import letterbox


class YOLOv5Detector:
    def __init__(self, weights_path, img_size=(640, 640), conf_thres=0.70, iou_thres=0.2, max_det=10,
                 device='', classes=None, agnostic_nms=False, augment=False, visualize=False, half=True, dnn=False,
                 data='data/coco128.yaml', ui=False):
        # 设置设备：默认遵从上层传入，必要时安全回退到CPU
        self.ui = ui
        requested_device = str(device).strip().lower() if device is not None else ''
        if requested_device in ('', 'none'):
            # select_device('cpu') 会设置 CUDA_VISIBLE_DEVICES=-1，导致后续 TensorRT
            # 无法再初始化 CUDA（.engine 会段错误），故 TensorRT 权重默认走 GPU
            wlow = str(weights_path).lower()
            if wlow.endswith('.engine'):
                if not torch.cuda.is_available():
                    raise RuntimeError(
                        'TensorRT 模型 (*.engine) 需要 CUDA。当前无可用 GPU，请在 config.yaml 中改用 '
                        'models/car.onnx 与 models/armor.onnx（或 .pt），或安装/修复 CUDA 驱动。'
                    )
                requested_device = '0'
            else:
                requested_device = '0' if torch.cuda.is_available() else 'cpu'

        # YOLOv5 的 select_device 常用 '0' 指代第一张GPU
        if requested_device.startswith('cuda'):
            if ':' in requested_device:
                requested_device = requested_device.split(':', 1)[1] or '0'
            else:
                requested_device = '0'
        elif requested_device == 'gpu':
            requested_device = '0'

        # CUDA不可用时回退CPU，避免初始化报错
        if requested_device != 'cpu' and not torch.cuda.is_available():
            requested_device = 'cpu'

        self.device = select_device(requested_device)
        print(f"使用设备: {self.device} (requested={device})")

        # 加载模型 - 禁用半精度(CPU不支持FP16)
        self.model = DetectMultiBackend(weights_path, device=self.device, dnn=dnn, fp16=False, data=data)

        stride, self.names, pt, jit, onnx, engine = self.model.stride, self.model.names, self.model.pt, self.model.jit, self.model.onnx, self.model.engine
        self.img_size = check_img_size(img_size, s=stride)
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]
        # CPU模式下禁用半精度
        self.half = False  # CPU不支持半精度
        if pt or jit:
            self.model.model.float()  # 使用全精度Float32
        self.save_time = 0
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.max_det = max_det
        self.classes = classes
        self.agnostic_nms = agnostic_nms
        self.augment = augment
        self.visualize = visualize
        bs = 1  # batch_size
        # 开始预测
        self.model.warmup(imgsz=(1 if pt or self.model.triton else bs, 3, *self.img_size))  # warmup

    @property
    def supports_dynamic_batch(self):
        """Return whether this backend accepts a runtime batch larger than one."""
        if self.model.pt or self.model.jit:
            return True
        if self.model.engine:
            return bool(getattr(self.model, 'dynamic', False))
        if self.model.onnx and hasattr(self.model, 'session'):
            batch_dimension = self.model.session.get_inputs()[0].shape[0]
            return isinstance(batch_dimension, str) or batch_dimension in (None, -1)
        return False

    def predict(self, img):
        # 对图片进行处理

        im0 = img.copy()
        im = letterbox(im0, self.img_size, self.model.stride, auto=self.model.pt)[0]
        # cv2.imshow('im', im)
        im = im.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        im = np.ascontiguousarray(im)

        im = torch.from_numpy(im).to(self.device)
        im = im.float()
        im /= 255
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim

        # 预测
        pred = self.model(im, augment=self.augment, visualize=self.visualize)

        # NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms,
                                   max_det=self.max_det)

        # 用于存放结果
        detections = []

        # 处理预测结果
        for i, det in enumerate(pred):
            # gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            if len(det):

                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()
                # print(det)
                for *xyxy, conf, cls in reversed(det):
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4))).view(-1).tolist()
                    xywh = [round(x) for x in xywh]
                    xywh = [xywh[0] - xywh[2] // 2, xywh[1] - xywh[3] // 2, xywh[2], xywh[3]]
                    if self.ui:
                        annotator = Annotator(np.ascontiguousarray(img), line_width=3, example=str(self.names))
                        # print(int(cls))
                        label = f'{self.names[int(cls)]} {conf:.2f}'
                        annotator.box_label(xyxy, label, color=self.colors[int(cls)])

                    cls = self.names[int(cls)]
                    conf = float(conf)
                    line = (cls, xywh, conf)
                    detections.append(line)
        # if self.save_data:
        #     if time.time() - self.save_time >= 1:
        #         self.save_time = time.time()
        #         save_path = "save_data/img/" + str(round(time.time())) + ".jpg"
        #         cv2.imwrite(save_path, img)
        #         # save_path = "save_data/img_ui/img_ui_" + str(round(time.time())) + ".jpg"
        #         # cv2.imwrite(save_path, im0)
                # print(1)

        # LOGGER.info(f'({t3 - t2:.3f}s)')

        return detections

    def predict_batch(self, images):
        """Run one model call for a list of equally sized logical images.

        Results use the same ``(class_name, xywh, confidence)`` format as
        :meth:`predict`, grouped by input image. Dynamic TensorRT, PyTorch and
        dynamic ONNX backends are supported. Static TensorRT callers should use
        the normalized-tile fallback instead.
        """
        images = list(images)
        if not images:
            return []
        if len(images) > 1 and not self.supports_dynamic_batch:
            raise RuntimeError('当前装甲模型后端不支持动态 batch')

        prepared = []
        originals = []
        for image in images:
            if image is None or image.size == 0:
                raise ValueError('batch 中存在空图像')
            original = np.ascontiguousarray(image)
            resized = letterbox(original, self.img_size, self.model.stride, auto=False)[0]
            resized = resized.transpose((2, 0, 1))[::-1]
            prepared.append(np.ascontiguousarray(resized))
            originals.append(original)

        tensor = torch.from_numpy(np.stack(prepared, axis=0)).to(self.device)
        tensor = tensor.float() / 255.0
        predictions = self.model(tensor, augment=self.augment, visualize=self.visualize)
        predictions = non_max_suppression(
            predictions,
            self.conf_thres,
            self.iou_thres,
            self.classes,
            self.agnostic_nms,
            max_det=self.max_det,
        )

        grouped = []
        for original, detections_tensor in zip(originals, predictions):
            detections = []
            if len(detections_tensor):
                detections_tensor[:, :4] = scale_boxes(
                    tensor.shape[2:], detections_tensor[:, :4], original.shape
                ).round()
                for *xyxy, conf, cls in reversed(detections_tensor):
                    xywh = xyxy2xywh(torch.tensor(xyxy).view(1, 4)).view(-1).tolist()
                    xywh = [round(value) for value in xywh]
                    xywh = [
                        xywh[0] - xywh[2] // 2,
                        xywh[1] - xywh[3] // 2,
                        xywh[2],
                        xywh[3],
                    ]
                    detections.append((self.names[int(cls)], xywh, float(conf)))
            grouped.append(detections)
        return grouped
