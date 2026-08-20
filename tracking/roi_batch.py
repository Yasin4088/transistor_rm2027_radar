from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

import cv2
import numpy as np

from .types import Box


@dataclass(frozen=True)
class RoiTransform:
    track_id: int
    crop_box: Box
    scale: float
    pad_x: float
    pad_y: float
    tile_x: int = 0
    tile_y: int = 0

    def tile_box_to_image(self, box: Box) -> Box:
        x1, y1, x2, y2 = box
        crop_x1, crop_y1, _, _ = self.crop_box
        return (
            (x1 - self.tile_x - self.pad_x) / self.scale + crop_x1,
            (y1 - self.tile_y - self.pad_y) / self.scale + crop_y1,
            (x2 - self.tile_x - self.pad_x) / self.scale + crop_x1,
            (y2 - self.tile_y - self.pad_y) / self.scale + crop_y1,
        )


def expand_box(box: Box, image_shape: Sequence[int], margin_ratio: float) -> Box:
    height, width = image_shape[:2]
    x1, y1, x2, y2 = map(float, box)
    margin_x = (x2 - x1) * float(margin_ratio)
    margin_y = (y2 - y1) * float(margin_ratio)
    return (
        max(0.0, x1 - margin_x),
        max(0.0, y1 - margin_y),
        min(float(width), x2 + margin_x),
        min(float(height), y2 + margin_y),
    )


def make_letterboxed_roi(
    image: np.ndarray,
    track_id: int,
    box: Box,
    size: int = 320,
    margin_ratio: float = 0.08,
) -> Tuple[np.ndarray, RoiTransform]:
    crop_box = expand_box(box, image.shape, margin_ratio)
    x1, y1, x2, y2 = crop_box
    ix1, iy1 = int(np.floor(x1)), int(np.floor(y1))
    ix2, iy2 = int(np.ceil(x2)), int(np.ceil(y2))
    crop = image[iy1:iy2, ix1:ix2]
    if crop.size == 0:
        raise ValueError(f"empty ROI for track {track_id}: {crop_box}")
    height, width = crop.shape[:2]
    scale = min(size / width, size / height)
    resized_width = max(1, int(round(width * scale)))
    resized_height = max(1, int(round(height * scale)))
    resized = cv2.resize(crop, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    tile = np.zeros((size, size, 3), dtype=np.uint8)
    pad_x = (size - resized_width) // 2
    pad_y = (size - resized_height) // 2
    tile[pad_y:pad_y + resized_height, pad_x:pad_x + resized_width] = resized
    transform = RoiTransform(
        track_id=int(track_id),
        crop_box=(float(ix1), float(iy1), float(ix2), float(iy2)),
        scale=float(scale),
        pad_x=float(pad_x),
        pad_y=float(pad_y),
    )
    return tile, transform


def pack_tiles(
    tiles: Sequence[np.ndarray],
    transforms: Sequence[RoiTransform],
    tile_size: int = 320,
    canvas_size: int = 1280,
) -> Tuple[List[np.ndarray], List[List[RoiTransform]]]:
    if len(tiles) != len(transforms):
        raise ValueError("tiles and transforms must have the same length")
    if canvas_size % tile_size:
        raise ValueError("canvas_size must be divisible by tile_size")
    per_row = canvas_size // tile_size
    per_canvas = per_row * per_row
    canvases: List[np.ndarray] = []
    canvas_transforms: List[List[RoiTransform]] = []
    for start in range(0, len(tiles), per_canvas):
        canvas = np.zeros((canvas_size, canvas_size, 3), dtype=np.uint8)
        group: List[RoiTransform] = []
        for local_index, (tile, transform) in enumerate(
            zip(tiles[start:start + per_canvas], transforms[start:start + per_canvas])
        ):
            row, column = divmod(local_index, per_row)
            tile_x, tile_y = column * tile_size, row * tile_size
            canvas[tile_y:tile_y + tile_size, tile_x:tile_x + tile_size] = tile
            group.append(
                RoiTransform(
                    track_id=transform.track_id,
                    crop_box=transform.crop_box,
                    scale=transform.scale,
                    pad_x=transform.pad_x,
                    pad_y=transform.pad_y,
                    tile_x=tile_x,
                    tile_y=tile_y,
                )
            )
        canvases.append(canvas)
        canvas_transforms.append(group)
    return canvases, canvas_transforms


def find_transform_for_box(
    box: Box,
    transforms: Iterable[RoiTransform],
    tile_size: int = 320,
) -> RoiTransform | None:
    x1, y1, x2, y2 = box
    cx, cy = (x1 + x2) * 0.5, (y1 + y2) * 0.5
    for transform in transforms:
        if (
            transform.tile_x <= cx < transform.tile_x + tile_size
            and transform.tile_y <= cy < transform.tile_y + tile_size
        ):
            return transform
    return None
