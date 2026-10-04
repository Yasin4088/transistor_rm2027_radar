import cv2
import numpy as np
import glob
from pathlib import Path

# 棋盘格参数：9x12 格子 -> 内部角点 8x11
CHESS_COLS = 11  # 每行内部角点数 = 列格数 - 1
CHESS_ROWS = 8   # 每列内部角点数 = 行格数 - 1
SQUARE_MM = 30.0  # 每格边长 mm

REPO_ROOT = Path(__file__).resolve().parents[2]
CALIB_DIR = REPO_ROOT / "data" / "calibration"
DATA_DIR = CALIB_DIR / "chessboard"
OUT_PATH = CALIB_DIR / "camera_intrinsics.npz"

images = sorted(
    glob.glob(str(DATA_DIR / "*.bmp")) + glob.glob(str(DATA_DIR / "*.jpg"))
)
print(f"找到 {len(images)} 张图片")

# 生成 3D 棋盘角点坐标（z=0 平面）
objp = np.zeros((CHESS_ROWS * CHESS_COLS, 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESS_COLS, 0:CHESS_ROWS].T.reshape(-1, 2) * SQUARE_MM

obj_points, img_points = [], []
ok_count = 0
for path in images:
    img = cv2.imread(path)
    if img is None:
        print(f"  读取失败: {path}")
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (CHESS_COLS, CHESS_ROWS), None)
    if ret:
        corners = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        obj_points.append(objp)
        img_points.append(corners)
        ok_count += 1
    else:
        print(f"  未找到角点: {Path(path).name}")

print(f"成功检测角点: {ok_count}/{len(images)}")

if ok_count < 5:
    print("角点检测成功数太少，检查棋盘格参数或图片质量")
    raise SystemExit(1)

# 标定
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, cv2.imread(images[0]).shape[:2][::-1], None, None
)

print(f"\n重投影误差(RMS): {ret:.3f} px")
print("\n内参矩阵 K:")
print(K)
print(f"\n畸变系数 dist: {dist.ravel()}")

# 保存
CALIB_DIR.mkdir(parents=True, exist_ok=True)
np.savez(str(OUT_PATH), K=K, dist=dist)
print(f"\n已保存到: {OUT_PATH}")

# 可视化检测结果（第一张成功图）
for path in images:
    img = cv2.imread(path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (CHESS_COLS, CHESS_ROWS), None)
    if ret:
        cv2.drawChessboardCorners(img, (CHESS_COLS, CHESS_ROWS), corners, ret)
        cv2.imwrite("/tmp/opencode/calib_check.png", img)
        print("角点检测可视化已保存: /tmp/opencode/calib_check.png")
        break
