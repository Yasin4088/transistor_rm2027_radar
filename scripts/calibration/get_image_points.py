import cv2
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
CALIB_DIR = REPO_ROOT / "data" / "calibration"

IMG_PATH = CALIB_DIR / "calibration_frame.jpg"
OUT_PATH = CALIB_DIR / "image_points.txt"

img = cv2.imread(str(IMG_PATH))
if img is None:
    raise FileNotFoundError(f"无法读取标定图像: {IMG_PATH}")

points = []


def on_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        print(f"点 {len(points)}: ({x}, {y})")
        cv2.circle(img, (x, y), 6, (0, 0, 255), -1)
        cv2.imshow("points", img)


cv2.imshow("points", img)
cv2.setMouseCallback("points", on_click)
print("按图上编号 1→6 依次点击，点完按 ESC")
cv2.waitKey(0)
cv2.destroyAllWindows()

CALIB_DIR.mkdir(parents=True, exist_ok=True)
with open(OUT_PATH, "w") as f:
    for x, y in points:
        f.write(f"{x} {y}\n")
print(f"已保存 {len(points)} 个点到: {OUT_PATH}")
