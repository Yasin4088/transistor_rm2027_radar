import cv2
import os

IMG_PATH = '/home/elysia/robomaster/RM2025-Radar-Algorithm/demo/demo1.jpg'
OUT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'demo_points.txt')

img = cv2.imread(IMG_PATH)
points = []

def on_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        print(f'点 {len(points)}: ({x}, {y})')
        cv2.circle(img, (x, y), 6, (0, 0, 255), -1)
        cv2.imshow('points', img)

cv2.imshow('points', img)
cv2.setMouseCallback('points', on_click)
print('按图上编号 1→6 依次点击，点完按 ESC')
cv2.waitKey(0)
cv2.destroyAllWindows()

with open(OUT_PATH, 'w') as f:
    for x, y in points:
        f.write(f'{x} {y}\n')
print(f'已保存 {len(points)} 个点到: {OUT_PATH}')
