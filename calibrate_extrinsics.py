import numpy as np
import pandas as pd
import cv2

# 读取标定点
df = pd.read_excel('points_xyz.xlsx')
obj_xlsx = df[['x', 'y', 'z']].values.astype(np.float64)  # x=长边, y=短边, z=高
img_pts = np.loadtxt('image_points.txt').astype(np.float64)  # 像素坐标，顺序与序号一致

# 轴序转换：xlsx(x长边,y短边,z高) → mesh(x短边,y高,z长边)
obj_mesh = np.column_stack([obj_xlsx[:, 1], obj_xlsx[:, 2], obj_xlsx[:, 0]])

print(f'3D 点: {obj_mesh.shape[0]} 个 | 图像点: {img_pts.shape[0]} 个')
assert len(obj_mesh) == len(img_pts), '点数不一致!'

# 相机内参（棋盘格标定）
data = np.load('camera_intrinsics.npz')
K = data['K'].astype(np.float64)
dist = data['dist'].astype(np.float64)
print(f'内参 K: fx={K[0,0]:.1f}, fy={K[1,1]:.1f}, cx={K[0,2]:.1f}, cy={K[1,2]:.1f}')

# PnP 求解（ITERATIVE）
success, rvec, tvec = cv2.solvePnP(
    objectPoints=obj_mesh.astype(np.float64),
    imagePoints=img_pts.astype(np.float64),
    cameraMatrix=K,
    distCoeffs=dist,
    flags=cv2.SOLVEPNP_ITERATIVE,
)

if success:
    R, _ = cv2.Rodrigues(rvec)
    print('\n=== 标定成功 ===')
    print('旋转矩阵 R:')
    print(R)
    print('\n平移向量 t:')
    print(tvec)

    # 残差
    proj, _ = cv2.projectPoints(obj_mesh, rvec, tvec, K, dist)
    proj = proj.reshape(-1, 2)
    errs = np.linalg.norm(proj - img_pts, axis=1)
    print(f'\n平均残差: {errs.mean():.2f} px')
    print(f'最大残差: {errs.max():.2f} px')
    for i, e in enumerate(errs):
        print(f'  点{df.序号[i]}: {e:.2f}px')

    # 保存 R/T
    np.savez('extrinsics.npz', R=R, t=tvec)
    print('\n已保存: extrinsics.npz (R, t)')
else:
    print('PnP 求解失败!')
