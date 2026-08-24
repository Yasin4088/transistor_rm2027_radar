import numpy as np
import open3d as o3d
import cv2


def build_pixel_to_world_from_npz(npz_path, mesh, R=None, T=None):
    """从标定 npz 构建 PixelToWorld（R/T 可选，缺省用单位矩阵+俯视10m假外参）"""
    data = np.load(npz_path)
    K = data['K'].astype(np.float64)
    dist = data['dist'].astype(np.float64)
    if R is None:
        R = np.eye(3)
    if T is None:
        T = np.array([[0.0], [0.0], [10.0]])
    return PixelToWorld(K, R, T, mesh, dist_coeffs=dist)


class PixelToWorld:
    """像素 → 世界坐标：反投影成射线，与场地 3D mesh 求交（移植自 HKUST RM2025 开源）"""

    def __init__(self, camera_matrix, R, T, mesh, dist_coeffs=None):
        self.camera_matrix = np.array(camera_matrix, dtype=np.float64)
        self.dist_coeffs = (np.zeros(5) if dist_coeffs is None
                            else np.array(dist_coeffs, dtype=np.float64))
        self.R = np.array(R, dtype=np.float64)
        self.T = np.array(T, dtype=np.float64).reshape(3, 1)
        self.mesh = mesh
        self.scene = o3d.t.geometry.RaycastingScene()
        self.scene.add_triangles(
            o3d.t.geometry.TriangleMesh.from_legacy(self.mesh))

    def pixel_to_world(self, pixel):
        u, v = pixel
        # ① 去畸变
        if not np.all(self.dist_coeffs == 0):
            pts = np.array([[u, v]], dtype=np.float32).reshape(-1, 1, 2)
            undist = cv2.undistortPoints(pts, self.camera_matrix,
                                         self.dist_coeffs, P=self.camera_matrix)
            u, v = undist[0, 0]
        # ② 内参反投影 → 相机系方向
        pixel_hom = np.array([u, v, 1.0], dtype=np.float64)
        cam_dir = np.linalg.inv(self.camera_matrix) @ pixel_hom
        # ③ 外参旋转 → 世界系方向
        world_dir = self.R.T @ cam_dir
        origin = -self.R.T @ self.T.flatten()
        # ④ 射线与 mesh 求交
        ray = o3d.core.Tensor([[*origin, *world_dir]], dtype=o3d.core.Dtype.Float32)
        t_hit = self.scene.cast_rays(ray)["t_hit"].numpy()[0]
        if t_hit < float("inf"):
            return origin + t_hit * world_dir
        return None

    def __call__(self, pixel):
        return self.pixel_to_world(pixel)


if __name__ == "__main__":
    import sys
    # 冒烟测试：加载 2025 PLY，随机发射一条射线验证求交
    mesh = o3d.io.read_triangle_mesh(
        "/home/elysia/robomaster/RM2025-Radar-Algorithm/field/RMUC2025_National.PLY")
    print("mesh 加载成功, 顶点数:", len(np.asarray(mesh.vertices)))
    p2w = PixelToWorld(
        camera_matrix=[[2500.0, 0.0, 1536.0], [0.0, 2500.0, 1024.0], [0.0, 0.0, 1.0]],
        R=np.eye(3),
        T=np.array([[0.0], [0.0], [10.0]]),
        mesh=mesh,
    )
    world = p2w.pixel_to_world((1536, 1024))
    print("中心像素射线求交结果:", world)
