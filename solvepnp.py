import numpy as np
import cv2
from typing import Tuple, Optional


class PnPSolver:
    def __init__(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray, verbose=False):
        """
        PnP 相机外参求解器（移植自 HKUST RM2025 开源 transform/solvepnp.py）

        参数：
            camera_matrix (np.ndarray): 相机内参矩阵 (3x3)
            dist_coeffs (np.ndarray): 畸变系数 (5,)
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.verbose = verbose

    @classmethod
    def from_config(cls, config: dict | str) -> "PnPSolver":
        """从配置（dict 或 yaml 路径）创建，格式：config["transform"]["K"] / ["dist_coeffs"]"""
        if isinstance(config, str):
            import yaml

            with open(config, "r") as f:
                config = yaml.safe_load(f)
            return cls.from_config(config)

        camera_matrix = np.array(config["transform"]["K"], dtype=np.float32)
        dist_coeffs = np.array(config["transform"]["dist_coeffs"], dtype=np.float32)
        return cls(camera_matrix, dist_coeffs, config["transform"].get("verbose", True))

    def solve(
        self, object_points: np.ndarray, image_points: np.ndarray
    ) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray], Optional[float]]:
        """
        PnP 求解相机姿态。

        参数：
            object_points (np.ndarray): 3D 世界坐标点 (N, 3)
            image_points (np.ndarray): 2D 图像坐标点 (M, 2)，M <= N

        返回：
            success (bool)
            R (np.ndarray): 旋转矩阵 (3x3)
            tvec (np.ndarray): 平移向量 (3x1)
            residual (float): 平均投影残差（像素）
        """
        self.object_point = object_points
        self.image_point = image_points

        if object_points.shape[0] < 4 or image_points.shape[0] < 4:
            raise ValueError(
                "At least 4 points are required for PnP. "
                f"Got {object_points.shape[0]} object points and {image_points.shape[0]} image points."
            )

        N = object_points.shape[0]
        M = image_points.shape[0]
        if M > N:
            raise ValueError(
                "Number of image points cannot exceed number of object points. "
                f"Got {N} object points and {M} image points."
            )

        if M == N:
            # 点数对齐：直接 ITERATIVE 求解（原版 L167 分支）
            success, rvec, tvec = cv2.solvePnP(
                objectPoints=object_points,
                imagePoints=image_points,
                cameraMatrix=self.camera_matrix,
                distCoeffs=self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            self.rvec, self.tvec = rvec, tvec
            if success:
                R, _ = cv2.Rodrigues(rvec)
                residual = self.calculate_residual(object_points, image_points)
                if self.verbose:
                    print("Rotation matrix R:")
                    print(R)
                    print("\nOffset t:")
                    print(tvec)
                    print("\nProjection residual (mean):", residual)
                return success, R, tvec, residual
            if self.verbose:
                print("PnP Solve failed")
            return False, None, None, None
        else:
            # M < N：暴力枚举 3D 点组合，用 RANSAC 找最优匹配（原版 L85 分支）
            import itertools
            import tqdm

            best_inliers, max_inliers = [], 0
            best_rvec, best_tvec, best_indices = None, None, None
            for indices in tqdm.tqdm(itertools.permutations(range(N), M)):
                success, rvec, tvec, inliers = cv2.solvePnPRansac(
                    objectPoints=object_points[list(indices)],
                    imagePoints=image_points,
                    cameraMatrix=self.camera_matrix,
                    distCoeffs=self.dist_coeffs,
                    reprojectionError=6.0,
                    confidence=0.99,
                    iterationsCount=100,
                    flags=cv2.SOLVEPNP_EPNP,
                )
                if success and len(inliers) > max_inliers:
                    max_inliers = len(inliers)
                    best_inliers = inliers
                    best_rvec, best_tvec = rvec, tvec
                    best_indices = list(indices)

            if max_inliers >= 4:
                object_points_matched = object_points[best_indices]
                # 用内点重新精化
                success, rvec, tvec = cv2.solvePnP(
                    objectPoints=object_points_matched[best_inliers.flatten()],
                    imagePoints=image_points[best_inliers.flatten()],
                    cameraMatrix=self.camera_matrix,
                    distCoeffs=self.dist_coeffs,
                    rvec=best_rvec,
                    tvec=best_tvec,
                    useExtrinsicGuess=True,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )
                self.rvec, self.tvec = rvec, tvec
                R, _ = cv2.Rodrigues(rvec)
                residual = self.calculate_residual(object_points_matched, image_points)
                if self.verbose:
                    print("Best matched 3D point indices:", best_indices)
                    print("Inliers:", best_inliers.flatten())
                    print("Rotation matrix R:")
                    print(R)
                    print("\nOffset t:")
                    print(tvec)
                    print("\nProjection residual (mean):", residual)
                return True, R, tvec, residual
            if self.verbose:
                print("PnP Solve with brute-force search failed")
            return False, None, None, None

    def calculate_residual(self, object_points: np.ndarray, image_points: np.ndarray) -> float:
        """计算平均投影残差（像素）：3D 点用 R/t 投影回图像，与标注点比距离"""
        if not hasattr(self, "rvec") or not hasattr(self, "tvec"):
            raise ValueError("Please solve the PnP first using the solve method.")
        self.projected_points, _ = cv2.projectPoints(
            objectPoints=object_points,
            rvec=self.rvec,
            tvec=self.tvec,
            cameraMatrix=self.camera_matrix,
            distCoeffs=self.dist_coeffs,
        )
        self.projected_points = self.projected_points.reshape(-1, 2)
        residual = np.linalg.norm(self.projected_points - image_points, axis=1)
        return np.mean(residual)

    def draw_visualize_image(self, img: np.ndarray) -> np.ndarray:
        """可视化：红色=投影点（由 R/t 算出），绿色=标注点"""
        img = img.copy()
        if not hasattr(self, "projected_points"):
            raise ValueError("Please solve the PnP first using the solve method.")

        for point in self.projected_points:
            x, y = point.astype(int)
            if 0 <= x < img.shape[1] and 0 <= y < img.shape[0]:
                cv2.circle(img, (x, y), 5, (0, 0, 255), -1)

        for point in self.image_point:
            x, y = point.astype(int)
            if 0 <= x < img.shape[1] and 0 <= y < img.shape[0]:
                cv2.circle(img, (x, y), 5, (0, 255, 0), -1)
        return img


if __name__ == "__main__":
    # 冒烟测试：用 HKUST 的 demo 图 + keypoint_6.txt 验证链路
    # 注意：需要图像点的像素坐标（在图上手动点 6 个点，或从标定文件读）
    import sys

    if len(sys.argv) < 3:
        print("用法: python solvepnp.py <图像点txt> <图路径>")
        print("图像点 txt 每行: x y（6 行，与 keypoint_6.txt 顺序对应）")
        sys.exit(1)

    image_points = np.loadtxt(sys.argv[1], dtype=np.float32)
    object_points = np.loadtxt(
        "/home/elysia/robomaster/RM2025-Radar-Algorithm/transform/keypoint_6.txt",
        dtype=np.float32,
    )
    # HKUST demo 相机内参（2000万像素相机）
    pnpsolver = PnPSolver(
        camera_matrix=np.array(
            [[5033.780199, 0.0, 2829.234535],
             [0.0, 5036.139955, 1929.489557],
             [0.0, 0.0, 1.0]], dtype=np.float32),
        dist_coeffs=np.array([-0.061883, 0.104794, 0.000434, -0.000036, 0.0], dtype=np.float32),
        verbose=True,
    )
    success, R, tvec, residual = pnpsolver.solve(object_points, image_points)
    if success:
        vis = pnpsolver.draw_visualize_image(cv2.imread(sys.argv[2]))
        cv2.imwrite("/tmp/opencode/pnp_result.png", vis)
        print("结果已保存: /tmp/opencode/pnp_result.png")
