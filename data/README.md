## 数据目录

该目录保存视觉定位相关的数据资产与标定产物。

```text
data/
├── README.md
├── calibration/                # 相机标定结果
│   ├── camera_intrinsics.npz   # 相机内参
│   └── extrinsics.npz          # 相机外参（R/t）
├── field/                      # 3D 场地网格（射线求交）
│   ├── RMUC2026_National.PLY   # 默认全国赛场地模型
│   └── RMUC2026_simple.PLY     # 简化场地模型（测试/占位）
└── homography_matrix/          # 2D 仿射回退矩阵
    ├── arrays_test_red.npy
    └── arrays_test_blue.npy
```
