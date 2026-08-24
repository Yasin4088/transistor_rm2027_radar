import numpy as np
import open3d as o3d

# ============================================================
# 手建 2026 场地粗糙 mesh（依据：2026 规则手册结构 + 2026 stp 提取的尺寸/位置）
# 坐标系：Open3D 标准 xyz 米制。场地 28m(x) × 15m(y)，地面 z=0
# stp 数据换算：stp 的 z 是高度轴（地面 z=-1.8m），x∈[-14,14], y∈[-7.5,7.5]
# 转换：mesh_x = stp_x + 0（对齐场地中心），mesh_z = stp_z + 1.8（地面抬到 0）
#        mesh_y = stp_y + 0
# ============================================================

meshes = []

def add_box(center, size):
    """添加一个盒子，center=[x,y,z] 中心，size=[dx,dy,dz]"""
    box = o3d.geometry.TriangleMesh.create_box(
        width=size[0], height=size[1], depth=size[2])
    box.translate((center[0]-size[0]/2, center[1]-size[1]/2, center[2]-size[2]/2))
    return box

def add_wedge(center, size, slope_dir):
    """添加楔形（坡道/梯形高地），slope_dir: 沿哪个方向起坡"""
    # 用两个三角面构造简单楔形
    x0, y0, z0 = center[0]-size[0]/2, center[1]-size[1]/2, center[2]-size[2]/2
    dx, dy, dz = size
    if slope_dir == 'x':
        verts = np.array([
            [x0, y0, z0], [x0+dx, y0, z0], [x0+dx, y0+dy, z0], [x0, y0+dy, z0],  # 底面
            [x0, y0, z0+dz], [x0+dx, y0, z0], [x0+dx, y0+dy, z0], [x0, y0+dy, z0+dz],  # 顶面(坡)
        ])
        faces = [[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[3,7,6],[3,6,2],[1,2,6],[1,6,5],[0,4,7],[0,7,3]]
    else:  # 'y'
        verts = np.array([
            [x0, y0, z0], [x0+dx, y0, z0], [x0+dx, y0+dy, z0], [x0, y0+dy, z0],
            [x0, y0, z0+dz], [x0+dx, y0+dy, z0+dz], [x0+dx, y0+dy, z0], [x0, y0+dy, z0+dz],
        ])
        # 简化：用盒子代替楔形（粗糙版本）
    return None

# ---- 1. 地面（28×15×0.2m）----
meshes.append(add_box([0, 0, -0.1], [28, 15, 0.2]))

# ---- 2. 梯形高地 ×2（stp [0][2]: 5.5×5.57×0.5m，对称）----
# stp 位置: [0] 在 (-0.7,-3.9), [2] 在 (-4.8,1.6) —— 注意这是 stp 坐标
# 实际按 2026 手册：梯形高地各半场一个，相对地面 200-400mm
meshes.append(add_box([-4.8, 3.9, 0.3], [5.5, 5.57, 0.4]))   # 红方梯形高地(高度400mm)
meshes.append(add_box([4.8, -3.9, 0.3], [5.5, 5.57, 0.4]))   # 蓝方梯形高地

# ---- 3. 中央高地（stp [175][176]: 3.9×3.85×0.3m）----
meshes.append(add_box([0, 0, 0.3], [3.9, 3.85, 0.3]))

# ---- 4. 坡道（连接高地与地面，10.5°~15°）----
# 用薄楔形近似：长 3m，高 0.3m（10° 左右）
def add_ramp(center, size, angle_axis):
    """斜坡：size=[长,宽,高]，用盒子+旋转近似，粗糙版直接用薄盒子"""
    return add_box(center, size)

# 中央高地两端坡道（stp 10.5°坡）
meshes.append(add_box([0, 2.2, 0.15], [3.0, 2.5, 0.3]))   # 北坡
meshes.append(add_box([0, -2.2, 0.15], [3.0, 2.5, 0.3]))  # 南坡

# ---- 5. 公路区/平台（stp [153][181]: 2.41×2.09×0.32m 等）----
meshes.append(add_box([-8.7, 3.5, 0.32], [2.41, 2.09, 0.32]))
meshes.append(add_box([8.7, -3.5, 0.32], [2.41, 2.09, 0.32]))
meshes.append(add_box([6.4, 5.3, 0.3], [3.9, 3.85, 0.3]))   # 另一高地
meshes.append(add_box([-10.3, -5.9, 0.3], [3.9, 3.85, 0.3]))

# ---- 6. 起伏路段（2026 新增）：地面凸起条 ----
for i in range(4):
    meshes.append(add_box([-6+i*3, 6.5, 0.06], [2.0, 0.4, 0.12]))
    meshes.append(add_box([-6+i*3, -6.5, 0.06], [2.0, 0.4, 0.12]))

# ---- 合并 ----
combined = meshes[0]
for m in meshes[1:]:
    combined += m

print('顶点数:', len(np.asarray(combined.vertices)))
print('面数:', len(np.asarray(combined.triangles)))

# 保存
o3d.io.write_triangle_mesh('/home/elysia/robomaster/transistor_rm2027_radar/field/RMUC2026_simple.PLY', combined)
print('已保存: field/RMUC2026_simple.PLY')
