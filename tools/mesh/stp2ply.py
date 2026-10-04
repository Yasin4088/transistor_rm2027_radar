# stp → PLY 转换脚本（FreeCAD 命令行）
# 用法: freecadcmd-python3 stp2ply.py <输入.stp> <输出.ply>
import sys
import os

STP_PATH = sys.argv[1] if len(sys.argv) > 1 else '/home/elysia/robomaster/UTF-8__RMUC2026_V2.0.0.stp'
PLY_PATH = sys.argv[2] if len(sys.argv) > 2 else '/home/elysia/robomaster/rmuc2026_full.ply'

import FreeCAD
import Part
import Mesh

doc = FreeCAD.newDocument('field')
print('导入 STEP...', flush=True)
Part.insert(STP_PATH, doc.Name)
print('STEP 导入完成，对象数:', len(doc.Objects), flush=True)

# 列出所有对象名，便于后续删减
print('对象列表:', flush=True)
for o in doc.Objects:
    try:
        print(' -', o.Label, '| shape体积:', round(o.Shape.Volume, 3) if hasattr(o, 'Shape') else 'N/A', flush=True)
    except Exception:
        print(' -', o.Label, '(无Shape)', flush=True)

# 合并所有 shape
print('合并形状...', flush=True)
shapes = []
for o in doc.Objects:
    try:
        if hasattr(o, 'Shape'):
            shapes.append(o.Shape)
    except Exception:
        pass

if not shapes:
    print('没有可导出的形状!', flush=True)
    sys.exit(1)

compound = Part.makeCompound(shapes)
print('合并完成，开始网格化...', flush=True)

# 网格化（线性偏差，值越小越精细）
mesh = Mesh.Mesh()
mesh.addFacets(compound.tessellate(100.0))  # 100mm 面片
print('网格面数:', len(mesh.Facets), flush=True)

# 写出 PLY
Mesh.export([mesh], PLY_PATH)
print('已导出:', PLY_PATH, flush=True)
print('完成!', flush=True)
