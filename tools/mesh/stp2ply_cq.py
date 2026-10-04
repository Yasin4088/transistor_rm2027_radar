# stp → PLY 转换（cadquery-ocp/OCC 内核，支持新 STEP）
import sys
import time

STP_PATH = sys.argv[1] if len(sys.argv) > 1 else '/home/elysia/robomaster/UTF-8__RMUC2026_V2.0.0.stp'
PLY_PATH = sys.argv[2] if len(sys.argv) > 2 else '/home/elysia/robomaster/rmuc2026_full.ply'

from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID

t0 = time.time()
print('读取 STEP...', flush=True)

reader = STEPControl_Reader()
status = reader.ReadFile(STP_PATH)
print(f'ReadFile status: {status} ({"OK" if status == IFSelect_RetDone else "FAIL"})', flush=True)

if status != IFSelect_RetDone:
    print('STEP 读取失败!', flush=True)
    sys.exit(1)

reader.TransferRoots()
shape = reader.OneShape()
print(f'STEP 读取完成，耗时 {time.time()-t0:.1f}s', flush=True)

# 统计 SOLID 数
n_solids = 0
exp = TopExp_Explorer(shape, TopAbs_SOLID)
while exp.More():
    n_solids += 1
    exp.Next()
print(f'SOLID 数: {n_solids}', flush=True)

# 网格化（10mm 精度，先试可行）
from OCP.BRepMesh import BRepMesh_IncrementalMesh
print('网格化...', flush=True)
BRepMesh_IncrementalMesh(shape, 10.0)

from OCP.StlAPI import StlAPI_Writer
stl_path = '/tmp/opencode/rmuc2026.stl'
writer = StlAPI_Writer()
writer.Write(shape, stl_path)
print(f'STL 已写出: {stl_path} 耗时 {time.time()-t0:.1f}s', flush=True)

# STL → PLY
import trimesh
m = trimesh.load(stl_path)
print(f'STL 面数: {len(m.faces)}', flush=True)
m.export(PLY_PATH)
print(f'PLY 已导出: {PLY_PATH}', flush=True)
print('完成!', flush=True)
