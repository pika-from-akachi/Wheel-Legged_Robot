"""将CAD OBJ转换为Web优化的GLB文件"""
import trimesh
import os
import sys

OBJ_PATH = "D:/Keilprojects/HAL_LIB/Wheel-Legged_Robot/3Dfiles/wode刀盾.obj"
MTL_PATH = "D:/Keilprojects/HAL_LIB/Wheel-Legged_Robot/3Dfiles/wode刀盾.mtl"
OUT_DIR  = "D:/Keilprojects/HAL_LIB/Wheel-Legged_Robot/esp32_app/data/3d"
OUT_FILE = os.path.join(OUT_DIR, "wode_dao_dun.glb")

os.makedirs(OUT_DIR, exist_ok=True)

print(f"[1/4] 加载 OBJ (322MB, 约748万三角面)...")
sys.stdout.flush()

scene = trimesh.load(OBJ_PATH, force='scene')
print(f"  加载完成: {len(scene.geometry)} 个子模型")
sys.stdout.flush()

# 合并所有几何体
print(f"[2/4] 合并几何体...")
sys.stdout.flush()

meshes = []
for name, geom in scene.geometry.items():
    if isinstance(geom, trimesh.Trimesh):
        meshes.append(geom)
    elif isinstance(geom, trimesh.Scene):
        for sub in geom.geometry.values():
            if isinstance(sub, trimesh.Trimesh):
                meshes.append(sub)

if not meshes:
    print("  错误: 没有找到网格数据!")
    sys.exit(1)

if len(meshes) > 1:
    combined = trimesh.util.concatenate(meshes)
else:
    combined = meshes[0]

orig_faces = len(combined.faces)
print(f"  原始三角面数: {orig_faces}")
sys.stdout.flush()

# 简化网格 - 目标50k面
# 简化网格 - 目标2%面数
if orig_faces > 50000:
    print(f"[3/4] 简化网格 ({orig_faces} -> {int(orig_faces * 0.02)} 面)...")
    sys.stdout.flush()
    simplified = combined.simplify_quadric_decimation(percent=0.98)
    print(f"  简化完成: {len(simplified.faces)} 面")
    sys.stdout.flush()
else:
    simplified = combined

# 估计文件大小并调整质量
if len(simplified.faces) > 200000:
    print("  面数仍然过高, 进一步简化...")
    sys.stdout.flush()
    simplified = combined.simplify_quadric_decimation(100000)
    print(f"  二次简化完成: {len(simplified.faces)} 面")
    sys.stdout.flush()

# 导出 GLB
print(f"[4/4] 导出 GLB...")
sys.stdout.flush()
trimesh.exchange.export.export_mesh(simplified, OUT_FILE, file_type='glb')
file_size = os.path.getsize(OUT_FILE)
print(f"\n✅ 完成!")
print(f"  输出: {OUT_FILE}")
print(f"  大小: {file_size/1024/1024:.1f} MB")
print(f"  面数: {len(simplified.faces)}")
