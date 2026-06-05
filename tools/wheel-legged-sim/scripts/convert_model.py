#!/usr/bin/env python3
"""
Convert wode_dao_dun GLB to MuJoCo-compatible mesh files.
- y-up → z-up, cm → m
- Per-part STLs (split if >200k face limit)
- Single merged OBJ (no face limit)
"""
import struct
import math
from pathlib import Path

import numpy as np
import pygltflib

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent.parent
GLB_PATH = ROOT / "esp32_app" / "data" / "3d" / "wode_dao_dun.glb"
MESH_DIR = ROOT / "tools" / "wheel-legged-sim" / "meshes"

SCALE = 0.01
STL_LIMIT = 190_000

idx_dtype_map = {5121: np.dtype('B'), 5123: np.dtype('<u2'), 5125: np.dtype('<u4')}


def transform(verts):
    """GLB y-up → MuJoCo z-up: (x,y,z)→(-z,-x,y)
    - MuJoCo +x = forward = -GLB z (character faces viewer in GLB)
    - MuJoCo +y = left = -GLB x
    - MuJoCo +z = up = GLB y
    """
    tv = np.empty_like(verts)
    tv[:, 0] = -verts[:, 2]   # new x = -old z
    tv[:, 1] = -verts[:, 0]   # new y = -old x
    tv[:, 2] = verts[:, 1]    # new z = old y
    return tv * SCALE


def write_stl(path, verts, faces):
    nf = len(faces)
    with open(path, 'wb') as f:
        f.write(b'MuJoCo mesh\x00' + b'\x00' * 69)
        f.write(struct.pack('<I', nf))
        for i0, i1, i2 in faces:
            v0, v1, v2 = verts[i0], verts[i1], verts[i2]
            n = np.cross(v1 - v0, v2 - v0)
            nl = np.linalg.norm(n)
            n = n / nl if nl > 1e-12 else np.array([0., 0., 1.])
            f.write(struct.pack('<3f', *n))
            f.write(struct.pack('<3f', *v0))
            f.write(struct.pack('<3f', *v1))
            f.write(struct.pack('<3f', *v2))
            f.write(b'\x00\x00')
    return nf


def write_obj(path, verts, faces):
    nf = len(faces)
    with open(path, 'w') as f:
        for v in verts:
            f.write(f'v {v[0]:.8f} {v[1]:.8f} {v[2]:.8f}\n')
        for i0, i1, i2 in faces:
            f.write(f'f {i0+1} {i1+1} {i2+1}\n')
    mb = path.stat().st_size / 1_000_000
    print(f"Wrote merged OBJ: {len(verts):,} verts, {nf:,} faces, {mb:.1f} MB")


def main():
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    for f in list(MESH_DIR.glob("*.stl")) + list(MESH_DIR.glob("*.obj")):
        f.unlink()

    print(f"Loading: {GLB_PATH}")
    gltf = pygltflib.GLTF2().load(str(GLB_PATH))
    binary = gltf._glb_data
    assert isinstance(binary, bytes)
    print(f"Binary: {len(binary):,} bytes, {len(gltf.meshes)} meshes\n")

    all_verts: list[np.ndarray] = []
    all_faces: list[np.ndarray] = []
    total_v = 0

    for mi, mesh in enumerate(gltf.meshes):
        for prim in mesh.primitives:
            pa = gltf.accessors[prim.attributes.POSITION]
            bv = gltf.bufferViews[pa.bufferView]
            off = (bv.byteOffset or 0) + (pa.byteOffset or 0)
            verts = np.frombuffer(binary, dtype=np.float32,
                                  count=pa.count * 3, offset=off) \
                .reshape(-1, 3).astype(np.float64)

            ia = gltf.accessors[prim.indices]
            bv2 = gltf.bufferViews[ia.bufferView]
            off2 = (bv2.byteOffset or 0) + (ia.byteOffset or 0)
            idt = idx_dtype_map.get(ia.componentType, np.dtype('<u4'))
            faces = np.frombuffer(binary, dtype=idt,
                                  count=ia.count, offset=off2) \
                .astype(np.int32).reshape(-1, 3)

            verts = transform(verts)
            name = mesh.name or f"mesh_{mi}"
            nf = len(faces)
            print(f"  [{mi:2d}] {name[:40]:40s} {len(verts):>7,} verts  {nf:>7,} faces", end="")

            if nf <= STL_LIMIT:
                fpath = MESH_DIR / f"part_{mi:02d}.stl"
                write_stl(fpath, verts, faces)
                print(f"  -> {fpath.name}")
            else:
                nchunks = math.ceil(nf / STL_LIMIT)
                chunk_sz = math.ceil(nf / nchunks)
                for ci in range(nchunks):
                    s = ci * chunk_sz
                    e = min(s + chunk_sz, nf)
                    cf = faces[s:e]
                    fpath = MESH_DIR / f"part_{mi:02d}_{ci}.stl"
                    write_stl(fpath, verts, cf)
                    print(f"\n           chunk {ci}: {len(cf):,} faces -> {fpath.name}")

            # Accumulate for merged OBJ
            all_verts.append(verts)
            all_faces.append(faces + total_v)
            total_v += len(verts)

    n_stl = len(list(MESH_DIR.glob("*.stl")))
    total_mb = sum(f.stat().st_size for f in MESH_DIR.glob("*.stl")) / 1_000_000
    print(f"\nSTL files: {n_stl}, {total_mb:.1f} MB")

    print("\n--- Merging into OBJ ---")
    merged_verts = np.vstack(all_verts)
    merged_faces = np.vstack(all_faces)
    obj_path = MESH_DIR / "wode_dao_dun.obj"
    write_obj(obj_path, merged_verts, merged_faces)
    print("Done.")


if __name__ == "__main__":
    main()
