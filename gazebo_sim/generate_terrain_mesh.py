#!/usr/bin/env python3
"""
generate_terrain_mesh.py

Converts the BC terrain heightmap PNG into a triangle mesh (OBJ) at the
same world scale used in generate_world.py. This lets the terrain be
loaded by RGL for LiDAR raycasting (RGL does not support <heightmap>
geometry - it only supports meshes).

Texturing is handled entirely via the SDF <material><pbr> block in
generate_world.py, not via an OBJ .mtl file - this avoids Ogre2
silently ignoring one material source when both are present.

Usage:
    python3 generate_terrain_mesh.py

Output:
    materials/meshes/terrain.obj   (vertices + normals + UVs + faces)
"""

from PIL import Image
import numpy as np
import os

SCRIPT_DIR    = os.path.dirname(os.path.abspath(__file__))
MATERIALS_DIR = os.path.join(SCRIPT_DIR, 'materials', 'textures')
HEIGHTMAP_PNG = os.path.join(MATERIALS_DIR, 'bc_terrain_heightmap_256.png')

MESH_DIR   = os.path.join(SCRIPT_DIR, 'materials', 'meshes')
OBJ_PATH   = os.path.join(MESH_DIR, 'terrain.obj')

TERRAIN_W    = 50.0
TERRAIN_H    = 50.0
TERRAIN_Z    = 1.0
TEXTURE_SIZE = 4.0

GRID_RES = 256


def build_mesh():
    img = Image.open(HEIGHTMAP_PNG).convert("L")
    arr = np.array(img)
    img_h, img_w = arr.shape

    if (img_w, img_h) != (GRID_RES, GRID_RES):
        img_r = img.resize((GRID_RES, GRID_RES), Image.BILINEAR)
        arr = np.array(img_r)

    n = GRID_RES
    xs = np.linspace(-TERRAIN_W / 2, TERRAIN_W / 2, n)
    ys = np.linspace(-TERRAIN_H / 2, TERRAIN_H / 2, n)

    vertices = []
    uvs = []
    for j in range(n):
        for i in range(n):
            x = xs[i]
            y = ys[j]
            z = (float(arr[j, i]) / 255.0) * TERRAIN_Z
            vertices.append((x, y, z))
            uvs.append((x / TEXTURE_SIZE, y / TEXTURE_SIZE))

    verts_arr = np.array(vertices).reshape(n, n, 3)
    dzdx = np.gradient(verts_arr[:, :, 2], axis=1)
    dzdy = np.gradient(verts_arr[:, :, 2], axis=0)
    normals = np.zeros_like(verts_arr)
    normals[:, :, 0] = -dzdx
    normals[:, :, 1] = -dzdy
    normals[:, :, 2] = 1.0
    norm_len = np.linalg.norm(normals, axis=2, keepdims=True)
    normals = normals / np.clip(norm_len, 1e-8, None)
    normals = normals.reshape(-1, 3)

    faces = []
    for j in range(n - 1):
        for i in range(n - 1):
            v00 = j * n + i + 1
            v10 = j * n + (i + 1) + 1
            v01 = (j + 1) * n + i + 1
            v11 = (j + 1) * n + (i + 1) + 1
            faces.append((v00, v10, v11))
            faces.append((v00, v11, v01))

    os.makedirs(MESH_DIR, exist_ok=True)

    with open(OBJ_PATH, 'w') as f:
        for x, y, z in vertices:
            f.write(f"v {x:.5f} {y:.5f} {z:.5f}\n")
        for u, v in uvs:
            f.write(f"vt {u:.5f} {v:.5f}\n")
        for nx, ny, nz in normals:
            f.write(f"vn {nx:.5f} {ny:.5f} {nz:.5f}\n")
        for a, b, c in faces:
            f.write(f"f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}\n")

    print(f"Written: {OBJ_PATH}")
    print(f"Vertices: {len(vertices)}  Triangles: {len(faces)}")


if __name__ == '__main__':
    build_mesh()
