from PIL import Image
import numpy as np

HEIGHTMAP_PNG = "/home/administrator/go_sim/src/gazebo_sim/materials/textures/bc_terrain_heightmap_256.png"
TERRAIN_W = 50.0
TERRAIN_H = 50.0
TERRAIN_Z = 1.0

img = Image.open(HEIGHTMAP_PNG).convert("L")
arr = np.array(img)
IMG_H, IMG_W = arr.shape

print(f"Image size: {IMG_W}x{IMG_H}")
print(f"Pixel value range: min={arr.min()}, max={arr.max()}, mean={arr.mean():.1f}")
print(f"\nSample terrain_z values at object positions:")

def terrain_z_v1(wx, wy):
    """Your original formula"""
    px = int(np.clip((wx + TERRAIN_W/2) / TERRAIN_W * IMG_W, 0, IMG_W-1))
    py = int(np.clip((wy + TERRAIN_H/2) / TERRAIN_H * IMG_H, 0, IMG_H-1))
    normalized = float(arr[py, px]) / 255.0
    return (normalized - 0.5) * TERRAIN_Z + 1.0

def terrain_z_v2(wx, wy):
    """Fixed formula"""
    px = int(np.clip((wx + TERRAIN_W/2) / TERRAIN_W * IMG_W, 0, IMG_W-1))
    py = int(np.clip((wy + TERRAIN_H/2) / TERRAIN_H * IMG_H, 0, IMG_H-1))
    normalized = float(arr[py, px]) / 255.0
    return (normalized - 0.5) * TERRAIN_Z

def terrain_z_v3(wx, wy):
    """Gazebo Harmonic heightmap — zero-based, no centering"""
    px = int(np.clip((wx + TERRAIN_W/2) / TERRAIN_W * IMG_W, 0, IMG_W-1))
    py = int(np.clip((wy + TERRAIN_H/2) / TERRAIN_H * IMG_H, 0, IMG_H-1))
    normalized = float(arr[py, px]) / 255.0
    return normalized * TERRAIN_Z

test_points = [(0.5, -1.6), (3.0, 2.0), (-2.5, 3.5), (0, 0)]
for x, y in test_points:
    print(f"  ({x:5.1f}, {y:5.1f})  v1={terrain_z_v1(x,y):.4f}  v2={terrain_z_v2(x,y):.4f}  v3={terrain_z_v3(x,y):.4f}")
