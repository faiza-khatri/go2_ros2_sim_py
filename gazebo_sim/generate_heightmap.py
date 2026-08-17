#!/usr/bin/env python3
"""
generate_heightmap.py
Creates a smooth, gently-rolling grayscale heightmap PNG at a valid
Ogre2 resolution (2^n + 1), authored directly for the target world size
so there's no stretching distortion when Gazebo applies <size>.

Usage:
    python3 generate_heightmap.py
Output:
    bc_terrain_heightmap_257.png  (257x257, 2^8+1)
"""

import numpy as np
from PIL import Image

OUT_RES = 257          # 2^8 + 1, valid Ogre2 heightmap size
BASE_GRID = 9          # low-res control grid -> smooth interpolation up to OUT_RES
RELIEF_STRENGTH = 0.35 # 0..1, how much of the full 0-255 range to use (keep low = gentle slopes)
SEED = 42

rng = np.random.default_rng(SEED)

# Low-res random height control points
base = rng.random((BASE_GRID, BASE_GRID)).astype(np.float32)

# Upsample smoothly via PIL bicubic interpolation (acts like smooth terrain, no sharp jumps)
base_img = Image.fromarray((base * 255).astype(np.uint8), mode="L")
smooth = base_img.resize((OUT_RES, OUT_RES), Image.BICUBIC)
arr = np.array(smooth).astype(np.float32)

# Normalize to 0-255, then compress range to keep slopes gentle
arr = (arr - arr.min()) / (arr.max() - arr.min())  # 0..1
arr = arr * RELIEF_STRENGTH * 255.0
arr = arr.astype(np.uint8)

out_img = Image.fromarray(arr, mode="L")
out_img.save("bc_terrain_heightmap_257.png")
print(f"Saved bc_terrain_heightmap_257.png at {out_img.size}, relief range 0-{int(255*RELIEF_STRENGTH)}")
