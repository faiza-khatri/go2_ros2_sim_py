#!/usr/bin/env python3
"""
generate_world.py
Generates forest.world for Gazebo Harmonic from a config.
All object Z poses are computed automatically from the heightmap.

Usage:
    python3 generate_world.py
Output:
    forest.world  (same directory as this script)
"""

from PIL import Image
import numpy as np
import os
import random
import subprocess

# ─────────────────────────────────────────────
# TERRAIN CONFIG  ← edit these
# ─────────────────────────────────────────────
# Source paths (used for heightmap sampling only)
SCRIPT_DIR    = os.path.dirname(os.path.abspath(__file__))
MATERIALS_DIR = os.path.join(SCRIPT_DIR, 'materials', 'textures')
HEIGHTMAP_PNG = os.path.join(MATERIALS_DIR, 'bc_terrain_heightmap_256.png')

# Installed paths (written into the world SDF for Gazebo to load)
pkg_share    = subprocess.check_output(['ros2', 'pkg', 'prefix', '--share', 'gazebo_sim']).decode().strip()
GZ_MATERIALS = os.path.join(pkg_share, 'materials', 'textures')
GZ_HEIGHTMAP = os.path.join(GZ_MATERIALS, 'bc_terrain_heightmap_256.png')
GZ_DIFFUSE   = os.path.join(GZ_MATERIALS, 'bc_moss_rock_diffuse.png')
GZ_NORMAL    = os.path.join(GZ_MATERIALS, 'bc_moss_rock_normal.png')

TERRAIN_W   = 50.0
TERRAIN_H   = 50.0
TERRAIN_Z   = 1.0
TERRAIN_POS = (0, 0, 0)
TEXTURE_SIZE = 4

X_MIN, X_MAX = -20, 20
Y_MIN, Y_MAX = -20, 20

OUTPUT_FILE = os.path.join(SCRIPT_DIR, 'world', 'cafe.world')

# ─────────────────────────────────────────────
# HEIGHTMAP SAMPLER
# ─────────────────────────────────────────────
img = Image.open(HEIGHTMAP_PNG).convert("L")
arr = np.array(img)
IMG_H, IMG_W = arr.shape

    
TREE_HALF_HEIGHT = 1.0  # half of tree mesh height in metres

def terrain_z(wx, wy):
    px = int(np.clip((wx + TERRAIN_W/2) / TERRAIN_W * IMG_W, 0, IMG_W-1))
    py = int(np.clip((wy + TERRAIN_H/2) / TERRAIN_H * IMG_H, 0, IMG_H-1))
    normalized = float(arr[py, px]) / 255.0
    return normalized * TERRAIN_Z
# ─────────────────────────────────────────────
# OBJECTS CONFIG  ← add / remove / move freely
# Each entry: (name, x, y, z_offset, type, ...type_args)
#
# Types:
#   "mesh"    → (uri, scale, collision_radius, collision_length)
#   "include" → (model_uri,)          uses <include>
#   "apriltag"→ (model_uri, roll, pitch, yaw, z_offset_override)
# ─────────────────────────────────────────────
NUM_TREES_SPRUCE = 50
TREES_SPRUCE = []
for i in range(NUM_TREES_SPRUCE):
	x = random.uniform(X_MIN, X_MAX)
	y = random.uniform(Y_MIN, Y_MAX)
	scale = random.uniform(0.5, 0.6)
	entry = ("spruce_tree_"+str(i), x, y, 0, scale)
	TREES_SPRUCE.append(entry)
	
NUM_TREES_FIR = 50
TREES_FIR = []
for i in range(NUM_TREES_FIR):
	x = random.uniform(X_MIN, X_MAX)
	y = random.uniform(Y_MIN, Y_MAX)
	scale = random.uniform(0.15, 0.2)
	entry = ("fir_tree_"+str(i), x, y, 0, scale)
	TREES_FIR.append(entry)


TABLES = [
    # (name,    x,     y,    z_offset)   ← z_offset = table leg height (~0.2)
    ("table2",  2.4,  -5.5,  0.2),
    ("table3", -1.5,  -5.5,  0.2),
    ("table4",  2.4,  -9.0,  0.2),
    ("table5", -1.5,  -9.0,  0.2),
]

APRILTAGS = [
    # (name,                    x,     y,    z_offset, roll,   pitch,  yaw)
    ("Apriltag36_11_00000",  -4.96,  1.5,   0.46,    1.5708, 0.0,  1.5708),
]

# ─────────────────────────────────────────────
# SDF BUILDERS
# ─────────────────────────────────────────────
def tree_sdf_spruce(name, x, y, z_offset, scale):
    
    tz = terrain_z(x, y)
    z  = tz + z_offset
    s  = scale
    return f"""
    <!-- {name}: terrain_z({x}, {y}) = {tz:.4f} -->
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z:.4f} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>model://spruce_tree/spruce_tree.glb</uri>
              <scale>{s} {s} {s}</scale>
            </mesh>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.15</radius>
              <length>2.0</length>
            </cylinder>
          </geometry>
          <pose>0 0 1.0 0 0 0</pose>
        </collision>
      </link>
    </model>"""

def tree_sdf_fir(name, x, y, z_offset, scale):
    
    tz = terrain_z(x, y)
    z  = tz + z_offset
    s  = scale
    return f"""
    <!-- {name}: terrain_z({x}, {y}) = {tz:.4f} -->
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z:.4f} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>model://fir_tree/lhpine.glb</uri>
              <scale>{s} {s} {s}</scale>
            </mesh>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.15</radius>
              <length>2.0</length>
            </cylinder>
          </geometry>
          <pose>0 0 1.0 0 0 0</pose>
        </collision>
      </link>
    </model>"""
    
def table_sdf(name, x, y, z_offset):
    tz = terrain_z(x, y)
    z  = tz + z_offset
    return f"""
    <!-- {name}: terrain_z({x}, {y}) = {tz:.4f} -->
    <include>
      <name>{name}</name>
      <pose>{x} {y} {z:.4f} 0 0 0</pose>
      <uri>model://cafe_table</uri>
    </include>"""

def apriltag_sdf(name, x, y, z_offset, roll, pitch, yaw):
    tz = terrain_z(x, y)
    z  = tz + z_offset
    return f"""
    <!-- {name}: terrain_z({x}, {y}) = {tz:.4f} -->
    <include>
      <name>{name}</name>
      <pose>{x} {y} {z:.4f} {roll} {pitch} {yaw}</pose>
      <static>true</static>
      <uri>model://{name}</uri>
    </include>"""

# ─────────────────────────────────────────────
# ASSEMBLE WORLD
# ─────────────────────────────────────────────
tree_xml_spruce     = "".join(tree_sdf_spruce(*t)     for t in TREES_SPRUCE)
tree_xml_fir     = "".join(tree_sdf_fir(*t)     for t in TREES_FIR)
table_xml    = "".join(table_sdf(*t)    for t in TABLES)
apriltag_xml = "".join(apriltag_sdf(*t) for t in APRILTAGS)

world = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <plugin filename="gz-sim-physics-system"          name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system"    name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-sensors-system"          name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"              name="gz::sim::systems::Imu"></plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="terrain">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>{GZ_HEIGHTMAP}</uri>
              <size>{TERRAIN_W} {TERRAIN_H} {TERRAIN_Z}</size>
              <pos>{TERRAIN_POS[0]} {TERRAIN_POS[1]} {TERRAIN_POS[2]}</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <heightmap>
              <use_terrain_paging>false</use_terrain_paging>
              <uri>{GZ_HEIGHTMAP}</uri>
              <size>{TERRAIN_W} {TERRAIN_H} {TERRAIN_Z}</size>
              <pos>{TERRAIN_POS[0]} {TERRAIN_POS[1]} {TERRAIN_POS[2]}</pos>
              <texture>
                <diffuse>{GZ_DIFFUSE}</diffuse>
                <normal>{GZ_NORMAL}</normal>
                <size>{TEXTURE_SIZE}</size>
              </texture>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>
{tree_xml_spruce}
{tree_xml_fir}

    <!-- ── Tables ───────────────────────────── -->
{table_xml}

    <!-- ── AprilTags ─────────────────────────── -->
{apriltag_xml}

  </world>
</sdf>
"""

with open(OUTPUT_FILE, "w") as f:
    f.write(world)

print(f"Written: {OUTPUT_FILE}")
print(f"\nTerrain: {TERRAIN_W}x{TERRAIN_H}m, Z scale {TERRAIN_Z}m")
print(f"Heightmap sampled at {IMG_W}x{IMG_H}px\n")
print("Object Z poses:")
for name, x, y, z_off, scale in TREES_SPRUCE:
    tz = terrain_z(x, y)
    print(f"  {name:20s}  terrain_z={tz:.4f}  pose_z={tz+z_off:.4f}")
for name, x, y, z_off, scale in TREES_FIR:
    tz = terrain_z(x, y)
    print(f"  {name:20s}  terrain_z={tz:.4f}  pose_z={tz+z_off:.4f}")
for name, x, y, z_off in TABLES:
    tz = terrain_z(x, y)
    print(f"  {name:20s}  terrain_z={tz:.4f}  pose_z={tz+z_off:.4f}")
for name, x, y, z_off, *_ in APRILTAGS:
    tz = terrain_z(x, y)
    print(f"  {name:20s}  terrain_z={tz:.4f}  pose_z={tz+z_off:.4f}")
