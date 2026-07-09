#!/usr/bin/env python3
"""
generate_world.py  (forest.glb variant)

Ground + trees now come from a single pre-made asset (forest.glb) instead
of a procedurally-scattered heightmap forest. This script:

  1. Places forest.glb as one static visual (ground + tree canopies baked
     together — whatever draw-call count is internal to that asset).
  2. Adds a flat <collision><plane> for ground physics, since forest.glb
     itself carries no Gazebo collision data.
  3. Adds one collision-only cylinder per tree, positioned from
     tree_layout_from_asset.json (produced by extract_tree_layout_blender.py)
     — NOT randomly generated like the old spruce/fir version.

Prerequisite (run once, or whenever forest.glb changes):
    blender --background --python extract_tree_layout_blender.py \\
        -- models/forest_scene/forest.glb tree_layout_from_asset.json

Usage:
    python3 generate_world.py
Output:
    world/cafe.world
"""

import os
import json

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# ─────────────────────────────────────────────
# EDIT THESE TO MATCH YOUR ASSET
# ─────────────────────────────────────────────
# Where you saved the sourced forest asset:
FOREST_GLB = os.path.join(SCRIPT_DIR, 'models', 'forest_scene', 'forest.glb')

# Ground plane size in meters — check the "Plane" object's Dimensions
# (press N in Blender, select Plane, read X/Y) and set these to match.
GROUND_SIZE_X = 50.0   # ← placeholder, confirm against Blender's N-panel
GROUND_SIZE_Y = 50.0   # ← placeholder, confirm against Blender's N-panel

# Ground height — the Plane's world Z from the screenshot was ~0.0
GROUND_Z = 0.0

# Trunk collision cylinder dimensions (same idea as before: a thin proxy
# for contact/lidar, not a canopy-accurate collision shape)
COLLISION_RADIUS = 0.15
COLLISION_LENGTH = 2.0

TREE_LAYOUT_JSON = os.path.join(SCRIPT_DIR, 'tree_layout_from_asset.json')
OUTPUT_FILE = os.path.join(SCRIPT_DIR, 'world', 'cafe.world')

APRILTAGS = [
    # (name,                    x,     y,    z_offset, roll,   pitch,  yaw)
    ("Apriltag36_11_00000",  -4.96,  1.5,   0.46,    1.5708, 0.0,  1.5708),
]

# ─────────────────────────────────────────────
# LOAD TREE LAYOUT (extracted from the asset, not randomly generated)
# ─────────────────────────────────────────────
if not os.path.exists(TREE_LAYOUT_JSON):
    raise FileNotFoundError(
        f"{TREE_LAYOUT_JSON} not found. Run this first:\n"
        f"  blender --background --python extract_tree_layout_blender.py "
        f"-- {FOREST_GLB} {TREE_LAYOUT_JSON}"
    )

with open(TREE_LAYOUT_JSON) as f:
    TREES = json.load(f)

print(f"Loaded {len(TREES)} tree positions from {TREE_LAYOUT_JSON}")

# ─────────────────────────────────────────────
# SDF BUILDERS
# ─────────────────────────────────────────────
def forest_visual_sdf():
    return f"""
    <model name="forest_visual">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>{FOREST_GLB}</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>"""

def ground_collision_sdf():
    return f"""
    <model name="forest_ground_collision">
      <static>true</static>
      <pose>0 0 {GROUND_Z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{GROUND_SIZE_X} {GROUND_SIZE_Y}</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.8</mu>
                <mu2>0.8</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>
    </model>"""

def tree_collision_only_sdf(name, x, y, z, radius, length):
    return f"""
    <model name="{name}_collision">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{radius}</radius>
              <length>{length}</length>
            </cylinder>
          </geometry>
          <pose>0 0 {length/2:.4f} 0 0 0</pose>
        </collision>
      </link>
    </model>"""

def apriltag_sdf(name, x, y, z_offset, roll, pitch, yaw):
    z = GROUND_Z + z_offset
    return f"""
    <include>
      <name>{name}</name>
      <pose>{x} {y} {z:.4f} {roll} {pitch} {yaw}</pose>
      <static>true</static>
      <uri>model://{name}</uri>
    </include>"""

# ─────────────────────────────────────────────
# ASSEMBLE WORLD
# ─────────────────────────────────────────────
tree_collisions_xml = "".join(
    tree_collision_only_sdf(t["name"], t["x"], t["y"], t["z"],
                             COLLISION_RADIUS, COLLISION_LENGTH)
    for t in TREES
)

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
    <plugin name='rgl::RGLServerPluginManager' filename='RGLServerPluginManager'>
        <do_ignore_entities_in_lidar_link>true</do_ignore_entities_in_lidar_link>
    </plugin>

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

    <!-- ── Ground + trees visual, baked as one sourced asset ── -->
{forest_visual_sdf()}

    <!-- ── Flat physics ground (forest.glb carries no collision) ── -->
{ground_collision_sdf()}

    <!-- ── Per-tree collision-only proxies, positioned from the asset ── -->
{tree_collisions_xml}

    <!-- ── AprilTags ─────────────────────────── -->
{apriltag_xml}

  </world>
</sdf>
"""

os.makedirs(os.path.dirname(OUTPUT_FILE), exist_ok=True)
with open(OUTPUT_FILE, "w") as f:
    f.write(world)

print(f"Written: {OUTPUT_FILE}")
print(f"Ground: {GROUND_SIZE_X}x{GROUND_SIZE_Y}m at z={GROUND_Z}  "
      f"(confirm these match forest.glb's actual Plane dimensions)")
print(f"Trees: {len(TREES)} collision proxies, 1 visual draw call for the whole forest")
