#!/usr/bin/env python3
"""
extract_tree_layout_blender.py

Opens a pre-made forest.glb (ground + trees baked together as one asset)
and extracts the world-space (x, y, z) base position of every tree object
inside it. Ground/material/camera/light objects are excluded by name/type
heuristics — check the printed list against your Outliner and adjust
EXCLUDE_NAMES below if anything looks wrong.

This does NOT touch or re-export the mesh. It only reads transforms to
build collision-cylinder positions for generate_world.py, since forest.glb
itself becomes the single visual asset (ground + tree canopies together)
and carries no Gazebo collision data of its own.

Usage:
    blender --background --python extract_tree_layout_blender.py -- forest.glb tree_layout_from_asset.json

Output JSON:
    [
      {"name": "scene_001", "x": 1.23, "y": -4.5, "z": 0.0},
      ...
    ]
"""

import bpy
import json
import sys
import os
from mathutils import Vector

# Objects whose name contains any of these (case-insensitive) are treated
# as ground/non-tree and skipped. Adjust after checking the printed list.
EXCLUDE_NAME_SUBSTRINGS = ["plane", "material", "camera", "light", "cube"]


def get_script_args():
    argv = sys.argv
    if "--" not in argv:
        raise RuntimeError(
            "Usage: blender --background --python extract_tree_layout_blender.py "
            "-- <forest.glb> <output.json>"
        )
    return argv[argv.index("--") + 1:]


def is_tree_candidate(obj):
    if obj.type != 'MESH':
        return False
    name_lower = obj.name.lower()
    if any(sub in name_lower for sub in EXCLUDE_NAME_SUBSTRINGS):
        return False
    return True


def world_bbox_base_center(obj):
    """Returns (x_center, y_center, z_min) of the object's world-space
    bounding box — z_min so the collision cylinder sits on the ground,
    x/y center so it's under the trunk regardless of mesh origin."""
    corners = [obj.matrix_world @ Vector(c) for c in obj.bound_box]
    xs = [c.x for c in corners]
    ys = [c.y for c in corners]
    zs = [c.z for c in corners]
    return (sum(xs) / len(xs), sum(ys) / len(ys), min(zs))


def main():
    args = get_script_args()
    if len(args) < 2:
        raise RuntimeError("Need both <forest.glb> and <output.json> arguments")
    glb_path, output_json = args[0], args[1]

    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)

    print(f"Importing {glb_path} ...")
    bpy.ops.import_scene.gltf(filepath=glb_path)

    all_objects = list(bpy.data.objects)
    print(f"\nTotal objects in file: {len(all_objects)}")
    for o in all_objects:
        print(f"  - {o.name:30s} type={o.type}")

    candidates = [o for o in all_objects if is_tree_candidate(o)]
    print(f"\nTree candidates after filtering ({len(candidates)}):")

    trees = []
    for obj in candidates:
        x, y, z = world_bbox_base_center(obj)
        print(f"  {obj.name:20s}  x={x:7.3f}  y={y:7.3f}  z={z:7.3f}")
        trees.append({"name": obj.name, "x": x, "y": y, "z": z})

    with open(output_json, "w") as f:
        json.dump(trees, f, indent=2)

    print(f"\nWritten: {output_json}  ({len(trees)} tree positions)")
    print("\nIf this count or the excluded/included names look wrong, edit "
          "EXCLUDE_NAME_SUBSTRINGS at the top of this script and re-run.")


if __name__ == "__main__":
    main()
