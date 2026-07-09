#!/usr/bin/env python3
"""
merge_trees_blender.py

Run INSIDE Blender (headless) to bake all tree instances of each species
into a single merged static mesh, using the exact x/y/z/scale/yaw values
generate_world.py already computed and wrote to trees_layout.json.

This turns 60 spruce model instances into 1 draw call, and 60 fir
instances into 1 draw call. Collisions stay as individual cylinders in
generate_world.py — this script only touches visuals.

Usage:
    1. First run generate_world.py once to produce trees_layout.json
       (it will warn that the merged GLBs don't exist yet — that's expected).
    2. Then run:
         blender --background --python merge_trees_blender.py -- trees_layout.json
    3. Re-run generate_world.py to emit the world file referencing the
       merged GLBs.

Requires Blender 3.x+ with the built-in glTF importer/exporter (bundled
by default — no addon install needed).
"""

import bpy
import json
import sys
import os
import math
import mathutils


def get_script_args():
    argv = sys.argv
    if "--" not in argv:
        raise RuntimeError(
            "No layout JSON path given. Run as:\n"
            "  blender --background --python merge_trees_blender.py -- trees_layout.json"
        )
    return argv[argv.index("--") + 1:]


def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    # purge orphan mesh/material data left over between species passes
    for block in list(bpy.data.meshes):
        if block.users == 0:
            bpy.data.meshes.remove(block)
    for block in list(bpy.data.materials):
        if block.users == 0:
            bpy.data.materials.remove(block)


def import_and_join_template(glb_path):
    """Import a (possibly multi-object) GLB and join everything into one
    mesh object we can use as a duplication template."""
    if not os.path.exists(glb_path):
        raise FileNotFoundError(f"Base tree mesh not found: {glb_path}")

    before = set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=glb_path)
    imported = [o for o in bpy.data.objects if o not in before]

    mesh_objs = [o for o in imported if o.type == 'MESH']
    if not mesh_objs:
        raise RuntimeError(f"No mesh objects found after importing {glb_path}")

    if len(mesh_objs) > 1:
        bpy.ops.object.select_all(action='DESELECT')
        for o in mesh_objs:
            o.select_set(True)
        bpy.context.view_layer.objects.active = mesh_objs[0]
        bpy.ops.object.join()

    template = bpy.context.view_layer.objects.active
    # zero out any transform baked in from the import itself
    template.location = (0, 0, 0)
    template.rotation_euler = (0, 0, 0)
    template.scale = (1, 1, 1)

    # remove any leftover empties from the import (armatures/nulls etc.)
    for o in list(bpy.data.objects):
        if o.type != 'MESH':
            bpy.data.objects.remove(o, do_unlink=True)

    return template


def build_merged_species(species_key, species_data):
    print(f"\n=== Building merged mesh for '{species_key}' "
          f"({len(species_data['instances'])} instances) ===")

    clear_scene()
    template = import_and_join_template(species_data["base_mesh_path"])

    duplicates = []
    for inst in species_data["instances"]:
        dup = template.copy()
        dup.data = template.data.copy()  # independent mesh data per instance
        bpy.context.collection.objects.link(dup)

        s = inst["scale"]
        yaw = inst.get("yaw", 0.0)

        # Set matrix_world directly rather than location/rotation_euler/scale.
        # In background mode, those three properties only update matrix_basis;
        # matrix_world isn't recomputed until the dependency graph runs, and
        # nothing ticks it here. bpy.ops.object.join() reads matrix_world, so
        # without this fix every duplicate joins at the template's original
        # (0,0,0) transform instead of its assigned position — which is
        # exactly the "all trees piled in one spot" bug.
        mat_loc   = mathutils.Matrix.Translation((inst["x"], inst["y"], inst["z"]))
        mat_rot   = mathutils.Matrix.Rotation(yaw, 4, 'Z')
        mat_scale = mathutils.Matrix.Diagonal((s, s, s, 1.0))
        dup.matrix_world = mat_loc @ mat_rot @ mat_scale

        duplicates.append(dup)

    # remove the original template so it's not duplicated at the origin
    bpy.data.objects.remove(template, do_unlink=True)

    # force a full dependency graph refresh before join, as a second safety
    # net on top of the direct matrix_world assignment above
    bpy.context.view_layer.update()

    # sanity check: positions should be spread across the full instance
    # range, not clustered at one point. If min≈max here, the transform
    # bug is back — stop and investigate before spending time on export.
    xs = [d.matrix_world.translation.x for d in duplicates]
    ys = [d.matrix_world.translation.y for d in duplicates]
    print(f"  Position spread check — x: [{min(xs):.2f}, {max(xs):.2f}]  "
          f"y: [{min(ys):.2f}, {max(ys):.2f}]  (should span most of your "
          f"X_MIN/X_MAX, Y_MIN/Y_MAX range, not be clustered near 0,0)")

    # join all instance duplicates into one final mesh
    bpy.ops.object.select_all(action='DESELECT')
    for d in duplicates:
        d.select_set(True)
    bpy.context.view_layer.objects.active = duplicates[0]
    bpy.ops.object.join()

    merged = bpy.context.view_layer.objects.active
    merged.name = f"forest_{species_key}_merged"

    out_path = species_data["merged_output"]
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    bpy.ops.object.select_all(action='DESELECT')
    merged.select_set(True)
    bpy.context.view_layer.objects.active = merged

    bpy.ops.export_scene.gltf(
        filepath=out_path,
        export_format='GLB',
        use_selection=True,
        export_apply=True,
    )
    print(f"Written: {out_path}")


def main():
    args = get_script_args()
    layout_path = args[0]

    with open(layout_path) as f:
        layout = json.load(f)

    print(f"Loaded layout (seed={layout.get('random_seed')}) from {layout_path}")

    for species_key in ("spruce", "fir"):
        if species_key in layout:
            build_merged_species(species_key, layout[species_key])

    print("\nDone. Re-run generate_world.py to emit the world file "
          "referencing the merged meshes.")


if __name__ == "__main__":
    main()
