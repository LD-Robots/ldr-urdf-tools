#!/usr/bin/env python3
"""Export a simplified robot package with separate visual/collision meshes.

Creates a new ldr-harambe-N directory containing:
  meshes/
    visual/     <- STLs simplified to 15%
    collision/  <- STLs simplified to 10%
  robot_harambe.urdf  <- patched to reference the new mesh paths
"""

import glob
import json
import os
import re
import shutil

import pymeshlab

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
SOURCE_DIR = os.path.join(ROOT_DIR, "ldr-harambe")

VISUAL_RATIO = 0.15
COLLISION_RATIO = 0.10


def load_config():
    config_path = os.path.join(SOURCE_DIR, "config.json")
    with open(config_path, "r") as f:
        raw = re.sub(r"//.*", "", f.read())
    return json.loads(raw)


def next_output_dir():
    """Find the next ldr-harambe-N directory number."""
    existing = glob.glob(os.path.join(ROOT_DIR, "ldr-harambe-[0-9]*"))
    nums = []
    for d in existing:
        m = re.search(r"ldr-harambe-(\d+)$", d)
        if m:
            nums.append(int(m.group(1)))
    n = max(nums) + 1 if nums else 1
    return os.path.join(ROOT_DIR, f"ldr-harambe-{n}")


def simplify_stl(src_path: str, dst_path: str, target_ratio: float) -> dict:
    """Decimate a single STL to *target_ratio* of its original face count."""
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(src_path)
    original_faces = ms.current_mesh().face_number()

    if original_faces == 0:
        shutil.copy2(src_path, dst_path)
        return {"file": os.path.basename(src_path), "original": 0, "simplified": 0}

    ms.meshing_decimation_quadric_edge_collapse(
        targetfacenum=max(int(original_faces * target_ratio), 4),
        preservetopology=True,
    )
    simplified_faces = ms.current_mesh().face_number()
    ms.save_current_mesh(dst_path, binary=True)
    return {
        "file": os.path.basename(src_path),
        "original": original_faces,
        "simplified": simplified_faces,
    }


def simplify_batch(stl_files, src_dir, dst_dir, ratio, label):
    """Simplify a list of STL files into dst_dir at the given ratio."""
    os.makedirs(dst_dir, exist_ok=True)
    print(f"\n{label} ({ratio:.0%}) → {os.path.relpath(dst_dir, ROOT_DIR)}/")
    for stl in stl_files:
        src = os.path.join(src_dir, stl)
        dst = os.path.join(dst_dir, stl)
        info = simplify_stl(src, dst, ratio)
        print(
            f"  {info['file']:50s}  {info['original']:>6d} → {info['simplified']:>6d} faces"
        )


def patch_urdf(urdf_src: str, urdf_dst: str, old_assets: str):
    """Copy URDF, rewriting visual meshes to meshes/visual/ and collision to meshes/collision/."""
    with open(urdf_src, "r") as f:
        content = f.read()

    visual_prefix = "meshes/visual/"
    collision_prefix = "meshes/collision/"

    def replace_visual(match):
        block = match.group(0)
        return block.replace(
            f'filename="package://{old_assets}/',
            f'filename="{visual_prefix}',
        ).replace(
            f'filename="{old_assets}/',
            f'filename="{visual_prefix}',
        )

    def replace_collision(match):
        block = match.group(0)
        return block.replace(
            f'filename="package://{old_assets}/',
            f'filename="{collision_prefix}',
        ).replace(
            f'filename="{old_assets}/',
            f'filename="{collision_prefix}',
        )

    content = re.sub(r"<visual>.*?</visual>", replace_visual, content, flags=re.DOTALL)
    content = re.sub(
        r"<collision>.*?</collision>", replace_collision, content, flags=re.DOTALL
    )

    with open(urdf_dst, "w") as f:
        f.write(content)


def main():
    config = load_config()
    assets_dir = config.get("assets_directory", "assets")
    output_filename = config.get("output_filename", "robot_harambe")

    assets_path = os.path.join(SOURCE_DIR, assets_dir)
    stl_files = sorted(f for f in os.listdir(assets_path) if f.endswith(".stl"))

    out_dir = next_output_dir()
    visual_dir = os.path.join(out_dir, "meshes", "visual")
    collision_dir = os.path.join(out_dir, "meshes", "collision")

    print(f"Source:  ldr-harambe/{assets_dir}/  ({len(stl_files)} STL files)")
    print(f"Output:  {os.path.relpath(out_dir, ROOT_DIR)}/")

    simplify_batch(stl_files, assets_path, visual_dir, VISUAL_RATIO, "Visual meshes")
    simplify_batch(stl_files, assets_path, collision_dir, COLLISION_RATIO, "Collision meshes")

    urdf_src = os.path.join(SOURCE_DIR, f"{output_filename}.urdf")
    urdf_dst = os.path.join(out_dir, f"{output_filename}.urdf")
    patch_urdf(urdf_src, urdf_dst, assets_dir)
    print(f"\nPatched URDF → {os.path.relpath(urdf_dst, ROOT_DIR)}")

    print(f"\nDone. Output in {os.path.relpath(out_dir, ROOT_DIR)}/")


if __name__ == "__main__":
    main()
