#!/usr/bin/env python3
"""
Simplify collision meshes in a URDF file.

Reads a URDF, finds all <collision> mesh references, simplifies them using
Open3D (decimation or convex hull), and writes a new URDF with collision
mesh paths pointing to the output directory. Visual meshes are untouched.

Optionally strips collision geometry for meshes matching patterns from a
YAML config file (e.g., motors, fasteners, bearings).

Usage:
    python simplify_collisions.py -i robot.urdf -o robot_collision.urdf
    python simplify_collisions.py -i robot.urdf --convex
    python simplify_collisions.py -i robot.urdf -c configs/strip_collision.yaml
"""

import argparse
import os
import shutil
import struct
import xml.etree.ElementTree as ET

import numpy as np
import open3d as o3d
import yaml


def count_stl_triangles(path: str) -> int:
    """Quick triangle count from binary STL header (no full parse needed)."""
    with open(path, "rb") as f:
        f.read(80)
        return struct.unpack("<I", f.read(4))[0]


def decimate_mesh(input_path: str, output_path: str, ratio: float) -> dict:
    """
    Decimate a mesh using quadric error metric.

    Returns dict with stats: original, final, reduction_pct.
    """
    mesh = o3d.io.read_triangle_mesh(input_path)
    original = len(mesh.triangles)

    if original == 0:
        shutil.copy2(input_path, output_path)
        return {"original": 0, "final": 0, "reduction_pct": 0.0}

    target = max(int(original * ratio), 50)

    simplified = mesh.simplify_quadric_decimation(
        target_number_of_triangles=target
    )

    simplified.compute_vertex_normals()

    o3d.io.write_triangle_mesh(output_path, simplified)
    final = len(simplified.triangles)

    return {
        "original": original,
        "final": final,
        "reduction_pct": (1 - final / original) * 100 if original > 0 else 0,
    }


def convex_hull_mesh(input_path: str, output_path: str) -> dict:
    """
    Compute convex hull of a mesh - ideal for collision geometry.

    Returns dict with stats.
    """
    mesh = o3d.io.read_triangle_mesh(input_path)
    original = len(mesh.triangles)

    if original == 0:
        shutil.copy2(input_path, output_path)
        return {"original": 0, "final": 0, "reduction_pct": 0.0}

    hull, _ = mesh.compute_convex_hull()
    hull.compute_vertex_normals()

    o3d.io.write_triangle_mesh(output_path, hull)
    final = len(hull.triangles)

    return {
        "original": original,
        "final": final,
        "reduction_pct": (1 - final / original) * 100 if original > 0 else 0,
    }


def load_strip_config(config_path: str) -> tuple[bool, list[str]]:
    """Load strip_collision config. Returns (strip_all, patterns)."""
    with open(config_path) as f:
        cfg = yaml.safe_load(f)
    strip_all = cfg.get("strip_all", False)
    patterns = [p.lower() for p in cfg.get("strip_collision", [])]
    return strip_all, patterns


def find_collision_meshes(urdf_path: str) -> list[str]:
    """Extract unique mesh filenames from <collision> elements in URDF."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    meshes = set()
    for link in root.iter("link"):
        for collision in link.findall("collision"):
            for mesh_el in collision.iter("mesh"):
                filename = mesh_el.get("filename", "")
                if filename.startswith("package://"):
                    filename = filename[len("package://"):]
                if filename.endswith(".stl"):
                    meshes.add(filename)
    return sorted(meshes)


def should_strip(filename: str, strip_all: bool, patterns: list[str]) -> bool:
    """Check if a collision element should be stripped entirely."""
    if strip_all:
        return True
    basename = os.path.basename(filename).lower()
    return any(p in basename for p in patterns)


def _package_prefix(dir_path: str) -> str:
    """Strip leading directories before 'meshes/' for package:// paths."""
    idx = dir_path.find("meshes/")
    if idx >= 0:
        return dir_path[idx:]
    return dir_path


def create_output_urdf(
    input_urdf: str,
    output_urdf: str,
    collision_dir: str,
    strip_all: bool,
    patterns: list[str],
) -> tuple[int, int]:
    """
    Rewrite URDF: collision mesh paths point to collision_dir.
    Visual meshes are left untouched.
    Collisions matching strip patterns are removed entirely.

    Returns (stripped_count, rewritten_count).
    """
    tree = ET.parse(input_urdf)
    root = tree.getroot()

    col_prefix = _package_prefix(collision_dir)
    stripped = 0
    rewritten = 0

    for link in root.iter("link"):
        for collision in list(link.findall("collision")):
            mesh_el = collision.find(".//mesh")
            if mesh_el is None:
                continue

            fn = mesh_el.get("filename", "")
            if fn.startswith("package://"):
                fn = fn[len("package://"):]

            if not fn.endswith(".stl"):
                continue

            if should_strip(fn, strip_all, patterns):
                link.remove(collision)
                stripped += 1
            else:
                basename = os.path.basename(fn)
                mesh_el.set("filename", f"package://{col_prefix}{basename}")
                rewritten += 1

    ET.indent(tree, space="  ")
    tree.write(output_urdf, xml_declaration=True, encoding="utf-8")
    return stripped, rewritten


def main():
    parser = argparse.ArgumentParser(
        description="Simplify collision meshes in a URDF using Open3D."
    )
    parser.add_argument(
        "-i", "--input",
        default="urdf/robot_with_limits.urdf",
        help="Input URDF file (default: urdf/robot_with_limits.urdf)",
    )
    parser.add_argument(
        "-o", "--output",
        default=None,
        help="Output URDF file (default: <input>_simplified.urdf)",
    )
    parser.add_argument(
        "-r", "--ratio",
        type=float,
        default=0.1,
        help="Target ratio of triangles to keep (default: 0.1 = 10%%)",
    )
    parser.add_argument(
        "--source-dir",
        default=None,
        help="Directory containing original STL meshes. "
             "If not specified, resolved from URDF mesh paths.",
    )
    parser.add_argument(
        "--collision-dir",
        default="urdf/meshes/collision/",
        help="Output directory for simplified collision meshes "
             "(default: urdf/meshes/collision/)",
    )
    parser.add_argument(
        "--convex",
        action="store_true",
        help="Use convex hulls instead of decimation (recommended for Gazebo).",
    )
    parser.add_argument(
        "--min-triangles",
        type=int,
        default=50000,
        help="Skip decimation for meshes with fewer triangles (default: 50000). "
             "Small meshes are copied as-is.",
    )
    parser.add_argument(
        "-c", "--config",
        default=None,
        help="YAML config with strip_collision patterns "
             "(e.g., configs/strip_collision.yaml). "
             "Matching collisions are removed entirely.",
    )
    args = parser.parse_args()

    # Resolve output path: default to <input>_simplified.urdf in same directory
    if args.output is None:
        base, ext = os.path.splitext(args.input)
        args.output = f"{base}_simplified{ext}"

    # Resolve source directory from URDF if not specified
    urdf_dir = os.path.dirname(os.path.abspath(args.input))
    if args.source_dir is None:
        tree = ET.parse(args.input)
        for mesh_el in tree.getroot().iter("mesh"):
            fn = mesh_el.get("filename", "")
            if fn.startswith("package://"):
                fn = fn[len("package://"):]
            if fn.endswith(".stl"):
                args.source_dir = os.path.join(urdf_dir, os.path.dirname(fn))
                break
        if args.source_dir is None:
            args.source_dir = os.path.join(urdf_dir, "assets")
        print(f"Source directory (from URDF): {args.source_dir}")

    # Load strip config if provided
    strip_all = False
    patterns = []
    if args.config:
        if not os.path.exists(args.config):
            print(f"Warning: config '{args.config}' not found, "
                  "no collisions will be stripped")
        else:
            strip_all, patterns = load_strip_config(args.config)
            if strip_all:
                print("Config: strip ALL collisions")
            else:
                print(f"Config: {len(patterns)} strip patterns "
                      f"from {args.config}")

    # Find collision meshes in URDF
    collision_meshes = find_collision_meshes(args.input)
    print(f"Found {len(collision_meshes)} unique collision STL meshes "
          f"in {args.input}")
    print(f"Reading originals from: {args.source_dir}")
    mode_label = "CONVEX HULL" if args.convex else f"decimate at {args.ratio:.0%}"
    print(f"Collision mode: {mode_label}")
    print()

    # Create output directory
    os.makedirs(args.collision_dir, exist_ok=True)

    # Process each collision mesh
    total_original = 0
    total_final = 0
    processed = 0
    skipped_strip = 0
    skipped_missing = 0

    for mesh_rel in collision_meshes:
        basename = os.path.basename(mesh_rel)

        # Check if this mesh should be stripped
        if should_strip(mesh_rel, strip_all, patterns):
            skipped_strip += 1
            print(f"  STRIP  {basename}")
            continue

        # Find source mesh
        mesh_path = os.path.join(args.source_dir, basename)
        if not os.path.exists(mesh_path):
            skipped_missing += 1
            print(f"  SKIP   {basename} (not found in {args.source_dir})")
            continue

        tri_count = count_stl_triangles(mesh_path)
        col_path = os.path.join(args.collision_dir, basename)

        # Small mesh: copy as-is (unless convex hull requested)
        if tri_count < args.min_triangles and not args.convex:
            if os.path.abspath(mesh_path) != os.path.abspath(col_path):
                shutil.copy2(mesh_path, col_path)
            total_original += tri_count
            total_final += tri_count
            processed += 1
            print(
                f"  {basename:40s}  {tri_count:>8,} tri  "
                f"COPY (under {args.min_triangles:,} threshold)"
            )
            continue

        # Simplify
        if args.convex:
            stats = convex_hull_mesh(mesh_path, col_path)
        else:
            stats = decimate_mesh(mesh_path, col_path, args.ratio)

        total_original += stats["original"]
        total_final += stats["final"]
        processed += 1

        orig_size = os.path.getsize(mesh_path)
        new_size = os.path.getsize(col_path)
        mode = "convex" if args.convex else "decimate"
        print(
            f"  {basename:40s}  {stats['original']:>8,} -> "
            f"{stats['final']:>8,} tri  "
            f"({stats['reduction_pct']:5.1f}% reduction)  "
            f"{orig_size/1024/1024:.1f}MB -> {new_size/1024/1024:.1f}MB"
        )

    # Summary
    print(f"\n{'='*80}")
    print(f"Collision meshes processed: {processed}")
    print(f"Collision meshes stripped:  {skipped_strip}")
    print(f"Meshes not found:          {skipped_missing}")
    if total_original > 0:
        print(f"Total original triangles:   {total_original:>12,}")
        print(f"Total final triangles:      {total_final:>12,}")
        print(f"Overall reduction:          "
              f"{(1 - total_final / total_original) * 100:.1f}%")

    # Create output URDF
    stripped, rewritten = create_output_urdf(
        args.input,
        args.output,
        args.collision_dir,
        strip_all,
        patterns,
    )
    print(f"\nURDF written: {args.output}")
    print(f"  Collision elements stripped: {stripped}")
    print(f"  Collision paths rewritten:  {rewritten}")
    print(f"\nDone! Use '{args.output}' for simulation.")


if __name__ == "__main__":
    main()
