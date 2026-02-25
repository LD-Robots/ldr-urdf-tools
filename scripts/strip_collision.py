#!/usr/bin/env python3
"""
Strip <collision> elements from URDF links by matching mesh filenames.

Removes collision geometry for internal parts (motors, stators, rotors, etc.)
that slow down physics simulation without adding value.

Usage:
    python strip_collision.py -i urdf/robot_gazebo.urdf
    python strip_collision.py -i urdf/robot_gazebo.urdf -o urdf/robot_no_collision.urdf
    python strip_collision.py -i urdf/robot_gazebo.urdf -c configs/strip_collision.yaml
"""

import argparse
import os
import xml.etree.ElementTree as ET

import yaml


def load_patterns(config_path: str) -> list[str]:
    """Load collision strip patterns from YAML config."""
    with open(config_path) as f:
        cfg = yaml.safe_load(f)
    patterns = cfg.get("strip_collision", [])
    return [p.lower() for p in patterns]


def strip_collision(root: ET.Element, patterns: list[str]) -> tuple[int, int]:
    """Remove <collision> elements whose mesh filename matches any pattern.

    Returns (matched_links, removed_collisions).
    """
    matched_links = 0
    removed_collisions = 0

    for link in root.iter("link"):
        link_matched = False
        for col in list(link.findall("collision")):
            mesh = col.find(".//mesh")
            if mesh is None:
                continue
            fn = os.path.basename(mesh.get("filename", "")).lower()
            if any(p in fn for p in patterns):
                link.remove(col)
                removed_collisions += 1
                link_matched = True
        if link_matched:
            matched_links += 1

    return matched_links, removed_collisions


def main():
    parser = argparse.ArgumentParser(
        description="Strip <collision> elements from URDF by matching mesh filenames."
    )
    parser.add_argument(
        "-i", "--input",
        default="urdf/robot_gazebo.urdf",
        help="Input URDF file (default: urdf/robot_gazebo.urdf)",
    )
    parser.add_argument(
        "-o", "--output",
        default=None,
        help="Output URDF file (default: <input>_no_collision.urdf)",
    )
    parser.add_argument(
        "-c", "--config",
        default="configs/strip_collision.yaml",
        help="YAML config with mesh filename patterns (default: configs/strip_collision.yaml)",
    )
    args = parser.parse_args()

    if args.output is None:
        base, ext = os.path.splitext(args.input)
        args.output = f"{base}_no_collision{ext}"

    # Load config
    patterns = load_patterns(args.config)
    print(f"Patterns: {patterns}")

    # Parse and strip
    tree = ET.parse(args.input)
    root = tree.getroot()

    total_collisions = sum(
        len(link.findall("collision")) for link in root.iter("link")
    )
    matched, removed = strip_collision(root, patterns)

    # Write output
    ET.indent(tree, space="  ")
    tree.write(args.output, xml_declaration=True, encoding="utf-8")

    print(f"\nStripped collision from {args.input}:")
    print(f"  Total <collision> elements: {total_collisions}")
    print(f"  Links affected: {matched}")
    print(f"  Removed <collision> elements: {removed}")
    print(f"  Remaining: {total_collisions - removed}")
    print(f"  Output: {args.output}")


if __name__ == "__main__":
    main()
