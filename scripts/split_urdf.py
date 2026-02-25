#!/usr/bin/env python3
"""
Split a URDF into separate xacro files for joints and links.

Generates xacro files organized by body section (Base, Torso, Left Arm,
Right Arm, Left Leg, Right Leg), matching the format used by ROS2
robot_description packages.

Usage:
    python split_urdf.py -i robot_gazebo.urdf -p my_robot_description
    python split_urdf.py -i robot_gazebo.urdf -p my_robot_description -o urdf/
    python split_urdf.py -i robot_gazebo.urdf -p my_robot_description --damping 0.5 --friction 0.1
"""

import argparse
import os
import re
import xml.etree.ElementTree as ET
from collections import OrderedDict


# Body section classification based on joint name patterns
JOINT_SECTIONS = OrderedDict([
    ("Waist", ["waist"]),
    ("Left Arm", ["left_shoulder", "left_elbow", "left_wrist"]),
    ("Right Arm", ["right_shoulder", "right_elbow", "right_wrist"]),
    ("Left Leg", ["left_hip", "left_knee", "left_ankle"]),
    ("Right Leg", ["right_hip", "right_knee", "right_ankle"]),
])

# Joints classified as leg joints (for fixed_legs xacro support)
LEG_JOINT_PATTERNS = [
    "hip", "knee", "ankle", "waist",
]


def classify_joint(joint_name: str) -> str:
    """Classify a joint into a body section based on its name."""
    for section, patterns in JOINT_SECTIONS.items():
        for pattern in patterns:
            if joint_name.startswith(pattern):
                return section
    return "Other"


def is_leg_joint(joint_name: str) -> bool:
    """Check if a joint should use the leg_joint_type xacro variable."""
    for pattern in LEG_JOINT_PATTERNS:
        if pattern in joint_name:
            return True
    return False


def compute_link_depth(root: ET.Element) -> dict[str, int]:
    """Compute the depth of each link in the kinematic tree (root = 0)."""
    # Build parent->children map
    children_of = {}  # parent_link -> [child_link, ...]
    child_set = set()
    for joint in root.iter("joint"):
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        children_of.setdefault(parent, []).append(child)
        child_set.add(child)

    # Find root links
    all_links = {link.get("name") for link in root.iter("link")}
    root_links = all_links - child_set

    # BFS to compute depth
    depth = {}
    queue = list(root_links)
    for rl in root_links:
        depth[rl] = 0
    while queue:
        link = queue.pop(0)
        for ch in children_of.get(link, []):
            depth[ch] = depth[link] + 1
            queue.append(ch)

    return depth


def classify_links(root: ET.Element) -> dict[str, str]:
    """
    Classify links into body sections by walking the kinematic tree.

    Uses joint parent/child relationships to assign each link to the
    section of the joint that introduces it.
    """
    link_sections = {}

    # Build parent->children map from joints
    joint_map = {}  # child_link -> (joint_name, parent_link)
    for joint in root.iter("joint"):
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        joint_map[child] = (joint.get("name"), parent)

    # Find root link (not a child of any joint)
    all_links = {link.get("name") for link in root.iter("link")}
    child_links = set(joint_map.keys())
    root_links = all_links - child_links

    for rl in root_links:
        link_sections[rl] = "Base"

    # Classify each link based on the joint that introduces it
    for link_name in all_links:
        if link_name in link_sections:
            continue
        if link_name in joint_map:
            joint_name, _ = joint_map[link_name]
            section = classify_joint(joint_name)
            # Torso is the child of waist joint
            if section == "Waist":
                link_sections[link_name] = "Torso"
            else:
                link_sections[link_name] = section

    return link_sections


def elem_to_string(elem: ET.Element, indent: str = "  ") -> str:
    """Convert an XML element to a nicely indented string."""
    ET.indent(elem, space=indent)
    raw = ET.tostring(elem, encoding="unicode", short_empty_elements=True)
    # Remove trailing newline and fix " />" -> "/>"
    return raw.rstrip().replace(" />", "/>")


def rewrite_mesh_paths(elem: ET.Element, package_name: str) -> None:
    """Rewrite mesh filename attributes to use package:// with the given package name."""
    for mesh_el in elem.iter("mesh"):
        fn = mesh_el.get("filename", "")
        if fn.startswith("package://"):
            # Extract relative path after package://
            rel = fn[len("package://"):]
            # Replace with package name prefix
            mesh_el.set("filename", f"package://{package_name}/{rel}")


def build_xacro(
    sections: OrderedDict[str, list[str]],
    header_lines: list[str] | None = None,
) -> str:
    """Build a complete xacro file from ordered sections of XML strings."""
    lines = ['<?xml version="1.0"?>', '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">']

    if header_lines:
        lines.append("")
        for hl in header_lines:
            lines.append(f"  {hl}")

    for section_name, elements in sections.items():
        if not elements:
            continue
        lines.append("")
        lines.append(f"  <!-- {'=' * 66} -->")
        lines.append(f"  <!-- {section_name:<65s} -->")
        lines.append(f"  <!-- {'=' * 66} -->")
        for i, elem_str in enumerate(elements):
            # Indent each line by 2 spaces
            indented = "\n".join(
                f"  {line}" if line.strip() else ""
                for line in elem_str.split("\n")
            )
            lines.append(indented)
            # Empty line between elements, but not after the last one
            if i < len(elements) - 1:
                lines.append("")

    lines.append("")
    lines.append("</robot>")
    lines.append("")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description="Split a URDF into separate xacro files for joints and links."
    )
    parser.add_argument(
        "-i", "--input",
        default="urdf/robot_gazebo.urdf",
        help="Input URDF file (default: urdf/robot_gazebo.urdf)",
    )
    parser.add_argument(
        "-p", "--package",
        required=True,
        help="ROS package name for mesh paths (e.g. my_robot_description)",
    )
    parser.add_argument(
        "-o", "--output-dir",
        default="urdf/",
        help="Output directory (default: urdf/). Creates joints/ and links/ subdirs.",
    )
    parser.add_argument(
        "-n", "--name",
        default=None,
        help="Base name for output files (default: derived from input filename). "
             "Produces <name>_joints.xacro and <name>_links.xacro.",
    )
    parser.add_argument(
        "--damping",
        type=float,
        default=0.5,
        help="Joint damping value (default: 0.5)",
    )
    parser.add_argument(
        "--friction",
        type=float,
        default=0.1,
        help="Joint friction value (default: 0.1)",
    )
    parser.add_argument(
        "--fixed-legs",
        action="store_true",
        help="Add xacro support for fixed_legs argument (leg joints become fixed when set)",
    )
    args = parser.parse_args()

    # Parse input
    tree = ET.parse(args.input)
    root = tree.getroot()

    # Derive base name
    base_name = args.name or os.path.splitext(os.path.basename(args.input))[0]
    base_name = base_name.replace("robot_", "").replace("_urdf", "")

    # Create output directories (relative to CWD)
    joints_dir = os.path.join(args.output_dir, "joints")
    links_dir = os.path.join(args.output_dir, "links")
    os.makedirs(joints_dir, exist_ok=True)
    os.makedirs(links_dir, exist_ok=True)

    # Classify links and compute kinematic depth
    link_sections = classify_links(root)
    link_depth = compute_link_depth(root)

    # Organize joints by section, sorted by kinematic chain (proximal → distal)
    section_order = ["Waist", "Left Arm", "Right Arm", "Left Leg", "Right Leg"]
    joints_by_section = OrderedDict((s, []) for s in section_order)
    joints_by_section["Other"] = []

    joint_entries = []  # (section, depth, elem_str)
    for joint in root.iter("joint"):
        name = joint.get("name")
        section = classify_joint(name)

        # Add dynamics if not present
        if joint.find("dynamics") is None:
            dynamics = ET.SubElement(joint, "dynamics")
            dynamics.set("damping", str(args.damping))
            dynamics.set("friction", str(args.friction))

        # Handle fixed_legs xacro variable
        if args.fixed_legs and is_leg_joint(name):
            joint.set("type", "${leg_joint_type}")

        child_link = joint.find("child").get("link")
        depth = link_depth.get(child_link, 0)
        elem_str = elem_to_string(joint)
        joint_entries.append((section, depth, elem_str))

    # Sort by depth within each section
    for section, depth, elem_str in sorted(joint_entries, key=lambda x: x[1]):
        joints_by_section[section].append(elem_str)

    # Organize links by section
    links_by_section = OrderedDict([
        ("Base", []),
        ("Torso", []),
        ("Left Arm", []),
        ("Right Arm", []),
        ("Left Leg", []),
        ("Right Leg", []),
        ("Other", []),
    ])

    link_entries = []  # (section, depth, elem_str)
    for link in root.iter("link"):
        name = link.get("name")
        section = link_sections.get(name, "Other")

        # Rewrite mesh paths
        rewrite_mesh_paths(link, args.package)

        depth = link_depth.get(name, 0)
        elem_str = elem_to_string(link)
        link_entries.append((section, depth, elem_str))

    # Sort by depth within each section
    for section, depth, elem_str in sorted(link_entries, key=lambda x: x[1]):
        links_by_section[section].append(elem_str)

    # Build joints xacro
    joints_header = []
    if args.fixed_legs:
        joints_header = [
            f"<!-- {'=' * 66} -->",
            f"<!-- {'Legs — set fixed_legs:=false for revolute leg joints':<65s} -->",
            f"<!-- {'=' * 66} -->",
            '<xacro:property name="leg_joint_type" '
            "value=\"${'fixed' if '$(arg fixed_legs)' == 'true' else 'revolute'}\"/>",
        ]

    joints_content = build_xacro(joints_by_section, joints_header or None)
    links_content = build_xacro(links_by_section)

    # Write output files
    joints_path = os.path.join(joints_dir, f"{base_name}_joints.xacro")
    links_path = os.path.join(links_dir, f"{base_name}_links.xacro")

    with open(joints_path, "w") as f:
        f.write(joints_content)

    with open(links_path, "w") as f:
        f.write(links_content)

    # Summary
    joint_count = sum(len(v) for v in joints_by_section.values())
    link_count = sum(len(v) for v in links_by_section.values())

    print(f"Split {args.input} into xacro files:\n")
    print(f"  Joints: {joints_path} ({joint_count} joints)")
    for section, joints in joints_by_section.items():
        if joints:
            print(f"    {section}: {len(joints)}")

    print(f"\n  Links:  {links_path} ({link_count} links)")
    for section, links in links_by_section.items():
        if links:
            print(f"    {section}: {len(links)}")

    print(f"\n  Package: {args.package}")
    print(f"  Mesh paths: package://{args.package}/meshes/visual|collision/*.stl")
    if args.fixed_legs:
        print("  Fixed legs: enabled (use arg fixed_legs:=true)")


if __name__ == "__main__":
    main()
