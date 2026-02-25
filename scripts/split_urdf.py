#!/usr/bin/env python3
"""
Split a URDF into separate xacro files per body section.

Generates xacro files matching the full_robot_description structure:
  urdf/joints/body_joints.xacro    — waist joints + xacro args (fixed_legs, only_left)
  urdf/joints/left_arm_joints.xacro
  urdf/joints/right_arm_joints.xacro
  urdf/joints/left_foot_joints.xacro
  urdf/joints/right_foot_joints.xacro
  urdf/links/body_links.xacro      — base + torso links
  urdf/links/left_arm_links.xacro
  urdf/links/right_arm_links.xacro
  urdf/links/left_foot_links.xacro
  urdf/links/right_foot_links.xacro

Usage:
    python split_urdf.py -i robot_gazebo.urdf -p full_robot_description
    python split_urdf.py -i robot_gazebo.urdf -p full_robot_description --fixed-legs
"""

import argparse
import os
import xml.etree.ElementTree as ET
from collections import OrderedDict


# ─── Section definitions ──────────────────────────────────────────────────────

# Joint name prefix → section
JOINT_SECTIONS = OrderedDict([
    ("Body", ["waist"]),
    ("Left Arm", ["left_shoulder", "left_elbow", "left_wrist"]),
    ("Right Arm", ["right_shoulder", "right_elbow", "right_wrist"]),
    ("Left Foot", ["left_hip", "left_knee", "left_ankle"]),
    ("Right Foot", ["right_hip", "right_knee", "right_ankle"]),
])

# Section → output filename stem
SECTION_FILES = OrderedDict([
    ("Body", "body"),
    ("Left Arm", "left_arm"),
    ("Right Arm", "right_arm"),
    ("Left Foot", "left_foot"),
    ("Right Foot", "right_foot"),
])

# Joints classified as leg joints (for fixed_legs xacro support)
LEG_JOINT_PATTERNS = ["hip", "knee", "ankle"]


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


def is_waist_joint(joint_name: str) -> bool:
    """Check if a joint should use the waist_joint_type xacro variable."""
    return "waist" in joint_name


def compute_link_depth(root: ET.Element) -> dict[str, int]:
    """Compute the depth of each link in the kinematic tree (root = 0)."""
    children_of = {}
    child_set = set()
    for joint in root.iter("joint"):
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        children_of.setdefault(parent, []).append(child)
        child_set.add(child)

    all_links = {link.get("name") for link in root.iter("link")}
    root_links = all_links - child_set

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
    """Classify links into body sections by walking the kinematic tree."""
    link_sections = {}

    joint_map = {}  # child_link -> (joint_name, parent_link)
    for joint in root.iter("joint"):
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        joint_map[child] = (joint.get("name"), parent)

    all_links = {link.get("name") for link in root.iter("link")}
    child_links = set(joint_map.keys())
    root_links = all_links - child_links

    # Root links go to Body
    for rl in root_links:
        link_sections[rl] = "Body"

    # Classify each link based on the joint that introduces it
    for link_name in all_links:
        if link_name in link_sections:
            continue
        if link_name in joint_map:
            joint_name, _ = joint_map[link_name]
            section = classify_joint(joint_name)
            # Child of waist joint is torso → still Body
            if section == "Body":
                link_sections[link_name] = "Body"
            else:
                link_sections[link_name] = section

    return link_sections


def elem_to_string(elem: ET.Element, indent: str = "  ") -> str:
    """Convert an XML element to a nicely indented string."""
    ET.indent(elem, space=indent)
    raw = ET.tostring(elem, encoding="unicode", short_empty_elements=True)
    return raw.rstrip().replace(" />", "/>")


def rewrite_mesh_paths(elem: ET.Element, package_name: str) -> None:
    """Rewrite mesh filename attributes to use package:// with the given package name."""
    for mesh_el in elem.iter("mesh"):
        fn = mesh_el.get("filename", "")
        if fn.startswith("package://"):
            rel = fn[len("package://"):]
            mesh_el.set("filename", f"package://{package_name}/{rel}")


# ─── File generation ──────────────────────────────────────────────────────────

def build_section_xacro(elements: list[str], section_name: str,
                        header_lines: list[str] | None = None) -> str:
    """Build a standalone xacro file for one body section."""
    lines = ['<?xml version="1.0"?>', '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">']

    if header_lines:
        lines.append("")
        for hl in header_lines:
            lines.append(f"  {hl}")

    lines.append("")
    lines.append(f"  <!-- {'=' * 66} -->")
    lines.append(f"  <!-- {section_name:<65s} -->")
    lines.append(f"  <!-- {'=' * 66} -->")

    for i, elem_str in enumerate(elements):
        indented = "\n".join(
            f"  {line}" if line.strip() else ""
            for line in elem_str.split("\n")
        )
        lines.append(indented)
        if i < len(elements) - 1:
            lines.append("")

    lines.append("")
    lines.append("</robot>")
    lines.append("")
    return "\n".join(lines)


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Split a URDF into separate xacro files per body section."
    )
    parser.add_argument(
        "-i", "--input",
        default="urdf/robot_gazebo.urdf",
        help="Input URDF file (default: urdf/robot_gazebo.urdf)",
    )
    parser.add_argument(
        "-p", "--package",
        required=True,
        help="ROS package name (e.g. full_robot_description)",
    )
    parser.add_argument(
        "-o", "--output-dir",
        default="urdf/",
        help="Output directory (default: urdf/). Creates joints/ and links/ subdirs.",
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
        help="Add xacro support for fixed_legs argument (leg/waist joints become fixed)",
    )
    parser.add_argument(
        "--only-left",
        action="store_true",
        default=True,
        help="Add xacro support for only_left argument (waist locked when true, default: true)",
    )
    args = parser.parse_args()

    # Parse input
    tree = ET.parse(args.input)
    root = tree.getroot()

    # Create output directories
    joints_dir = os.path.join(args.output_dir, "joints")
    links_dir = os.path.join(args.output_dir, "links")
    os.makedirs(joints_dir, exist_ok=True)
    os.makedirs(links_dir, exist_ok=True)

    # Classify links and compute kinematic depth
    link_sections = classify_links(root)
    link_depth = compute_link_depth(root)

    # ── Organize joints by section ──
    joints_by_section = OrderedDict((s, []) for s in SECTION_FILES)
    joints_by_section["Other"] = []

    joint_entries = []
    for joint in root.iter("joint"):
        name = joint.get("name")
        section = classify_joint(name)

        # Add dynamics if not present
        if joint.find("dynamics") is None:
            dynamics = ET.SubElement(joint, "dynamics")
            dynamics.set("damping", str(args.damping))
            dynamics.set("friction", str(args.friction))

        # Handle fixed_legs: leg joints use ${leg_joint_type}
        if args.fixed_legs and is_leg_joint(name):
            joint.set("type", "${leg_joint_type}")

        # Handle waist joint type (fixed_legs or only_left)
        if args.fixed_legs and is_waist_joint(name):
            joint.set("type", "${waist_joint_type}")

        child_link = joint.find("child").get("link")
        depth = link_depth.get(child_link, 0)
        elem_str = elem_to_string(joint)
        joint_entries.append((section, depth, elem_str))

    for section, depth, elem_str in sorted(joint_entries, key=lambda x: x[1]):
        if section in joints_by_section:
            joints_by_section[section].append(elem_str)
        else:
            joints_by_section["Other"].append(elem_str)

    # ── Organize links by section ──
    links_by_section = OrderedDict((s, []) for s in SECTION_FILES)
    links_by_section["Other"] = []

    link_entries = []
    for link in root.iter("link"):
        name = link.get("name")
        section = link_sections.get(name, "Other")

        rewrite_mesh_paths(link, args.package)

        depth = link_depth.get(name, 0)
        elem_str = elem_to_string(link)
        link_entries.append((section, depth, elem_str))

    for section, depth, elem_str in sorted(link_entries, key=lambda x: x[1]):
        if section in links_by_section:
            links_by_section[section].append(elem_str)
        else:
            links_by_section["Other"].append(elem_str)

    # ── Write per-section files ──
    active_sections = []  # sections that have content

    for section, stem in SECTION_FILES.items():
        joints = joints_by_section.get(section, [])
        links = links_by_section.get(section, [])

        if not joints and not links:
            continue
        active_sections.append(stem)

        # Write joints file
        if joints:
            header = None
            if section == "Body" and args.fixed_legs:
                header = [
                    f"<!-- {'=' * 66} -->",
                    f"<!-- {'Leg joint type — set fixed_legs:=true to lock leg joints':<65s} -->",
                    f"<!-- {'=' * 66} -->",
                    '<xacro:arg name="fixed_legs" default="false"/>',
                    "<xacro:property name=\"leg_joint_type\" "
                    "value=\"${'fixed' if '$(arg fixed_legs)' == 'true' else 'revolute'}\"/>",
                    "",
                    f"<!-- {'=' * 66} -->",
                    f"<!-- {'only_left — when true, also lock waist yaw joint':<65s} -->",
                    f"<!-- {'=' * 66} -->",
                    '<xacro:arg name="only_left" default="true"/>',
                    "<xacro:property name=\"waist_joint_type\" "
                    "value=\"${'fixed' if '$(arg only_left)' == 'true' or '$(arg fixed_legs)' == 'true' else 'revolute'}\"/>",
                ]

            content = build_section_xacro(joints, section, header)
            path = os.path.join(joints_dir, f"{stem}_joints.xacro")
            with open(path, "w") as f:
                f.write(content)

        # Write links file
        if links:
            content = build_section_xacro(links, section)
            path = os.path.join(links_dir, f"{stem}_links.xacro")
            with open(path, "w") as f:
                f.write(content)

    # Handle "Other" section if present
    other_joints = joints_by_section.get("Other", [])
    other_links = links_by_section.get("Other", [])
    if other_joints:
        content = build_section_xacro(other_joints, "Other")
        path = os.path.join(joints_dir, "other_joints.xacro")
        with open(path, "w") as f:
            f.write(content)
        active_sections.append("other")

    if other_links:
        content = build_section_xacro(other_links, "Other")
        path = os.path.join(links_dir, "other_links.xacro")
        with open(path, "w") as f:
            f.write(content)

    # ── Summary ──
    total_joints = sum(len(v) for v in joints_by_section.values())
    total_links = sum(len(v) for v in links_by_section.values())

    print(f"Split {args.input} into xacro files:\n")
    print(f"  Package: {args.package}\n")

    for section, stem in SECTION_FILES.items():
        joints = joints_by_section.get(section, [])
        links = links_by_section.get(section, [])
        if joints or links:
            j_path = os.path.join(joints_dir, f"{stem}_joints.xacro")
            l_path = os.path.join(links_dir, f"{stem}_links.xacro")
            print(f"  {section}:")
            if joints:
                print(f"    {j_path} ({len(joints)} joints)")
            if links:
                print(f"    {l_path} ({len(links)} links)")

    if other_joints or other_links:
        print(f"\n  Other:")
        if other_joints:
            print(f"    {os.path.join(joints_dir, 'other_joints.xacro')} ({len(other_joints)} joints)")
        if other_links:
            print(f"    {os.path.join(links_dir, 'other_links.xacro')} ({len(other_links)} links)")

    print(f"\n  Total: {total_joints} joints, {total_links} links")
    print(f"  Mesh paths: package://{args.package}/meshes/...")
    if args.fixed_legs:
        print("  Fixed legs: enabled (use arg fixed_legs:=true)")


if __name__ == "__main__":
    main()
