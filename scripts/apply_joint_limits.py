#!/usr/bin/env python3
"""Apply joint limits from a YAML config to a URDF file.

Usage:
    python apply_joint_limits.py                          # uses defaults
    python apply_joint_limits.py -c my_limits.yaml        # custom config
    python apply_joint_limits.py -i in.urdf -o out.urdf   # custom I/O
"""

import argparse
import xml.etree.ElementTree as ET

import yaml


def apply_limits(input_urdf: str, config_path: str, output_urdf: str) -> None:
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    joint_limits = config.get("joints", {})

    tree = ET.parse(input_urdf)
    root = tree.getroot()

    applied = []
    missing = []

    for joint in root.iter("joint"):
        name = joint.get("name")
        if name not in joint_limits:
            continue

        limits = joint_limits[name]
        limit_elem = joint.find("limit")

        if limit_elem is None:
            limit_elem = ET.SubElement(joint, "limit")

        limit_elem.set("lower", str(limits["lower"]))
        limit_elem.set("upper", str(limits["upper"]))
        limit_elem.set("effort", str(limits.get("effort", 10)))
        limit_elem.set("velocity", str(limits.get("velocity", 10)))
        applied.append(name)

    for name in joint_limits:
        if name not in applied:
            missing.append(name)

    ET.indent(tree, space="  ")
    tree.write(output_urdf, xml_declaration=True, encoding="utf-8")

    print(f"Applied limits to {len(applied)} joints -> {output_urdf}")
    for name in applied:
        lim = joint_limits[name]
        print(f"  {name}: [{lim['lower']}, {lim['upper']}]")

    if missing:
        print(f"\nWARNING: {len(missing)} joints in config not found in URDF:")
        for name in missing:
            print(f"  - {name}")


def main():
    parser = argparse.ArgumentParser(description="Apply joint limits from YAML config to URDF")
    parser.add_argument("-i", "--input", default="urdf/robot_simplified.urdf", help="Input URDF file")
    parser.add_argument("-c", "--config", default="util/configs/joint_limits.yaml", help="YAML config file")
    parser.add_argument("-o", "--output", default="urdf/robot_with_limits.urdf", help="Output URDF file")
    args = parser.parse_args()

    apply_limits(args.input, args.config, args.output)


if __name__ == "__main__":
    main()
