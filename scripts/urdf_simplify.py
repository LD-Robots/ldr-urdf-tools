#!/usr/bin/env python3
"""
URDF Simplifier Script
- Merges sub-link masses into parent links
- Removes duplicate joints and fixed-joint sub-links
- Strips meshes matching patterns from config

Usage:
    python urdf_simplify.py <input_urdf> [--output <output_urdf>]
"""

import xml.etree.ElementTree as ET
import argparse
import sys
import os
import yaml
from collections import defaultdict

from joint_correction import get_desired_axis, axis_str


# ─── URDF parsing ────────────────────────────────────────────────────────────

def parse_urdf(filepath):
    tree = ET.parse(filepath)
    return tree, tree.getroot()


def get_links_info(root):
    links = {}
    for el in root.findall('link'):
        name = el.get('name')
        mass = 0.0
        inertial = el.find('inertial')
        if inertial is not None:
            m = inertial.find('mass')
            if m is not None:
                mass = float(m.get('value', '0'))
        links[name] = {'mass': mass}
    return links


def get_joints_info(root):
    joints = []
    for el in root.findall('joint'):
        origin = el.find('origin')
        axis_el = el.find('axis')
        joints.append({
            'name': el.get('name'),
            'type': el.get('type'),
            'parent': el.find('parent').get('link'),
            'child': el.find('child').get('link'),
            'axis': axis_el.get('xyz') if axis_el is not None else '0 0 1',
            'xyz': origin.get('xyz', '0 0 0') if origin is not None else '0 0 0',
            'rpy': origin.get('rpy', '0 0 0') if origin is not None else '0 0 0',
        })
    return joints


# ─── Analysis ─────────────────────────────────────────────────────────────────

def identify_sub_links(joints):
    """Find duplicate-joint children and fixed-joint children."""
    sub_links = {}
    first_child = {}
    for j in joints:
        if j['type'] == 'fixed':
            sub_links[j['child']] = j['parent']
        elif j['name'] in first_child:
            sub_links[j['child']] = first_child[j['name']]
        else:
            first_child[j['name']] = j['child']
    return sub_links


def resolve_parent(name, sub_links):
    visited = set()
    while name in sub_links:
        if name in visited:
            break
        visited.add(name)
        name = sub_links[name]
    return name


# ─── Main simplification ─────────────────────────────────────────────────────

def simplify_urdf(root, links_info, sub_links, strip_meshes=()):
    """URDF simplification: merge masses, strip meshes, remove sub-links."""

    # --- Merge masses ---
    mass_add = defaultdict(float)
    for sub, _ in sub_links.items():
        target = resolve_parent(sub, sub_links)
        if sub in links_info:
            mass_add[target] += links_info[sub]['mass']

    for link_el in root.findall('link'):
        name = link_el.get('name')
        if name in mass_add:
            inertial = link_el.find('inertial')
            if inertial is not None:
                m = inertial.find('mass')
                if m is not None:
                    old = float(m.get('value', '0'))
                    m.set('value', f'{old + mass_add[name]:.6f}')

    # --- Remove unwanted visuals/collisions based on strip_meshes patterns ---
    if strip_meshes:
        for link_el in root.findall('link'):
            for tag in ('visual', 'collision'):
                for elem in list(link_el.findall(tag)):
                    mesh = elem.find('.//mesh')
                    if mesh is not None:
                        fn = mesh.get('filename', '').lower()
                        if any(k in fn for k in strip_meshes):
                            link_el.remove(elem)

    # --- Remove sub-links and duplicate/fixed joints ---
    for link_el in list(root.findall('link')):
        if link_el.get('name') in sub_links:
            root.remove(link_el)

    seen_names = set()
    for joint_el in list(root.findall('joint')):
        child = joint_el.find('child').get('link')
        name = joint_el.get('name')
        if child in sub_links or name in seen_names:
            root.remove(joint_el)
            continue
        seen_names.add(name)

    return root


# ─── Display ──────────────────────────────────────────────────────────────────

def print_tree(root_link, children, links_info, sub_links, prefix=""):
    mass = links_info.get(root_link, {}).get('mass', 0)
    is_sub = root_link in sub_links
    tag = f"SUB -> '{resolve_parent(root_link, sub_links)}'" if is_sub else "MAIN"
    print(f"{prefix}{root_link}  ({mass:.3f} kg)  [{tag}]")

    for i, (j, child) in enumerate(children.get(root_link, [])):
        last = (i == len(children[root_link]) - 1)
        conn = "└── " if last else "├── "
        npfx = prefix + ("    " if last else "│   ")

        axis_cur = j['axis']
        desired = get_desired_axis(j['name'])
        axis_info = f"axis=[{axis_cur}]"
        if desired:
            desired_str = axis_str(desired)
            if axis_cur != desired_str:
                axis_info += f" -> [{desired_str}]"
            else:
                axis_info += " OK"

        print(f"{prefix}{conn}joint: {j['name']} ({j['type']}) {axis_info}")
        print_tree(child, children, links_info, sub_links, npfx)


# ─── Pretty print XML ────────────────────────────────────────────────────────

def indent_xml(elem, level=0):
    indent = "\n" + "  " * level
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = indent + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = indent
        for child in elem:
            indent_xml(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = indent
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = indent
    if level == 0:
        elem.tail = "\n"


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Simplify URDF: merge links, remove sub-links, strip meshes')
    parser.add_argument('input_urdf', help='Input URDF file path')
    parser.add_argument('--output', '-o', help='Output URDF file (default: <input>_simplified.urdf)')
    parser.add_argument('--config', '-c', default='util/configs/simplify_config.yaml',
                        help='YAML config with strip_meshes list (default: util/configs/simplify_config.yaml)')
    args = parser.parse_args()

    if not os.path.exists(args.input_urdf):
        print(f"Error: '{args.input_urdf}' not found.")
        sys.exit(1)

    # Load strip_meshes from config
    strip_meshes = ()
    if os.path.exists(args.config):
        with open(args.config) as f:
            cfg = yaml.safe_load(f) or {}
        strip_meshes = tuple(cfg.get('strip_meshes', []))
        print(f"Config: {args.config} ({len(strip_meshes)} strip patterns)")
    else:
        print(f"Warning: config '{args.config}' not found, no meshes will be stripped")

    print(f"\n{'='*70}")
    print(f"  URDF Simplifier")
    print(f"{'='*70}")
    print(f"\nParsing: {args.input_urdf}")

    tree, root = parse_urdf(args.input_urdf)
    robot_name = root.get('name', 'unknown')
    print(f"Robot: {robot_name}")

    links_info = get_links_info(root)
    joints_info = get_joints_info(root)
    total_mass = sum(v['mass'] for v in links_info.values())

    print(f"Links: {len(links_info)}, Joints: {len(joints_info)}, Mass: {total_mass:.3f} kg")

    # Build tree for display
    children = defaultdict(list)
    parent_map = {}
    for j in joints_info:
        children[j['parent']].append((j, j['child']))
        parent_map[j['child']] = j['parent']

    root_link = None
    for name in links_info:
        if name not in parent_map:
            root_link = name
            break
    print(f"Root: {root_link}")

    sub_links = identify_sub_links(joints_info)
    main_links = {n for n in links_info if n not in sub_links}

    print(f"\n{'='*70}")
    print(f"  ANALYSIS")
    print(f"{'='*70}")

    print(f"\nMain links ({len(main_links)}):")
    for name in sorted(main_links):
        mass = links_info[name]['mass']
        extra = sum(links_info[s]['mass'] for s in sub_links
                    if resolve_parent(s, sub_links) == name and s in links_info)
        if extra > 0:
            print(f"  {name}  ({mass:.3f} + {extra:.3f} = {mass+extra:.3f} kg)")
        else:
            print(f"  {name}  ({mass:.3f} kg)")

    if sub_links:
        print(f"\nSub-links to remove ({len(sub_links)}):")
        for s in sorted(sub_links):
            t = resolve_parent(s, sub_links)
            m = links_info.get(s, {}).get('mass', 0)
            print(f"  {s}  ({m:.3f} kg) -> '{t}'")

    print(f"\nJoint axis info:")
    seen = set()
    for j in joints_info:
        if j['name'] in seen:
            print(f"  {j['name']}: DUPLICATE (remove)")
            continue
        seen.add(j['name'])
        if j['type'] != 'revolute':
            continue
        desired = get_desired_axis(j['name'])
        if desired:
            desired_str = axis_str(desired)
            if j['axis'] != desired_str:
                print(f"  {j['name']}: [{j['axis']}] (fix with joint_correction.py)")
            else:
                print(f"  {j['name']}: [{j['axis']}] OK")

    print(f"\n{'='*70}")
    print(f"  KINEMATIC TREE")
    print(f"{'='*70}\n")
    print_tree(root_link, children, links_info, sub_links)

    print(f"\n{'='*70}")
    resp = input("\nProceed? (yes/no): ").strip().lower()
    if resp != 'yes':
        print("Aborted.")
        sys.exit(0)

    # Output path
    if args.output:
        out = args.output
    else:
        base, ext = os.path.splitext(args.input_urdf)
        out = f"{base}_simplified{ext}"

    # Re-parse clean copy
    tree2, root2 = parse_urdf(args.input_urdf)

    print(f"\nSimplifying...")
    simplified = simplify_urdf(root2, links_info, sub_links, strip_meshes)

    n_links = len(simplified.findall('link'))
    n_joints = len(simplified.findall('joint'))

    indent_xml(simplified)
    tree2.write(out, xml_declaration=True, encoding='unicode')

    print(f"\nDone!")
    print(f"  Output: {out}")
    print(f"  Links: {len(links_info)} -> {n_links}")
    print(f"  Joints: {len(joints_info)} -> {n_joints}")
    print(f"  Mass preserved: {total_mass:.3f} kg")


if __name__ == '__main__':
    main()
