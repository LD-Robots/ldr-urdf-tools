#!/usr/bin/env python3
"""
URDF Simplifier Script
- Reorients all frames to Z-up, X-forward, Y-left convention
- Fixes joint axes: pitch/knee/elbow -> Y, roll -> X, yaw -> Z
- Joint origin rpy values become small (close to 0) like in G1
- Merges sub-link masses, removes duplicate joints
- If physical axis is inverted vs convention, flips limits to keep positive axes

Usage:
    python simplify_urdf.py <input_urdf> [--output <output_urdf>]
"""

import xml.etree.ElementTree as ET
import argparse
import sys
import os
import math
import yaml
from collections import defaultdict, deque


# ─── 3x3 Rotation Math (no numpy needed) ─────────────────────────────────────

def mm(A, B):
    """3x3 matrix multiply."""
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]

def mv(M, v):
    """3x3 matrix @ 3-vector."""
    return [sum(M[i][j] * v[j] for j in range(3)) for i in range(3)]

def mt(M):
    """Transpose (= inverse for rotation matrices)."""
    return [[M[j][i] for j in range(3)] for i in range(3)]

def eye():
    return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

def rx(a):
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]

def ry(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]

def rz(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]

def rpy_to_mat(r, p, y):
    """URDF: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    return mm(rz(y), mm(ry(p), rx(r)))

def mat_to_rpy(R):
    """Extract roll, pitch, yaw from rotation matrix."""
    sy = math.sqrt(R[0][0]**2 + R[1][0]**2)
    if sy > 1e-6:
        roll = math.atan2(R[2][1], R[2][2])
        pitch = math.atan2(-R[2][0], sy)
        yaw = math.atan2(R[1][0], R[0][0])
    else:
        roll = math.atan2(-R[1][2], R[1][1])
        pitch = math.atan2(-R[2][0], sy)
        yaw = 0.0
    return roll, pitch, yaw

def dot3(a, b):
    return sum(x * y for x, y in zip(a, b))

def norm3(v):
    return math.sqrt(dot3(v, v))

def clean(v, tol=1e-10):
    return [0.0 if abs(x) < tol else x for x in v]

def parse_vec(s):
    return [float(x) for x in s.strip().split()]

def fmt_vec(v):
    return ' '.join(f'{x:.6g}' for x in v)

def fmt_rpy(r, p, y):
    vals = clean([r, p, y], tol=1e-4)  # <0.006° is noise
    return fmt_vec(vals)


# ─── Axis convention ──────────────────────────────────────────────────────────

def get_desired_axis(joint_name):
    """Standard axis based on joint name: pitch/knee/elbow->Y, roll->X, yaw->Z."""
    n = joint_name.lower()
    if 'pitch' in n or 'knee' in n or 'elbow' in n:
        return [0, 1, 0]
    elif 'roll' in n:
        return [1, 0, 0]
    elif 'yaw' in n:
        return [0, 0, 1]
    return None

def axis_str(v):
    """Format axis vector as string like '0 1 0'."""
    return ' '.join(str(int(round(x))) for x in v)


def cross3(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def build_canonical_frame(v_world, desired_idx):
    """Build right-handed frame closest to world frame with v_world as one axis.

    desired_idx: 0=X (roll), 1=Y (pitch), 2=Z (yaw)
    v_world: physical axis direction in world frame (unit vector)

    Returns: R_canon (3x3 columns = child axes in world), flip (bool)
    """
    # Choose axis direction closest to positive world axis
    desired_dir = [0, 0, 0]
    desired_dir[desired_idx] = 1
    flip = dot3(v_world, desired_dir) < 0
    v = [-x for x in v_world] if flip else v_world[:]
    nv = norm3(v)
    if nv > 1e-10:
        v = [x / nv for x in v]

    if desired_idx == 1:  # Pitch: Y = v
        ref = [0, 0, 1]
        z = [ref[i] - dot3(ref, v) * v[i] for i in range(3)]
        nz = norm3(z)
        if nz < 1e-6:
            ref = [1, 0, 0]
            z = [ref[i] - dot3(ref, v) * v[i] for i in range(3)]
            nz = norm3(z)
        z = [x / nz for x in z]
        x = cross3(v, z)
        cols = [x, v, z]
    elif desired_idx == 0:  # Roll: X = v
        ref = [0, 0, 1]
        z = [ref[i] - dot3(ref, v) * v[i] for i in range(3)]
        nz = norm3(z)
        if nz < 1e-6:
            ref = [0, 1, 0]
            z = [ref[i] - dot3(ref, v) * v[i] for i in range(3)]
            nz = norm3(z)
        z = [x / nz for x in z]
        y = cross3(z, v)
        cols = [v, y, z]
    else:  # Yaw: Z = v
        ref = [1, 0, 0]
        x = [ref[i] - dot3(ref, v) * v[i] for i in range(3)]
        nx = norm3(x)
        if nx < 1e-6:
            ref = [0, 1, 0]
            x = [ref[i] - dot3(ref, v) * v[i] for i in range(3)]
            nx = norm3(x)
        x = [xi / nx for xi in x]
        y = cross3(v, x)
        cols = [x, y, v]

    # R_canon: columns = child axes in world frame
    # R[i][j] = cols[j][i]
    R = [[cols[j][i] for j in range(3)] for i in range(3)]
    return R, flip


# ─── Global rotation: old base frame -> Z-up X-forward Y-left ────────────────
#
# From analysis of robot_v1 base frame:
#   X_old = right,  Y_old = forward,  Z_old = up
# Desired:
#   X_new = forward, Y_new = left, Z_new = up
#
# Mapping: X_new = Y_old, Y_new = -X_old, Z_new = Z_old
# This is Rz(-90°) applied to coordinates.
#
R_GLOBAL = rz(-math.pi / 2)  # [[0,1,0],[-1,0,0],[0,0,1]]
R_GLOBAL_INV = mt(R_GLOBAL)


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


def transform_origin(origin_el, R_inv):
    """Apply rotation R_inv to origin element's xyz and rpy."""
    if origin_el is None:
        return
    xyz = parse_vec(origin_el.get('xyz', '0 0 0'))
    rpy = parse_vec(origin_el.get('rpy', '0 0 0'))

    xyz_new = clean(mv(R_inv, xyz))
    R_old = rpy_to_mat(rpy[0], rpy[1], rpy[2])
    R_new = mm(R_inv, R_old)
    r, p, y = mat_to_rpy(R_new)
    rpy_new = clean([r, p, y])

    origin_el.set('xyz', fmt_vec(xyz_new))
    origin_el.set('rpy', fmt_rpy(rpy_new[0], rpy_new[1], rpy_new[2]))


def transform_inertia(inertia_el, R):
    """I_new = R @ I_old @ R^T."""
    if inertia_el is None:
        return
    ixx = float(inertia_el.get('ixx', '0'))
    ixy = float(inertia_el.get('ixy', '0'))
    ixz = float(inertia_el.get('ixz', '0'))
    iyy = float(inertia_el.get('iyy', '0'))
    iyz = float(inertia_el.get('iyz', '0'))
    izz = float(inertia_el.get('izz', '0'))

    I = [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]
    I_new = mm(R, mm(I, mt(R)))

    inertia_el.set('ixx', f'{I_new[0][0]:.6g}')
    inertia_el.set('ixy', f'{I_new[0][1]:.6g}')
    inertia_el.set('ixz', f'{I_new[0][2]:.6g}')
    inertia_el.set('iyy', f'{I_new[1][1]:.6g}')
    inertia_el.set('iyz', f'{I_new[1][2]:.6g}')
    inertia_el.set('izz', f'{I_new[2][2]:.6g}')


def transform_link(link_el, R_transform):
    """Apply R_transform to all elements in a link."""
    inertial = link_el.find('inertial')
    if inertial is not None:
        origin = inertial.find('origin')
        if origin is not None:
            xyz = parse_vec(origin.get('xyz', '0 0 0'))
            xyz_new = clean(mv(R_transform, xyz))
            origin.set('xyz', fmt_vec(xyz_new))
        transform_inertia(inertial.find('inertia'), R_transform)

    for tag in ('visual', 'collision'):
        for elem in link_el.findall(tag):
            origin = elem.find('origin')
            if origin is not None:
                transform_origin(origin, R_transform)


# ─── Main simplification ─────────────────────────────────────────────────────

def simplify_urdf(root, links_info, sub_links, strip_meshes=()):
    """Full URDF simplification: merge, remove, reorient.

    Two-pass approach:
      Pass 1 - Compute world-frame rotation for every link in the original URDF
      Pass 2 - Build canonical (world-aligned) frames for each link independently,
               compute new joint rpy/xyz, transform link elements

    This avoids twist accumulation through the kinematic chain because each
    link's canonical frame is computed directly from its world-frame axis,
    not propagated from parent.
    """

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

    # --- Build tree ---
    link_els = {}
    for el in root.findall('link'):
        link_els[el.get('name')] = el

    children_map = defaultdict(list)
    child_to_joint = {}
    for el in root.findall('joint'):
        p = el.find('parent').get('link')
        c = el.find('child').get('link')
        children_map[p].append((el, c))
        child_to_joint[c] = el

    root_link = None
    for name in link_els:
        if name not in child_to_joint:
            root_link = name
            break

    # ── Pass 1: world-frame rotations in original URDF ──
    # R_world_old[link] maps: v_world = R_world_old[link] @ v_link
    R_world_old = {root_link: eye()}

    queue = deque([root_link])
    while queue:
        parent = queue.popleft()
        for joint_el, child in children_map.get(parent, []):
            origin_el = joint_el.find('origin')
            rpy_old = parse_vec(origin_el.get('rpy', '0 0 0'))
            R_joint = rpy_to_mat(rpy_old[0], rpy_old[1], rpy_old[2])
            R_world_old[child] = mm(R_world_old[parent], R_joint)
            queue.append(child)

    # ── Pass 2: canonical frames & update ──
    # R_canon[link] maps: v_new_world = R_canon[link] @ v_new_child
    # For root: canonical frame = new world frame = identity
    R_canon = {root_link: eye()}

    # Transform root link elements: old root frame → new world frame
    transform_link(link_els[root_link], R_GLOBAL)

    queue = deque([root_link])
    while queue:
        parent_name = queue.popleft()

        for joint_el, child_name in children_map.get(parent_name, []):
            joint_name = joint_el.get('name')
            joint_type = joint_el.get('type')
            origin_el = joint_el.find('origin')
            xyz_old = parse_vec(origin_el.get('xyz', '0 0 0'))

            # Joint xyz in new parent frame
            xyz_new_world = mv(R_GLOBAL, mv(R_world_old[parent_name], xyz_old))
            xyz_new = clean(mv(mt(R_canon[parent_name]), xyz_new_world))

            if joint_type == 'revolute':
                # Physical axis in new world frame
                axis_el = joint_el.find('axis')
                axis_local = parse_vec(axis_el.get('xyz', '0 0 1'))
                v_world_old = mv(R_world_old[child_name], axis_local)
                v_world_new = mv(R_GLOBAL, v_world_old)
                nv = norm3(v_world_new)
                if nv > 1e-10:
                    v_world_new = [x / nv for x in v_world_new]

                # Desired axis from joint name
                a_desired = get_desired_axis(joint_name)
                if a_desired is not None:
                    desired_idx = a_desired.index(1)
                    # If physical axis is far from name-based axis (>45°),
                    # fall back to closest standard axis (handles CAD asymmetry)
                    if abs(dot3(v_world_new, a_desired)) < 0.5:
                        max_idx = max(range(3), key=lambda i: abs(v_world_new[i]))
                        desired_idx = max_idx
                        a_desired = [0, 0, 0]
                        a_desired[desired_idx] = 1
                else:
                    max_idx = max(range(3), key=lambda i: abs(v_world_new[i]))
                    desired_idx = max_idx
                    a_desired = [0, 0, 0]
                    a_desired[desired_idx] = 1

                # Build canonical child frame (world-aligned, axis as one column)
                R_child_canon, flip = build_canonical_frame(v_world_new, desired_idx)
                R_canon[child_name] = R_child_canon

                # Joint rpy: rotation from new parent frame to new child frame
                R_joint_new = mm(mt(R_canon[parent_name]), R_child_canon)
                r, p, y = mat_to_rpy(R_joint_new)
                rpy_new = clean([r, p, y])

                # Update joint origin
                origin_el.set('xyz', fmt_vec(xyz_new))
                origin_el.set('rpy', fmt_rpy(rpy_new[0], rpy_new[1], rpy_new[2]))

                # Update axis (always positive standard axis)
                if axis_el is None:
                    axis_el = ET.SubElement(joint_el, 'axis')
                axis_el.set('xyz', axis_str(a_desired))

                # Flip limits if physical axis opposes desired direction
                if flip:
                    limit_el = joint_el.find('limit')
                    if limit_el is not None:
                        lo = float(limit_el.get('lower', '0'))
                        hi = float(limit_el.get('upper', '0'))
                        limit_el.set('lower', f'{-hi:.6g}')
                        limit_el.set('upper', f'{-lo:.6g}')

            else:
                # Non-revolute: child frame = parent frame (rpy=0)
                R_canon[child_name] = R_canon[parent_name]
                origin_el.set('xyz', fmt_vec(xyz_new))
                origin_el.set('rpy', '0 0 0')

            # Transform child link elements: old child frame → new canonical frame
            R_link_xform = mm(mt(R_canon[child_name]),
                              mm(R_GLOBAL, R_world_old[child_name]))
            if child_name in link_els:
                transform_link(link_els[child_name], R_link_xform)

            queue.append(child_name)

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
    parser = argparse.ArgumentParser(description='Simplify URDF: reorient frames, fix axes, merge links')
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
    print(f"  URDF Simplifier  (Z up, X forward, Y left)")
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

    print(f"\nJoint axis fixes:")
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
                print(f"  {j['name']}: [{j['axis']}] -> [{desired_str}] + frame reoriented")
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

    # Show result summary
    print(f"\nResult joint origins:")
    for j in simplified.findall('joint'):
        if j.get('type') != 'revolute':
            continue
        name = j.get('name')
        origin = j.find('origin')
        axis_el = j.find('axis')
        xyz = origin.get('xyz', '') if origin is not None else ''
        rpy = origin.get('rpy', '') if origin is not None else ''
        ax = axis_el.get('xyz', '') if axis_el is not None else ''
        print(f"  {name:40s} axis=[{ax:7s}] rpy=[{rpy}]")

    print(f"\nDone!")
    print(f"  Output: {out}")
    print(f"  Links: {len(links_info)} -> {n_links}")
    print(f"  Joints: {len(joints_info)} -> {n_joints}")
    print(f"  Mass preserved: {total_mass:.3f} kg")


if __name__ == '__main__':
    main()
