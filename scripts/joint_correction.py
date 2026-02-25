#!/usr/bin/env python3
"""
Joint Correction Script
- Reorients all frames to Z-up, X-forward, Y-left convention
- Fixes joint axes: pitch/knee/elbow -> Y, roll -> X, yaw -> Z
- Joint origin rpy values become small (close to 0)
- If physical axis is inverted vs convention, flips limits to keep positive axes

Usage:
    python joint_correction.py <input_urdf> [--output <output_urdf>]
"""

import xml.etree.ElementTree as ET
import argparse
import sys
import os
import math
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

def cross3(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

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


# ─── Canonical frame construction ─────────────────────────────────────────────

def build_canonical_frame(v_world, desired_idx):
    """Build right-handed frame closest to world frame with v_world as one axis.

    desired_idx: 0=X (roll), 1=Y (pitch), 2=Z (yaw)
    v_world: physical axis direction in world frame (unit vector)

    Returns: R_canon (3x3 columns = child axes in world), flip (bool)
    """
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


# ─── Transform utilities ─────────────────────────────────────────────────────

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


# ─── URDF parsing ────────────────────────────────────────────────────────────

def parse_urdf(filepath):
    tree = ET.parse(filepath)
    return tree, tree.getroot()


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


# ─── Joint correction ────────────────────────────────────────────────────────

def correct_joints(root):
    """Reorient all frames to Z-up, X-forward, Y-left and fix joint axes.

    Two-pass approach:
      Pass 1 - Compute world-frame rotation for every link in the original URDF
      Pass 2 - Build canonical (world-aligned) frames for each link independently,
               compute new joint rpy/xyz, transform link elements

    This avoids twist accumulation through the kinematic chain because each
    link's canonical frame is computed directly from its world-frame axis,
    not propagated from parent.
    """

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

                # Build canonical child frame
                R_child_canon, flip = build_canonical_frame(v_world_new, desired_idx)
                R_canon[child_name] = R_child_canon

                # Joint rpy: rotation from new parent frame to new child frame
                R_joint_new = mm(mt(R_canon[parent_name]), R_child_canon)
                r, p, y = mat_to_rpy(R_joint_new)
                rpy_new = clean([r, p, y])

                origin_el.set('xyz', fmt_vec(xyz_new))
                origin_el.set('rpy', fmt_rpy(rpy_new[0], rpy_new[1], rpy_new[2]))

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


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Fix joint axes and reorient URDF frames (Z up, X forward, Y left)')
    parser.add_argument('input_urdf', help='Input URDF file path')
    parser.add_argument('--output', '-o', help='Output URDF file (default: <input>_fixed_axes.urdf)')
    args = parser.parse_args()

    if not os.path.exists(args.input_urdf):
        print(f"Error: '{args.input_urdf}' not found.")
        sys.exit(1)

    print(f"\n{'='*70}")
    print(f"  Joint Correction  (Z up, X forward, Y left)")
    print(f"{'='*70}")
    print(f"\nParsing: {args.input_urdf}")

    tree, root = parse_urdf(args.input_urdf)
    robot_name = root.get('name', 'unknown')
    print(f"Robot: {robot_name}")

    n_links = len(root.findall('link'))
    n_joints = len(root.findall('joint'))
    print(f"Links: {n_links}, Joints: {n_joints}")

    # Show current axis state
    print(f"\nJoint axes before correction:")
    for j in root.findall('joint'):
        if j.get('type') != 'revolute':
            continue
        name = j.get('name')
        axis_el = j.find('axis')
        ax = axis_el.get('xyz', '0 0 1') if axis_el is not None else '0 0 1'
        desired = get_desired_axis(name)
        status = ""
        if desired:
            desired_s = axis_str(desired)
            if ax.strip() != desired_s:
                status = f" -> [{desired_s}]"
            else:
                status = " OK"
        print(f"  {name:40s} axis=[{ax:7s}]{status}")

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
        out = f"{base}_fixed_axes{ext}"

    print(f"\nCorrecting joints...")
    corrected = correct_joints(root)

    indent_xml(corrected)
    tree.write(out, xml_declaration=True, encoding='unicode')

    # Show result summary
    print(f"\nResult joint origins:")
    for j in corrected.findall('joint'):
        if j.get('type') != 'revolute':
            continue
        name = j.get('name')
        origin = j.find('origin')
        axis_el = j.find('axis')
        rpy = origin.get('rpy', '') if origin is not None else ''
        ax = axis_el.get('xyz', '') if axis_el is not None else ''
        print(f"  {name:40s} axis=[{ax:7s}] rpy=[{rpy}]")

    print(f"\nDone!")
    print(f"  Output: {out}")


if __name__ == '__main__':
    main()
