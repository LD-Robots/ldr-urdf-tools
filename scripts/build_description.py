#!/usr/bin/env python3
"""
Master script: runs the full URDF pipeline and deploys to a robot_description package.

Runs all 4 steps:
  1. urdf_simplify.py    — simplify URDF structure from Onshape export
  2. apply_joint_limits.py — apply joint limits from YAML config
  3. simplify_meshes.py   — decimate visual meshes + convex hull collision
  4. split_urdf.py        — split into joints/links xacro files

Then copies the results (xacro files + meshes) to the target robot_description package.

Usage (run from repo root):
    python util/scripts/build_description.py /path/to/my_robot_description
    python util/scripts/build_description.py /path/to/dual_arm_description -r 0.2 --fixed-legs
    python util/scripts/build_description.py /path/to/dual_arm_description --skip-simplify
"""

import argparse
import os
import shutil
import subprocess
import sys


def run(cmd: list[str], desc: str, cwd: str) -> None:
    """Run a command, printing it and checking for errors."""
    print(f"\n{'='*70}")
    print(f"  {desc}")
    print(f"  $ {' '.join(cmd)}")
    print(f"{'='*70}\n")
    result = subprocess.run(cmd, check=False, cwd=cwd)
    if result.returncode != 0:
        print(f"\nERROR: {desc} failed (exit code {result.returncode})")
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description="Run full URDF pipeline and deploy to a robot_description package."
    )
    parser.add_argument(
        "target",
        help="Path to target robot_description package "
             "(e.g. /path/to/dual_arm_description)",
    )
    parser.add_argument(
        "--source-urdf",
        default="robot.urdf",
        help="Source URDF from Onshape export (default: robot.urdf)",
    )
    parser.add_argument(
        "-r", "--ratio",
        type=float,
        default=0.2,
        help="Visual mesh decimation ratio (default: 0.2 = 20%%)",
    )
    parser.add_argument(
        "--skip-simplify",
        action="store_true",
        help="Skip step 1 (urdf_simplify.py), use existing urdf/robot_simplified.urdf",
    )
    parser.add_argument(
        "--skip-limits",
        action="store_true",
        help="Skip step 2 (apply_joint_limits.py), use existing urdf/robot_with_limits.urdf",
    )
    parser.add_argument(
        "--fixed-legs",
        action="store_true",
        help="Add xacro support for fixed_legs argument",
    )
    parser.add_argument(
        "--damping",
        type=float,
        default=0.5,
        help="Joint damping (default: 0.5)",
    )
    parser.add_argument(
        "--friction",
        type=float,
        default=0.1,
        help="Joint friction (default: 0.1)",
    )
    args = parser.parse_args()

    # Resolve paths — repo root is two levels up from this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(script_dir, "..", ".."))
    target = os.path.abspath(args.target)
    package_name = os.path.basename(target)

    print(f"Repo root:      {repo_root}")
    print(f"Target package: {package_name}")
    print(f"Target path:    {target}")

    if not os.path.isdir(target):
        print(f"ERROR: Target directory does not exist: {target}")
        sys.exit(1)

    # Step 1: Simplify URDF structure
    if not args.skip_simplify:
        run(
            [sys.executable, os.path.join(script_dir, "urdf_simplify.py"),
             args.source_urdf,
             "-o", os.path.join("urdf", "robot_simplified.urdf"),
             "-c", os.path.join("util", "configs", "simplify_config.yaml")],
            "Step 1/4: Simplify URDF structure",
            cwd=repo_root,
        )
    else:
        print("\n-- Skipping step 1 (urdf_simplify.py)")

    # Step 2: Apply joint limits
    if not args.skip_limits:
        run(
            [sys.executable, os.path.join(script_dir, "apply_joint_limits.py"),
             "-i", os.path.join("urdf", "robot_simplified.urdf"),
             "-o", os.path.join("urdf", "robot_with_limits.urdf"),
             "-c", os.path.join("util", "configs", "joint_limits.yaml")],
            "Step 2/4: Apply joint limits",
            cwd=repo_root,
        )
    else:
        print("\n-- Skipping step 2 (apply_joint_limits.py)")

    # Step 3: Simplify meshes
    run(
        [sys.executable, os.path.join(script_dir, "simplify_meshes.py"),
         "-i", os.path.join("urdf", "robot_with_limits.urdf"),
         "-o", os.path.join("urdf", "robot_gazebo.urdf"),
         "-r", str(args.ratio),
         "--visual-dir", os.path.join("urdf", "meshes", "visual") + "/",
         "--collision-dir", os.path.join("urdf", "meshes", "collision") + "/",
         "--convex-collision"],
        "Step 3/4: Decimate meshes (visual + convex collision)",
        cwd=repo_root,
    )

    # Step 4: Split into xacro files
    split_cmd = [
        sys.executable, os.path.join(script_dir, "split_urdf.py"),
        "-i", os.path.join("urdf", "robot_gazebo.urdf"),
        "-p", package_name,
        "-o", "urdf/",
        "--damping", str(args.damping),
        "--friction", str(args.friction),
    ]
    if args.fixed_legs:
        split_cmd.append("--fixed-legs")
    run(split_cmd, "Step 4/4: Split into joints/links xacro files", cwd=repo_root)

    # Deploy to target package
    print(f"\n{'='*70}")
    print(f"  Deploying to {target}")
    print(f"{'='*70}\n")

    # Derive xacro base name (e.g. "dual_arm" from "dual_arm_description")
    xacro_name = package_name.replace("_description", "")

    # Copy xacro files
    joints_src = os.path.join(repo_root, "urdf", "joints", "gazebo_joints.xacro")
    links_src = os.path.join(repo_root, "urdf", "links", "gazebo_links.xacro")
    joints_dst = os.path.join(target, "urdf", "joints", f"{xacro_name}_joints.xacro")
    links_dst = os.path.join(target, "urdf", "links", f"{xacro_name}_links.xacro")

    os.makedirs(os.path.dirname(joints_dst), exist_ok=True)
    os.makedirs(os.path.dirname(links_dst), exist_ok=True)

    shutil.copy2(joints_src, joints_dst)
    print(f"  Copied: {joints_dst}")
    shutil.copy2(links_src, links_dst)
    print(f"  Copied: {links_dst}")

    # Copy meshes
    for mesh_type in ["visual", "collision"]:
        src_dir = os.path.join(repo_root, "urdf", "meshes", mesh_type)
        dst_dir = os.path.join(target, "meshes", mesh_type)
        os.makedirs(dst_dir, exist_ok=True)

        count = 0
        for f in sorted(os.listdir(src_dir)):
            if f.endswith(".stl"):
                shutil.copy2(os.path.join(src_dir, f), os.path.join(dst_dir, f))
                count += 1
        print(f"  Copied: {count} {mesh_type} meshes -> {dst_dir}")

    print(f"\nDone! Package '{package_name}' updated.")
    print(f"  Xacro files:  {xacro_name}_joints.xacro, {xacro_name}_links.xacro")
    print(f"  Mesh paths:   package://{package_name}/meshes/visual|collision/*.stl")


if __name__ == "__main__":
    main()
