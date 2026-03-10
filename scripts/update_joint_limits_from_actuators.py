#!/usr/bin/env python3
"""Update effort and velocity in joint_limits.yaml using actuator_parameters.yaml.

Matches joints to actuators by the suffix in the joint name (e.g. _X4, _X6).
Joints without a suffix use a default actuator (--default-actuator).

Usage:
    python update_joint_limits_from_actuators.py
    python update_joint_limits_from_actuators.py --use-rated
    python update_joint_limits_from_actuators.py --default-actuator X8-120
    python update_joint_limits_from_actuators.py --dry-run
"""

import argparse
import math
import re
from pathlib import Path

import yaml

CONFIGS_DIR = Path(__file__).resolve().parent.parent / "configs"

# Map suffixes found in joint names to actuator keys in actuator_parameters.yaml
SUFFIX_TO_ACTUATOR = {
    "X4": "X4-36",
    "X6": "X6-60",
    "X8": "X8-120",
}


def detect_actuator(joint_name: str, default: str | None) -> str | None:
    """Extract actuator type from joint name suffix like '_X4' or '_X6'."""
    match = re.search(r"_(X\d+)$", joint_name)
    if match:
        suffix = match.group(1)
        return SUFFIX_TO_ACTUATOR.get(suffix)
    return default


def has_field_ahead(lines: list, current_idx: int, field_name: str) -> bool:
    """Check if field_name exists in the next few lines (before next joint or section)."""
    for i in range(current_idx + 1, min(current_idx + 5, len(lines))):
        line = lines[i].strip()
        if line.startswith(f"{field_name}:"):
            return True
        # Stop if we hit a new joint name or comment section
        if re.match(r"^\S+:", line) or line.startswith("#"):
            return False
    return False


def rpm_to_rad_s(rpm: float) -> float:
    return rpm * math.pi / 30.0


def main():
    parser = argparse.ArgumentParser(
        description="Update joint_limits.yaml effort/velocity from actuator parameters"
    )
    parser.add_argument(
        "--joint-limits",
        default=str(CONFIGS_DIR / "joint_limits.yaml"),
        help="Path to joint_limits.yaml",
    )
    parser.add_argument(
        "--actuator-params",
        default=str(CONFIGS_DIR / "actuator_parameters.yaml"),
        help="Path to actuator_parameters.yaml",
    )
    parser.add_argument(
        "--default-actuator",
        default="X8-120",
        help="Actuator to use for joints without a suffix (default: X8-120)",
    )
    parser.add_argument(
        "--use-peak",
        action="store_true",
        help="Use peak torque/no-load speed instead of rated values (default: rated)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print changes without writing to file",
    )
    args = parser.parse_args()

    with open(args.actuator_params, "r") as f:
        actuator_config = yaml.safe_load(f)
    actuators = actuator_config["actuators"]

    with open(args.joint_limits, "r") as f:
        limits_config = yaml.safe_load(f)
    joints = limits_config["joints"]

    updated = []
    skipped = []

    for joint_name, limits in joints.items():
        actuator_key = detect_actuator(joint_name, args.default_actuator)

        if actuator_key is None or actuator_key not in actuators:
            skipped.append((joint_name, actuator_key))
            continue

        params = actuators[actuator_key]

        if args.use_peak:
            effort = params["peak_torque_Nm"]
            velocity = rpm_to_rad_s(params["no_load_speed_RPM"])
        else:
            effort = params["rated_torque_Nm"]
            velocity = rpm_to_rad_s(params["rated_speed_RPM"])

        velocity = round(velocity, 2)
        friction = params["back_drive_torque_Nm"]
        damping = round(friction * 0.1, 3)

        old_effort = limits.get("effort")
        old_velocity = limits.get("velocity")
        old_friction = limits.get("friction")
        old_damping = limits.get("damping")
        limits["effort"] = effort
        limits["velocity"] = velocity
        limits["friction"] = friction
        limits["damping"] = damping

        updated.append(
            (joint_name, actuator_key, old_effort, effort, old_velocity, velocity,
             old_friction, friction, old_damping, damping)
        )

    # Print summary
    print(f"{'[DRY RUN] ' if args.dry_run else ''}Updated {len(updated)} joints "
          f"({'peak' if args.use_peak else 'rated'} values):\n")

    for name, actuator, old_e, new_e, old_v, new_v, old_f, new_f, old_d, new_d in updated:
        print(f"  {name} ({actuator}):")
        print(f"    effort:   {old_e} -> {new_e} N·m")
        print(f"    velocity: {old_v} -> {new_v} rad/s")
        print(f"    friction: {old_f} -> {new_f} N·m")
        print(f"    damping:  {old_d} -> {new_d} N·m·s/rad")

    if skipped:
        print(f"\nSkipped {len(skipped)} joints (no actuator match):")
        for name, key in skipped:
            print(f"  - {name} (looked for: {key})")

    if not args.dry_run:
        # Preserve comments by reading original and doing targeted replacements
        with open(args.joint_limits, "r") as f:
            original_lines = f.readlines()

        output_lines = []
        current_joint = None

        for idx, line in enumerate(original_lines):
            # Detect joint name lines (e.g. "  some_joint_name_X6:")
            joint_match = re.match(r"^  (\S+):$", line.rstrip())
            if joint_match:
                potential = joint_match.group(1)
                if potential in joints:
                    current_joint = potential

            if current_joint:
                effort_match = re.match(r"^(\s+effort:\s*)\S+", line)
                velocity_match = re.match(r"^(\s+velocity:\s*)\S+", line)
                friction_match = re.match(r"^(\s+friction:\s*)\S+", line)
                damping_match = re.match(r"^(\s+damping:\s*)\S+", line)
                if effort_match:
                    line = f"{effort_match.group(1)}{joints[current_joint]['effort']}\n"
                elif velocity_match:
                    line = f"{velocity_match.group(1)}{joints[current_joint]['velocity']}\n"
                    # Insert friction/damping after velocity if not already present
                    if not has_field_ahead(original_lines, idx, "friction"):
                        indent = re.match(r"^(\s+)", line).group(1)
                        output_lines.append(line)
                        output_lines.append(f"{indent}friction: {joints[current_joint]['friction']}\n")
                        output_lines.append(f"{indent}damping: {joints[current_joint]['damping']}\n")
                        current_joint = None
                        continue
                elif friction_match:
                    line = f"{friction_match.group(1)}{joints[current_joint]['friction']}\n"
                elif damping_match:
                    line = f"{damping_match.group(1)}{joints[current_joint]['damping']}\n"
                    current_joint = None  # damping is last field per joint

            output_lines.append(line)

        with open(args.joint_limits, "w") as f:
            f.writelines(output_lines)

        print(f"\nWritten to {args.joint_limits}")


if __name__ == "__main__":
    main()
