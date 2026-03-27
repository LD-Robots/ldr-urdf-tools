#!/usr/bin/env python3
"""Validate mass and inertial properties of all links in a URDF file.

Checks for missing inertial elements, zero/negative masses,
missing or invalid inertia tensors, and prints a summary table.

Usage:
    python validate_masses.py robot.urdf
"""

import argparse
import os
import sys
import xml.etree.ElementTree as ET


def validate_urdf_masses(urdf_path):
    """Parse URDF and validate mass/inertial properties for all links.

    Returns (results, errors) where results is a list of dicts and
    errors is a list of error message strings.
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    results = []
    errors = []

    for link in root.findall("link"):
        name = link.get("name", "<unnamed>")
        entry = {"name": name, "mass": None, "has_inertial": False, "has_inertia": False}

        inertial = link.find("inertial")
        if inertial is None:
            errors.append(f"MISSING INERTIAL: link '{name}' has no <inertial> element")
            results.append(entry)
            continue

        entry["has_inertial"] = True

        # Check mass
        mass_el = inertial.find("mass")
        if mass_el is None:
            errors.append(f"MISSING MASS: link '{name}' has <inertial> but no <mass>")
        else:
            mass = float(mass_el.get("value", "0"))
            entry["mass"] = mass
            if mass < 0:
                errors.append(f"NEGATIVE MASS: link '{name}' has mass {mass} kg")
            elif mass == 0:
                errors.append(f"ZERO MASS: link '{name}' has mass 0 kg")

        # Check inertia tensor
        inertia_el = inertial.find("inertia")
        if inertia_el is None:
            errors.append(f"MISSING INERTIA: link '{name}' has no <inertia> element")
        else:
            entry["has_inertia"] = True
            for diag in ("ixx", "iyy", "izz"):
                val = float(inertia_el.get(diag, "0"))
                if val <= 0:
                    errors.append(
                        f"INVALID INERTIA: link '{name}' has {diag}={val} (must be > 0)"
                    )

        results.append(entry)

    return results, errors


def print_report(results, errors):
    """Print a formatted mass report table and any errors."""
    # Sort by mass descending (None masses last)
    sorted_results = sorted(results, key=lambda r: r["mass"] if r["mass"] is not None else -1, reverse=True)

    # Table header
    name_width = max(len(r["name"]) for r in results) if results else 20
    name_width = max(name_width, 4)  # minimum "Link" header

    print(f"\n{'Link':<{name_width}}  {'Mass (kg)':>10}  {'Inertial':>8}  {'Inertia':>8}")
    print(f"{'-' * name_width}  {'-' * 10}  {'-' * 8}  {'-' * 8}")

    total_mass = 0.0
    for r in sorted_results:
        mass_str = f"{r['mass']:.4f}" if r["mass"] is not None else "N/A"
        inertial_str = "OK" if r["has_inertial"] else "MISSING"
        inertia_str = "OK" if r["has_inertia"] else "MISSING"
        print(f"{r['name']:<{name_width}}  {mass_str:>10}  {inertial_str:>8}  {inertia_str:>8}")
        if r["mass"] is not None:
            total_mass += r["mass"]

    print(f"{'-' * name_width}  {'-' * 10}  {'-' * 8}  {'-' * 8}")
    print(f"{'TOTAL':<{name_width}}  {total_mass:>10.4f}")
    print(f"\nLinks: {len(results)}  |  Total mass: {total_mass:.3f} kg")

    # Errors
    if errors:
        print(f"\n{'=' * 60}")
        print(f"  {len(errors)} issue(s) found:")
        print(f"{'=' * 60}")
        for err in errors:
            print(f"  - {err}")
    else:
        print("\nNo issues found.")


def main():
    parser = argparse.ArgumentParser(
        description="Validate mass and inertial properties in a URDF file."
    )
    parser.add_argument("input_urdf", help="Path to the URDF file to validate")
    args = parser.parse_args()

    if not os.path.exists(args.input_urdf):
        print(f"Error: '{args.input_urdf}' not found.")
        sys.exit(1)

    results, errors = validate_urdf_masses(args.input_urdf)
    print_report(results, errors)

    if errors:
        sys.exit(1)
    sys.exit(0)


if __name__ == "__main__":
    main()
