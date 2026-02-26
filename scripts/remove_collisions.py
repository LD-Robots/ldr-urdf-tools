#!/usr/bin/env python3
"""Remove unnecessary collision blocks from a URDF based on config.json ignore rules.

Reads the ignore patterns from ldr-harambe/config.json and strips <collision>
blocks for matching parts.  Operates on the latest ldr-harambe-N/ output
(or a path given as argument).

Usage:
    python remove_collisions.py                  # auto-detect latest ldr-harambe-N/
    python remove_collisions.py ldr-harambe-1    # explicit directory
"""

import fnmatch
import glob
import json
import os
import re
import sys

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
SOURCE_DIR = os.path.join(ROOT_DIR, "ldr-harambe")


def load_ignore_patterns():
    """Return a list of (glob_pattern, ignore_type) from config.json."""
    config_path = os.path.join(SOURCE_DIR, "config.json")
    with open(config_path, "r") as f:
        raw = re.sub(r"//.*", "", f.read())
    config = json.loads(raw)
    return config.get("ignore", {})


def latest_output_dir():
    """Find the highest-numbered ldr-harambe-N directory."""
    dirs = glob.glob(os.path.join(ROOT_DIR, "ldr-harambe-[0-9]*"))
    if not dirs:
        return None
    nums = {}
    for d in dirs:
        m = re.search(r"ldr-harambe-(\d+)$", d)
        if m:
            nums[int(m.group(1))] = d
    return nums[max(nums)] if nums else None


def should_ignore_collision(part_name, ignore_rules):
    """Check if a part name matches any collision ignore pattern."""
    for pattern, ignore_type in ignore_rules.items():
        if ignore_type != "collision":
            continue
        if fnmatch.fnmatch(part_name, pattern):
            return True
    return False


def remove_collisions(urdf_text, ignore_rules):
    """Remove <collision> blocks for parts matching ignore rules.

    The URDF uses ``<!-- Part part_name -->`` comments before each
    visual/collision pair.  We walk through these anchors and drop the
    collision block when the part matches an ignore pattern.
    """
    # Split into segments around <!-- Part ... --> comments
    # Each segment: (part_name, content_after_comment)
    part_pattern = re.compile(r"(<!-- Part (.+?) -->)")

    parts = list(part_pattern.finditer(urdf_text))
    if not parts:
        print("  No <!-- Part ... --> comments found in URDF.")
        return urdf_text

    removed = 0
    kept = 0
    result_chunks = []
    prev_end = 0

    for i, match in enumerate(parts):
        part_name = match.group(2).strip()
        comment_start = match.start()
        comment_end = match.end()

        # Content between previous part end and this comment
        result_chunks.append(urdf_text[prev_end:comment_start])

        # Determine the region owned by this part comment (up to next comment or end of link)
        if i + 1 < len(parts):
            region_end = parts[i + 1].start()
        else:
            region_end = len(urdf_text)

        region = urdf_text[comment_end:region_end]

        if should_ignore_collision(part_name, ignore_rules):
            # Keep the comment + visual, strip collision blocks
            cleaned = re.sub(
                r"\s*<collision>.*?</collision>", "", region, flags=re.DOTALL
            )
            result_chunks.append(match.group(0))
            result_chunks.append(cleaned)
            removed += 1
        else:
            # Keep everything as-is
            result_chunks.append(match.group(0))
            result_chunks.append(region)
            kept += 1

        prev_end = region_end

    # Remaining content after last part
    result_chunks.append(urdf_text[prev_end:])

    print(f"  Collision blocks removed: {removed}")
    print(f"  Collision blocks kept:    {kept}")
    return "".join(result_chunks)


def main():
    ignore_rules = load_ignore_patterns()
    if not ignore_rules:
        print("No ignore rules found in config.json")
        return

    collision_patterns = {k: v for k, v in ignore_rules.items() if v == "collision"}
    print(f"Ignore patterns ({len(collision_patterns)}):")
    for pat in collision_patterns:
        print(f"  {pat}")

    # Determine target directory
    if len(sys.argv) > 1:
        target_dir = os.path.join(ROOT_DIR, sys.argv[1])
    else:
        target_dir = latest_output_dir()

    if not target_dir or not os.path.isdir(target_dir):
        print(f"Error: output directory not found. Run simplify_collisions.py first.")
        sys.exit(1)

    urdf_files = glob.glob(os.path.join(target_dir, "*.urdf"))
    if not urdf_files:
        print(f"Error: no URDF files found in {os.path.relpath(target_dir, ROOT_DIR)}/")
        sys.exit(1)

    for urdf_path in urdf_files:
        rel = os.path.relpath(urdf_path, ROOT_DIR)
        print(f"\nProcessing {rel}")
        with open(urdf_path, "r") as f:
            original = f.read()

        result = remove_collisions(original, ignore_rules)

        with open(urdf_path, "w") as f:
            f.write(result)

        print(f"  Saved {rel}")


if __name__ == "__main__":
    main()
