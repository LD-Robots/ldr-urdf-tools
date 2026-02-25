# URDF Pipeline: Onshape to Gazebo

Pipeline for converting Onshape CAD exports into simulation-ready URDF/xacro files for a ROS2 robot_description package.

## Project structure

```
robot.urdf                          ← Onshape export (input)
util/
  scripts/                          ← all pipeline scripts
    build_description.py            ← master script (runs all 4 steps + deploy)
    urdf_simplify.py                ← merge sub-links, strip meshes, remove duplicates
    joint_correction.py             ← reorient frames, fix joint axes (Z up, X fwd, Y left)
    apply_joint_limits.py
    simplify_meshes.py
    split_urdf.py
  configs/                          ← YAML configuration
    simplify_config.yaml            ← mesh strip patterns
    joint_correction.yaml           ← base frame convention + axis mappings
    joint_limits.yaml               ← joint limits
urdf/                               ← generated outputs
  robot_simplified.urdf
  robot_fixed_axes.urdf
  robot_with_limits.urdf
  robot_gazebo.urdf
  {name}.urdf.xacro                 ← main xacro with includes
  joints/                           ← per-section joint xacro files
    body_joints.xacro
    left_arm_joints.xacro
    right_arm_joints.xacro
    left_foot_joints.xacro
    right_foot_joints.xacro
  links/                            ← per-section link xacro files
    body_links.xacro
    left_arm_links.xacro
    right_arm_links.xacro
    left_foot_links.xacro
    right_foot_links.xacro
  meshes/
    visual/*.stl
    collision/*.stl
```

## Pipeline

```
robot.urdf (Onshape export)
    │
    ▼  1. urdf_simplify.py
urdf/robot_simplified.urdf (merged links, stripped meshes)
    │
    ▼  2. joint_correction.py
urdf/robot_fixed_axes.urdf (Z up, X fwd, Y left, clean axes)
    │
    ▼  3. apply_joint_limits.py
urdf/robot_with_limits.urdf (joint limits from YAML)
    │
    ▼  4. simplify_meshes.py
urdf/robot_gazebo.urdf (decimated visual + convex collision meshes)
    │
    ▼  5. split_urdf.py
urdf/{name}.urdf.xacro + urdf/joints/*_joints.xacro + urdf/links/*_links.xacro
    │
    ▼  deploy to robot_description package
```

---

## Quick start — `build_description.py`

Runs all 5 steps and deploys the results (xacro files + meshes) to a target robot_description package. Run from repo root.

```bash
# Full pipeline with fixed legs support:
python util/scripts/build_description.py /path/to/dual_arm_description --fixed-legs

# Custom decimation ratio:
python util/scripts/build_description.py /path/to/dual_arm_description -r 0.2 --fixed-legs

# Skip steps 1-3 if URDF is already simplified + corrected:
python util/scripts/build_description.py /path/to/dual_arm_description --skip-simplify --skip-limits --fixed-legs
```

| Argument | Default | Description |
|----------|---------|-------------|
| `target` | — | Path to target robot_description package |
| `--source-urdf` | `robot.urdf` | Source URDF from Onshape export |
| `-r`, `--ratio` | `0.2` | Visual mesh decimation ratio (20%) |
| `--skip-simplify` | off | Skip steps 1-2, use existing `urdf/robot_fixed_axes.urdf` |
| `--skip-limits` | off | Skip step 3, use existing `urdf/robot_with_limits.urdf` |
| `--fixed-legs` | off | Add xacro support for `fixed_legs` argument |
| `--damping` | `0.5` | Joint damping value |
| `--friction` | `0.1` | Joint friction value |

### What it deploys

- `urdf/{name}.urdf.xacro` — main xacro with includes for each section
- `urdf/joints/*_joints.xacro` — joint files per body section (body, left_arm, right_arm, left_foot, right_foot)
- `urdf/links/*_links.xacro` — link files per body section
- `meshes/visual/*.stl` — decimated visual meshes
- `meshes/collision/*.stl` — convex hull collision meshes

The `<name>` is derived from the package name (e.g. `dual_arm_description` → `dual_arm`).

---

## Step 1: Simplify URDF structure — `urdf_simplify.py`

Simplifies URDF files exported from Onshape (via onshape-to-robot) by merging sub-links and removing duplicates.

### What it does

1. **Analyzes the URDF** — identifies main links, sub-links (duplicate/fixed joints), displays kinematic tree
2. **Waits for confirmation** (`yes`) before modifying anything
3. **Simplifies the structure**:
   - Removes sub-links (duplicate joints, fixed joints)
   - Merges sub-link masses into their parent main links
   - Strips meshes matching patterns from `simplify_config.yaml` (motor hardware, hands, brackets, etc.)

### Usage

```bash
python util/scripts/urdf_simplify.py robot.urdf
python util/scripts/urdf_simplify.py robot.urdf -o urdf/robot_simplified.urdf
python util/scripts/urdf_simplify.py robot.urdf -c util/configs/simplify_config.yaml
```

| Argument | Default | Description |
|----------|---------|-------------|
| `input_urdf` | — | Path to the input URDF file |
| `-o`, `--output` | `<input>_simplified.urdf` | Output file |
| `-c`, `--config` | `util/configs/simplify_config.yaml` | YAML config with `strip_meshes` list |

**Dependencies:** `pyyaml`

---

## Step 2: Fix joint axes — `joint_correction.py`

Reorients all frames and fixes joint axes to a standard convention.

### What it does

1. **Reorients all frames** to standard convention:
   - Z up, X forward, Y left
   - Pitch / knee / elbow → `axis="0 1 0"` (Y)
   - Roll → `axis="1 0 0"` (X)
   - Yaw → `axis="0 0 1"` (Z)
2. **Cleans rpy values** — joints get rpy close to `0 0 0`
3. **Preserves physics** — world-frame axes are identical (or flipped with limits swapped)

### Usage

```bash
python util/scripts/joint_correction.py urdf/robot_simplified.urdf
python util/scripts/joint_correction.py urdf/robot_simplified.urdf -o urdf/robot_fixed_axes.urdf
python util/scripts/joint_correction.py urdf/robot_simplified.urdf -c configs/joint_correction.yaml
```

| Argument | Default | Description |
|----------|---------|-------------|
| `input_urdf` | — | Path to the input URDF file |
| `-o`, `--output` | `<input>_fixed_axes.urdf` | Output file |
| `-c`, `--config` | `configs/joint_correction.yaml` | YAML config with base frame + axis conventions |

### YAML format

```yaml
# How the CAD export base frame is oriented in the real world
base_frame:
  x: right      # forward | back | left | right | up | down
  y: forward
  z: up

# Joint name keyword → desired axis (X, Y, or Z)
axis_conventions:
  pitch: Y
  knee:  Y
  elbow: Y
  roll:  X
  yaw:   Z
```

**Dependencies:** `pyyaml`

---

## Step 3: Apply joint limits — `apply_joint_limits.py`

Reads joint limits from a YAML config and writes them into the URDF.

### Usage

```bash
python util/scripts/apply_joint_limits.py
python util/scripts/apply_joint_limits.py -c util/configs/custom_limits.yaml
```

| Argument | Default | Description |
|----------|---------|-------------|
| `-i`, `--input` | `urdf/robot_simplified.urdf` | Input URDF |
| `-c`, `--config` | `util/configs/joint_limits.yaml` | YAML config with joint limits |
| `-o`, `--output` | `urdf/robot_with_limits.urdf` | Output URDF |

### YAML format

```yaml
joints:
  waist_yaw_joint:
    lower: -1.5708
    upper: 1.5708
    effort: 100
    velocity: 100
```

**Dependencies:** `pyyaml`

---

## Step 4: Simplify meshes — `simplify_meshes.py`

Decimates STL meshes for faster simulation. Uses Open3D's quadric decimation for visual meshes and convex hulls for collision meshes.

### Why

Onshape exports high-poly CAD meshes (6.4M triangles, 306 MB total). This is too heavy for real-time simulation. The script:
- **Visual meshes**: reduces triangle count by a ratio (default 10%) — good enough for rendering
- **Collision meshes**: computes convex hulls — much faster for physics engines (ODE/Bullet/DART)
- **Small meshes** (under `--min-triangles` threshold): copied as-is, no decimation

### Usage

```bash
# Recommended for Gazebo (visual at 20%, convex hull collision):
python util/scripts/simplify_meshes.py -r 0.2 --convex-collision

# Custom I/O:
python util/scripts/simplify_meshes.py -i urdf/robot_with_limits.urdf -o urdf/robot_gazebo.urdf -r 0.2 --convex-collision
```

| Argument | Default | Description |
|----------|---------|-------------|
| `-i`, `--input` | `urdf/robot_with_limits.urdf` | Input URDF |
| `-o`, `--output` | `urdf/robot_gazebo.urdf` | Output URDF |
| `-r`, `--ratio` | `0.1` | Fraction of triangles to keep for visual meshes |
| `--convex-collision` | off | Use convex hulls for collision (recommended for Gazebo) |
| `--collision-ratio` | same as `-r` | Separate decimation ratio for collision meshes |
| `--source-dir` | `meshes/` | Directory with original STL meshes |
| `--visual-dir` | `urdf/meshes/visual/` | Output directory for visual meshes |
| `--collision-dir` | `urdf/meshes/collision/` | Output directory for collision meshes |
| `--min-triangles` | `50000` | Meshes below this threshold are copied without decimation |

### Typical results

| | Original | Visual (20%) | Collision (convex) |
|---|---|---|---|
| Triangles | 6,426,130 | ~1,285,000 | ~210,000 |
| Size | 306 MB | ~61 MB | ~10 MB |

**Dependencies:** `open3d`

---

## Step 5: Split into xacro — `split_urdf.py`

Splits a monolithic URDF into separate xacro files per body section, matching the `full_robot_description` structure.

### What it generates

```
urdf/
  {name}.urdf.xacro                  ← main file with xacro:include
  joints/
    body_joints.xacro                ← waist joints + xacro args (fixed_legs, only_left)
    left_arm_joints.xacro
    right_arm_joints.xacro
    left_foot_joints.xacro           ← leg joints (${leg_joint_type} when --fixed-legs)
    right_foot_joints.xacro
  links/
    body_links.xacro                 ← base + torso links
    left_arm_links.xacro
    right_arm_links.xacro
    left_foot_links.xacro
    right_foot_links.xacro
```

### What it does

- Classifies joints/links into sections: Body, Left Arm, Right Arm, Left Foot, Right Foot
- Generates one xacro file per section (joints and links separately)
- Generates a main `.urdf.xacro` with `xacro:include` for each section + world joint
- Sorts joints in kinematic chain order (proximal → distal)
- Adds `<dynamics>` elements to joints
- Rewrites mesh paths to `package://<package_name>/meshes/...`
- With `--fixed-legs`: adds `xacro:arg`/`xacro:property` for `leg_joint_type` and `waist_joint_type`

### Usage

```bash
python util/scripts/split_urdf.py -i urdf/robot_gazebo.urdf -p full_robot_description --fixed-legs
```

| Argument | Default | Description |
|----------|---------|-------------|
| `-i`, `--input` | `urdf/robot_gazebo.urdf` | Input URDF |
| `-p`, `--package` | — | ROS package name (e.g. `full_robot_description`) |
| `-o`, `--output-dir` | `urdf/` | Output directory (creates `joints/` and `links/` subdirs) |
| `--damping` | `0.5` | Joint damping value |
| `--friction` | `0.1` | Joint friction value |
| `--fixed-legs` | off | Add xacro support for `fixed_legs` argument |
| `--only-left` | on | Add xacro support for `only_left` argument (locks waist) |
