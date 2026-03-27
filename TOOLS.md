# Standalone Tools

Utility scripts that can be run independently of the main pipeline. These are not part of the `build_description.py` pipeline steps — they are standalone tools for inspecting, validating, and debugging URDF files.

---

## Validate masses — `validate_masses.py`

Validates mass and inertial properties of all links in a URDF file. Useful for catching Onshape export issues (missing inertial elements, zero masses, unrealistic values) before running the pipeline or simulation.

### What it checks

1. **Missing `<inertial>`** — links with no inertial element at all
2. **Zero / negative mass** — `<mass value="0">` or negative values
3. **Missing `<inertia>` tensor** — inertial block without an inertia matrix
4. **Invalid inertia diagonal** — ixx, iyy, or izz values that are not positive

### Usage

```bash
# Basic validation:
python scripts/validate_masses.py /path/to/robot.urdf

# Validate the latest Onshape export (from repo root):
python ldr-urdf-tools/scripts/validate_masses.py ldr-harambe-v0.4/robot_harambe_v04.urdf

# Validate a post-pipeline URDF:
python ldr-urdf-tools/scripts/validate_masses.py ldr-harambe-v0.4/urdf/robot_simplified.urdf
```

| Argument | Default | Description |
|----------|---------|-------------|
| `input_urdf` | — | Path to the URDF file to validate |

### Example output

```
Link                            Mass (kg)  Inertial   Inertia
-----------------------------  ----------  --------  --------
urdf_foot_assembly               309.0000        OK        OK
urdf_foot_assembly_2             309.0000        OK        OK
urdf_simplified_torso              5.9000        OK        OK
urdf_base                          5.7340        OK        OK
urdf_l_tibia_assembly              2.2160        OK        OK
urdf_r_tibia_assembly              2.2150        OK        OK
...
-----------------------------  ----------  --------  --------
TOTAL                            654.3636

Links: 26  |  Total mass: 654.364 kg

No issues found.
```

When issues are found:

```
============================================================
  2 issue(s) found:
============================================================
  - ZERO MASS: link 'dummy_link' has mass 0 kg
  - MISSING INERTIA: link 'dummy_link' has no <inertia> element
```

### Exit codes

| Code | Meaning |
|------|---------|
| `0` | All checks passed |
| `1` | One or more issues found |

**Dependencies:** none (stdlib only)
