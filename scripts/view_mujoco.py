import mujoco
import mujoco.viewer
import os
import xml.etree.ElementTree as ET

# Ensure we run from ldr-harambe/ which contains the URDF and assets
os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), "ldr-harambe"))

urdf_path = "robot_harambe.urdf"
with open(urdf_path, "r") as f:
    urdf_xml = f.read()

# Strip package:// prefix so mesh paths resolve as relative
urdf_xml = urdf_xml.replace("package://", "")

# Add meshdir compiler directive so MuJoCo finds the STL files
urdf_xml = urdf_xml.replace(
    '<robot name="robot_harambe">',
    '<robot name="robot_harambe">\n'
    '  <mujoco><compiler meshdir="assets/" balanceinertia="true"/></mujoco>',
)

# Add a dummy world link + floating joint so MuJoCo absorbs the dummy
# into worldbody and keeps urdf_base as a proper body with a freejoint.
urdf_xml = urdf_xml.replace(
    '<link name="urdf_base">',
    '<link name="world"/>\n'
    '  <joint name="floating_base" type="floating">\n'
    '    <parent link="world"/>\n'
    '    <child link="urdf_base"/>\n'
    '  </joint>\n'
    '  <link name="urdf_base">',
)

# Add a ground plane via MJCF include trick: save URDF-converted model,
# then patch in a floor before final load.
tmp_model = mujoco.MjModel.from_xml_string(urdf_xml)
mjcf_path = "robot_harambe_converted.xml"
mujoco.mj_saveLastXML(mjcf_path, tmp_model)

tree = ET.parse(mjcf_path)
worldbody = tree.getroot().find("worldbody")
ET.SubElement(worldbody, "geom", name="floor", type="plane",
              size="5 5 0.1", rgba="0.8 0.8 0.8 1")
tree.write(mjcf_path, xml_declaration=True)

model = mujoco.MjModel.from_xml_path(mjcf_path)
data = mujoco.MjData(model)

# Compute the lowest geom point and raise the robot above the floor.
# Set it on model.qpos0 so resets also start above ground.
mujoco.mj_forward(model, data)
min_z = min(data.geom_xpos[i][2] for i in range(model.ngeom)
            if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) != "floor")
model.qpos0[2] -= min_z - 0.02
mujoco.mj_resetData(model, data)

mujoco.viewer.launch(model, data)
