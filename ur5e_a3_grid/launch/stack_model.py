"""Optional simulation-only riser; never edits the original URDF/SRDF files."""
import xml.etree.ElementTree as ET
from moveit_configs_utils import MoveItConfigsBuilder


def make_model(riser_height=0., package="ur5e_ag95_moveit_config", robot="ur5e_on_box"):
    height = float(riser_height)
    if not 0 <= height <= .3:
        raise ValueError("riser_height must be between 0 and 0.3 m")
    model = MoveItConfigsBuilder(robot, package_name=package).to_moveit_configs()
    root = ET.fromstring(model.robot_description["robot_description"])
    semantic = ET.fromstring(model.robot_description_semantic["robot_description_semantic"])
    # Start the standalone mock at the named home, rather than the flat zero
    # pose. Only GenericSystem initial states are changed, never live hardware.
    home = semantic.find("group_state[@name='home'][@group='ur5e_arm']")
    positions = {joint.get("name"): joint.get("value") for joint in home} if home is not None else {}
    positions["left_outer_knuckle_joint"] = "0.4"
    controls = root.findall("ros2_control")
    if not controls or any(control.findtext("hardware/plugin") != "mock_components/GenericSystem" for control in controls):
        raise ValueError("The standalone simulation requires GenericSystem hardware only")
    for control in controls:
        for joint in control.findall("joint"):
            initial = joint.find("state_interface[@name='position']/param[@name='initial_value']")
            if initial is not None and joint.get("name") in positions:
                initial.text = positions[joint.get("name")]
    if not height:
        model.robot_description["robot_description"] = ET.tostring(root, encoding="unicode")
        return model
    base = root.find("joint[@name='base_joint']")
    if base is None or base.find("parent").get("link") != "workbench":
        raise ValueError("The stacking riser requires the UR5e-on-workbench model")
    origin = base.find("origin")
    xyz = [float(value) for value in origin.get("xyz").split()]
    original_z = xyz[2]
    xyz[2] += height
    origin.set("xyz", " ".join(map(str, xyz)))
    riser = ET.SubElement(root, "link", name="ur5e_stack_riser")
    for kind in ("visual", "collision"):
        shape = ET.SubElement(riser, kind)
        geometry = ET.SubElement(shape, "geometry")
        ET.SubElement(geometry, "box", size=f"0.20 0.20 {height}")
        if kind == "visual":
            material = ET.SubElement(shape, "material", name="stack_riser_gray")
            ET.SubElement(material, "color", rgba="0.25 0.28 0.32 1")
    joint = ET.SubElement(root, "joint", name="workbench_to_stack_riser", type="fixed")
    ET.SubElement(joint, "parent", link="workbench")
    ET.SubElement(joint, "child", link="ur5e_stack_riser")
    ET.SubElement(joint, "origin", xyz=f"0 0 {original_z+height/2}", rpy="0 0 0")
    model.robot_description["robot_description"] = ET.tostring(root, encoding="unicode")
    for link in ("workbench", "base_link_inertia"):
        ET.SubElement(semantic, "disable_collisions", link1="ur5e_stack_riser", link2=link, reason="Adjacent")
    model.robot_description_semantic["robot_description_semantic"] = ET.tostring(semantic, encoding="unicode")
    return model
