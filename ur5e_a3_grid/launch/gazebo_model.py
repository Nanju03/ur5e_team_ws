"""Gazebo-only contact model, adapted from the preserved ur5_project_v2.

The source URDF, real drivers and GenericSystem model are never modified.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

def _element(parent, tag, text=None, **attributes):
    element = ET.SubElement(parent, tag, attributes)
    if text is not None:
        element.text = str(text)
    return element


def _gazebo_model(root, model):
    share = Path(get_package_share_directory("ur5e_a3_grid"))
    physics = yaml.safe_load((share / "config/gazebo_physics.yaml").read_text())
    control = root.find("ros2_control")
    control.set("name", "GazeboSystem")
    control.find("hardware/plugin").text = "gazebo_ros2_control/GazeboSystem"
    master_name = "left_outer_knuckle_joint"
    gripper = physics["gripper"]

    # GazeboSystem's position_pid exports the standard position interface, so
    # the existing arm trajectory controller still commands the same joints.
    for joint in control.findall("joint"):
        name = joint.get("name")
        command = joint.find("command_interface")
        if name == master_name:
            command.set("name", "effort")
            _element(command, "param", -gripper["max_effort_nm"], name="min")
            _element(command, "param", gripper["max_effort_nm"], name="max")
        else:
            command.set("name", "position_pid")
            gains = physics["arm_position_pid"][name]
            for key, value in gains.items():
                _element(joint, "param", value, name="pos_" + key)
        _element(joint, "state_interface", name="effort")

    # Humble gazebo_ros2_control 0.4.10 only propagates mimic commands for
    # direct position/velocity/effort modes, not position_pid. Preserve the
    # URDF mimic kinematics for MoveIt/RSP; the force-only plugin below couples
    # measured follower angles without SetPosition or object attachment.
    mimic_plugin = _element(_element(root, "gazebo"), "plugin",
                            name="ag95_mimic_coupling",
                            filename=str(Path(get_package_prefix("ur5e_a3_grid")) /
                                         "lib/libur5e_a3_mimic.so"))
    _element(mimic_plugin, "master", master_name)
    for joint in root.findall("joint"):
        name = joint.get("name")
        mimic = joint.find("mimic")
        if name == master_name or (mimic is not None and mimic.get("joint") == master_name):
            effort = gripper["max_effort_nm"] if name == master_name else gripper["mimic_max_effort_nm"]
            joint.find("limit").set("effort", str(effort))
            dynamics = joint.find("dynamics")
            if dynamics is None:
                dynamics = _element(joint, "dynamics")
            dynamics.set("damping", str(gripper["joint_damping"]))
            dynamics.set("friction", str(gripper["joint_friction"]))
        if mimic is None or mimic.get("joint") != master_name:
            continue
        follower = _element(control, "joint", name=name)
        # State-only followers prevent GazeboSystem from applying its own
        # command or hold-joint force on top of the coupling plugin.
        for interface in ("position", "velocity", "effort"):
            _element(follower, "state_interface", name=interface)
        coupling = _element(mimic_plugin, "follower", name=name)
        for key, value in {"multiplier": mimic.get("multiplier", "1"),
                           "offset": mimic.get("offset", "0"),
                           "kp": gripper["mimic_kp"], "kd": gripper["mimic_kd"],
                           "max_effort": gripper["mimic_max_effort_nm"]}.items():
            _element(coupling, key, value)

    # The world -> workbench fixed joint anchors the whole installation.
    # Give the table inertia so URDF-to-SDF conversion retains its collision.
    table = root.find("link[@name='workbench']")
    mass = physics["table"]["mass_kg"]
    dimensions = [float(x) for x in table.find("collision/geometry/box").get("size").split()]
    x, y, z = dimensions
    inertial = _element(table, "inertial")
    _element(inertial, "mass", value=str(mass))
    _element(inertial, "inertia", ixx=str(mass * (y*y + z*z) / 12),
             iyy=str(mass * (x*x + z*z) / 12), izz=str(mass * (x*x + y*y) / 12),
             ixy="0", ixz="0", iyz="0")
    for link in root.findall("link"):
        name = link.get("name")
        if link.find("collision") is None:
            continue
        if link.find("inertial") is None:
            # Retain the fixed riser collision through URDF-to-SDF reduction.
            box = link.find("collision/geometry/box")
            if box is None:
                raise ValueError("Gazebo collision link needs inertia: " + name)
            x, y, z = map(float, box.get("size").split())
            inertial = _element(link, "inertial")
            _element(inertial, "mass", value="1.0")
            _element(inertial, "inertia", ixx=str((y*y+z*z)/12), iyy=str((x*x+z*z)/12),
                     izz=str((x*x+y*y)/12), ixy="0", ixz="0", iyz="0")
        surface = physics["finger_contact"] if "finger" in name else physics["table"]
        gazebo = _element(root, "gazebo", reference=name)
        for key, value in {"mu1": surface["friction_mu1"], "mu2": surface["friction_mu2"],
                           "kp": surface["contact_kp"], "kd": surface["contact_kd"],
                           "minDepth": .0001, "maxVel": .05}.items():
            _element(gazebo, key, value)

    # Resolve package meshes for both Gazebo and the matching MoveIt model;
    # neither simulator startup nor a mesh lookup needs an online model store.
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if filename.startswith("package://"):
            mesh_package, relative = filename[len("package://"):].split("/", 1)
            mesh.set("filename", "file://" + str(Path(get_package_share_directory(mesh_package)) / relative))

    plugin = _element(_element(root, "gazebo"), "plugin",
                      name="gazebo_ros2_control", filename="libgazebo_ros2_control.so")
    _element(plugin, "robot_param", "robot_description")
    _element(plugin, "robot_param_node", "robot_state_publisher")
    _element(plugin, "parameters", str(share / "config/gazebo_controllers.yaml"))
    _element(plugin, "hold_joints", "true")
    # MoveIt's handle otherwise sends max_effort=0, overriding the effort
    # controller's configured default and producing zero gripping torque.
    controller = model.trajectory_execution["moveit_simple_controller_manager"]["dh_ag95_gripper_controller"]
    controller["max_effort"] = float(gripper["max_effort_nm"])
    controller["allowed_goal_duration_margin"] = float(gripper["allowed_goal_duration_margin"])

