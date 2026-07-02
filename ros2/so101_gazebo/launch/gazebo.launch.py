"""
Phase 7 — Gazebo Harmonic simulation for the SO-101 (controllable arm).

Starts:
  - Gazebo Harmonic with so101_table.world
  - robot_state_publisher (so101_gz.urdf.xacro → /robot_description, sim time)
  - ros_gz_sim 'create' to spawn the arm from /robot_description
  - ros_gz_bridge: Gazebo /clock → ROS /clock
  - controller spawners: joint_state_broadcaster + arm_controller
    (joint_trajectory_controller over the 6 joints)

Prerequisites:
  sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
                   ros-jazzy-gz-ros2-control ros-jazzy-joint-trajectory-controller \
                   ros-jazzy-joint-state-broadcaster

Usage:
  ros2 launch so101_gazebo gazebo.launch.py
  ros2 launch so101_gazebo gazebo.launch.py headless:=true

Drive the arm (example trajectory):
  ros2 topic pub --once /arm_controller/joint_trajectory \
    trajectory_msgs/msg/JointTrajectory \
    '{joint_names: [shoulder_pan,shoulder_lift,elbow_flex,wrist_flex,wrist_roll,gripper],
      points: [{positions: [0.3,0.0,0.0,0.0,0.0,0.0], time_from_start: {sec: 2}}]}'
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gz_pkg   = FindPackageShare("so101_gazebo")
    headless = LaunchConfiguration("headless")

    # So Gazebo can resolve package://so101_description/meshes/*.stl, point the
    # resource path at the directory that CONTAINS the so101_description share
    # folder (…/install/so101_description/share). Resolve it to an ABSOLUTE path
    # now (no ".."), since Gazebo does not collapse "..". Without this the arm
    # spawns as an entity but renders invisible (meshes not found).
    _desc_share = get_package_share_directory("so101_description")     # …/share/so101_description
    _resource_dir = os.path.dirname(_desc_share)                       # …/share
    _existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    _resource_path = f"{_resource_dir}:{_existing}" if _existing else _resource_dir
    # Set it BOTH ways: directly in this process env (so the gz server/gui that
    # the included gz_sim.launch.py spawns inherit it and can find the meshes —
    # SetEnvironmentVariable alone did not reach them), and as a launch action.
    os.environ["GZ_SIM_RESOURCE_PATH"] = _resource_path
    set_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", _resource_path)

    world = PathJoinSubstitution([gz_pkg, "worlds", "so101_table.world"])

    # Expand the xacro NOW and STRIP the <collision> geometry. The SO-101 uses its
    # detailed STL visual meshes as collision too; DART mesh-vs-mesh collision is
    # pathologically slow and drove RTF to ~0.1% (sim time frozen → arm "deaf").
    # The arm is position-controlled (kinematic) in sim, so it needs no collisions.
    import re, subprocess
    _xacro_file = os.path.join(get_package_share_directory("so101_gazebo"),
                               "urdf", "so101_gz.urdf.xacro")
    _raw = subprocess.check_output(["xacro", _xacro_file]).decode()
    _urdf = re.sub(r"<collision\b.*?</collision>", "", _raw, flags=re.DOTALL)
    # Gazebo does not reliably resolve package:// URIs → meshes don't load and the
    # arm renders invisible. Rewrite them to ABSOLUTE filesystem paths (resolve
    # symlinks too) so the mesh loader cannot fail.
    _mesh_root = os.path.realpath(_desc_share)            # …/so101_description (real path)
    _urdf = _urdf.replace("package://so101_description/", _mesh_root + "/")
    robot_description = _urdf   # plain URDF string (collisions removed, abs mesh paths)

    # Gazebo Sim (headless adds -s for server-only).
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ]),
        launch_arguments={"gz_args": ["-r -v3 ", world], "on_exit_shutdown": "true"}.items(),
    )

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "robot_description", "-name", "so101", "-z", "0.0"],
    )

    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    # Bridge the simulated gripper camera image gz → ROS so the perception nodes
    # (Grounding DINO / SAM2) can run on /gripper_camera/image_raw in sim.
    camera_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        arguments=[
            "/gripper_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/scene_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
    )

    # Longer CM timeout so the spawner waits for the (slow-to-start) gz controller
    # manager instead of timing out at 10 s, retrying, and hitting a STRICT
    # "already active" abort.
    jsb = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60"],
    )
    arm = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["arm_controller",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60"],
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        set_resource_path,
        gz_sim,
        rsp,
        clock_bridge,
        camera_bridge,
        spawn,
        # Load controllers only after the robot is spawned (and the controller
        # manager from gz_ros2_control is up).
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb,   on_exit=[arm])),
    ])
