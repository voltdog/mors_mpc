from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import math
import os
import yaml


def nav_static_tf(context):
    """Static TF base_link -> {lidar, base_footprint} for dddmr, mount taken from sensors.yaml."""
    if LaunchConfiguration("nav").perform(context).lower() != "true":
        return []

    with open(LaunchConfiguration("sensors_config_path").perform(context), "r", encoding="utf-8") as f:
        lidar = yaml.safe_load(f)["robosense_airy"]
    base_link = LaunchConfiguration("base_link_frame_id").perform(context)
    xyz = lidar["mount"]["xyz"]
    rpy = [math.radians(a) for a in lidar["mount"]["rpy_deg"]]
    footprint_z = LaunchConfiguration("base_footprint_z").perform(context)

    def static_tf(name, child, xyz, rpy):
        args = []
        for key, value in zip(("x", "y", "z", "roll", "pitch", "yaw"), [*xyz, *rpy]):
            args += [f"--{key}", str(value)]
        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=name,
            output="screen",
            arguments=args + ["--frame-id", base_link, "--child-frame-id", child],
        )

    return [
        static_tf("lidar_static_tf", lidar["frame_id"], xyz, rpy),
        static_tf("base_footprint_static_tf", "base_footprint", [0.0, 0.0, footprint_z], [0.0, 0.0, 0.0]),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_state_viewer")
    default_rviz_config = os.path.join(pkg_share, "rviz", "rviz_config.rviz")
    default_urdf = os.path.join(pkg_share, "urdf", "mors.urdf")
    default_heightmap_config = os.path.abspath(
        os.path.join(pkg_share, "..", "..", "..", "..", "..", "config", "heightmap_builder.yaml")
    )
    default_sensors_config = os.path.join(os.path.dirname(default_heightmap_config), "sensors.yaml")

    with open(default_urdf, "r", encoding="utf-8") as f:
        robot_description = f.read()

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="Path to RViz config file",
    )
    heightmap_config_arg = DeclareLaunchArgument(
        "heightmap_config_path",
        default_value=default_heightmap_config,
        description="Path to heightmap_builder.yaml used to decode HEIGHTMAP",
    )
    world_frame_arg = DeclareLaunchArgument(
        "world_frame_id",
        default_value="world",
        description="Global frame used for TF root and pointcloud frame_id",
    )
    pointcloud_frame_arg = DeclareLaunchArgument(
        "pointcloud_frame_id",
        default_value=LaunchConfiguration("world_frame_id"),
        description="Frame id assigned to /camera/poincloud",
    )
    base_link_frame_arg = DeclareLaunchArgument(
        "base_link_frame_id",
        default_value="base_link",
        description="Robot base link frame id",
    )
    nav_arg = DeclareLaunchArgument(
        "nav",
        default_value="false",
        description="Navigation mode: publish static TF base_link -> {lidar, base_footprint}",
    )
    stamp_from_lcm_arg = DeclareLaunchArgument(
        "stamp_from_lcm",
        default_value="false",
        description="Stamp TF, /odom and /joint_states with ROBOT_STATE timestamp",
    )
    sensors_config_arg = DeclareLaunchArgument(
        "sensors_config_path",
        default_value=default_sensors_config,
        description="Path to sensors.yaml with the robosense_airy mount",
    )
    base_footprint_z_arg = DeclareLaunchArgument(
        "base_footprint_z",
        default_value="-0.206",
        description="base_footprint offset along base_link z (nominal body height)",
    )

    viewer_node = Node(
        package="robot_state_viewer",
        executable="robot_state_viewer_node",
        name="robot_state_viewer",
        output="screen",
        parameters=[
            {
                "world_frame_id": LaunchConfiguration("world_frame_id"),
                "pointcloud_frame_id": LaunchConfiguration("pointcloud_frame_id"),
                "base_link_frame_id": LaunchConfiguration("base_link_frame_id"),
                "heightmap_config_path": LaunchConfiguration("heightmap_config_path"),
                "stamp_from_lcm": LaunchConfiguration("stamp_from_lcm"),
            }
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     arguments=["-d", LaunchConfiguration("rviz_config")],
    # )

    ld = LaunchDescription()
    ld.add_action(rviz_config_arg)
    ld.add_action(heightmap_config_arg)
    ld.add_action(world_frame_arg)
    ld.add_action(pointcloud_frame_arg)
    ld.add_action(base_link_frame_arg)
    ld.add_action(nav_arg)
    ld.add_action(stamp_from_lcm_arg)
    ld.add_action(sensors_config_arg)
    ld.add_action(base_footprint_z_arg)
    ld.add_action(viewer_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(OpaqueFunction(function=nav_static_tf))
    # ld.add_action(rviz_node)
    return ld
