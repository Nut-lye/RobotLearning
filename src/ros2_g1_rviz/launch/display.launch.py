from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _load_robot_description(urdf_path: Path) -> str:
    text = urdf_path.read_text(encoding="utf-8")

    # Unitree official G1 URDF files use relative mesh paths such as
    # meshes/pelvis.STL. RViz2 resolves file:// paths more reliably when the
    # URDF is loaded through robot_description instead of from its directory.
    mesh_dir = urdf_path.parent / "meshes"
    if mesh_dir.exists():
        text = text.replace('filename="meshes/', f'filename="file://{mesh_dir}/')

    return text


def _launch_setup(context, *_args, **_kwargs):
    share_dir = Path(get_package_share_directory("ros2_g1_rviz"))
    urdf_arg = LaunchConfiguration("urdf").perform(context).strip()
    urdf_path = Path(urdf_arg) if urdf_arg else share_dir / "urdf" / "g1_29dof_simple.urdf"
    rviz_config = LaunchConfiguration("rviz_config").perform(context).strip()
    rviz_config_path = Path(rviz_config) if rviz_config else share_dir / "rviz" / "g1_display.rviz"

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="g1_robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": _load_robot_description(urdf_path)}],
        )
    ]

    if _as_bool(LaunchConfiguration("use_demo").perform(context)):
        nodes.append(
            Node(
                package="ros2_g1_rviz",
                executable="demo_joint_state_publisher",
                name="g1_demo_joint_state_publisher",
                output="screen",
                parameters=[
                    {
                        "rate": float(LaunchConfiguration("rate").perform(context)),
                        "amplitude": float(LaunchConfiguration("amplitude").perform(context)),
                        "motion": LaunchConfiguration("motion").perform(context),
                    }
                ],
            )
        )

    if _as_bool(LaunchConfiguration("use_lowstate").perform(context)):
        nodes.append(
            Node(
                package="ros2_g1_rviz",
                executable="lowstate_to_joint_state",
                name="g1_lowstate_to_joint_state",
                output="screen",
                parameters=[
                    {
                        "lowstate_topic": LaunchConfiguration("lowstate_topic").perform(context),
                        "publish_topic": "joint_states",
                    }
                ],
            )
        )

    if _as_bool(LaunchConfiguration("use_sdk2_lowstate").perform(context)):
        nodes.append(
            Node(
                package="ros2_g1_rviz",
                executable="sdk2_lowstate_to_joint_state",
                name="g1_sdk2_lowstate_to_joint_state",
                output="screen",
                parameters=[
                    {
                        "iface": LaunchConfiguration("iface").perform(context),
                        "domain_id": int(LaunchConfiguration("domain_id").perform(context)),
                        "lowstate_topic": LaunchConfiguration("sdk2_lowstate_topic").perform(context),
                        "publish_topic": "joint_states",
                        "rate": float(LaunchConfiguration("rate").perform(context)),
                    }
                ],
            )
        )

    if _as_bool(LaunchConfiguration("rviz").perform(context)):
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(rviz_config_path)],
            )
        )

    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf",
                default_value="",
                description="Optional absolute path to an official Unitree G1 URDF.",
            ),
            DeclareLaunchArgument("use_demo", default_value="true"),
            DeclareLaunchArgument("use_lowstate", default_value="false"),
            DeclareLaunchArgument("use_sdk2_lowstate", default_value="false"),
            DeclareLaunchArgument("iface", default_value="", description="Network interface for Unitree SDK2."),
            DeclareLaunchArgument("domain_id", default_value="0", description="Unitree SDK2 DDS domain id."),
            DeclareLaunchArgument("lowstate_topic", default_value="lowstate"),
            DeclareLaunchArgument("sdk2_lowstate_topic", default_value="rt/lowstate"),
            DeclareLaunchArgument("motion", default_value="wave", description="wave, walk, or idle"),
            DeclareLaunchArgument("rate", default_value="50.0"),
            DeclareLaunchArgument("amplitude", default_value="0.45"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=""),
            OpaqueFunction(function=_launch_setup),
        ]
    )
