"""
ur_real_control_robotiq_ur16e.launch.py

실물 UR16e + Robotiq 2F-85 ros2_control bring-up 전용 런치.

이 런치는 controller_manager, UR driver, robot_state_publisher 및
필수 broadcaster/controller spawner만 실행한다. MoveIt2/RViz는 별도
ur_real_moveit_robotiq_ur16e.launch.py에서 실행한다.

사용 순서:
  1) 이 런치 실행
  2) UR 티칭팬던트에서 External Control 프로그램 Run
  3) ur_real_moveit_robotiq_ur16e.launch.py 실행
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur16e"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.56.101"),
        DeclareLaunchArgument("activate_joint_controller", default_value="false"),
        DeclareLaunchArgument("initial_joint_controller",
                              default_value="scaled_joint_trajectory_controller"),
        DeclareLaunchArgument("gripper_on_tool", default_value="true"),
        DeclareLaunchArgument("use_tool_communication", default_value="false"),
        DeclareLaunchArgument("enable_direct_robotiq_control", default_value="false"),
        DeclareLaunchArgument("gripper_com_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("headless_mode", default_value="false"),
        DeclareLaunchArgument("launch_dashboard_client", default_value="true"),
        DeclareLaunchArgument("tool_device_name", default_value="/tmp/ttyUR"),
        DeclareLaunchArgument("tool_tcp_port", default_value="54321"),
        DeclareLaunchArgument("tool_voltage", default_value="24"),
        DeclareLaunchArgument("tool_parity", default_value="0"),
        DeclareLaunchArgument("tool_baud_rate", default_value="115200"),
        DeclareLaunchArgument("tool_stop_bits", default_value="1"),
        DeclareLaunchArgument("tool_rx_idle_chars", default_value="1.5"),
        DeclareLaunchArgument("tool_tx_idle_chars", default_value="3.5"),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument("prefix", default_value=""),
        DeclareLaunchArgument("pedestal_x", default_value="2.0"),
        DeclareLaunchArgument("pedestal_y", default_value="0.8"),
        DeclareLaunchArgument("pedestal_z", default_value="0.9"),
        DeclareLaunchArgument("plate_x", default_value="0.60"),
        DeclareLaunchArgument("plate_y", default_value="0.25"),
        DeclareLaunchArgument("pallet_x", default_value="-0.60"),
        DeclareLaunchArgument("pallet_y", default_value="0.35"),
    ]

    return LaunchDescription([
        *declared_arguments,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("ur_setup_bringup"),
                "/launch/ur_real_moveit_robotiq_ur16e.launch.py",
            ]),
            launch_arguments={
                "launch_control": "true",
                "launch_moveit": "false",
                "launch_rviz": "false",
                "ur_type": LaunchConfiguration("ur_type", default="ur16e"),
                "robot_ip": LaunchConfiguration("robot_ip", default="192.168.56.101"),
                "activate_joint_controller": LaunchConfiguration(
                    "activate_joint_controller", default="false"),
                "initial_joint_controller": LaunchConfiguration(
                    "initial_joint_controller",
                    default="scaled_joint_trajectory_controller"),
                "gripper_on_tool": LaunchConfiguration("gripper_on_tool", default="true"),
                "use_tool_communication": LaunchConfiguration(
                    "use_tool_communication", default="false"),
                "enable_direct_robotiq_control": LaunchConfiguration(
                    "enable_direct_robotiq_control", default="false"),
                "gripper_com_port": LaunchConfiguration(
                    "gripper_com_port", default="/dev/ttyUSB0"),
                "headless_mode": LaunchConfiguration("headless_mode", default="false"),
                "launch_dashboard_client": LaunchConfiguration(
                    "launch_dashboard_client", default="true"),
                "tool_device_name": LaunchConfiguration("tool_device_name", default="/tmp/ttyUR"),
                "tool_tcp_port": LaunchConfiguration("tool_tcp_port", default="54321"),
                "tool_voltage": LaunchConfiguration("tool_voltage", default="24"),
                "tool_parity": LaunchConfiguration("tool_parity", default="0"),
                "tool_baud_rate": LaunchConfiguration("tool_baud_rate", default="115200"),
                "tool_stop_bits": LaunchConfiguration("tool_stop_bits", default="1"),
                "tool_rx_idle_chars": LaunchConfiguration("tool_rx_idle_chars", default="1.5"),
                "tool_tx_idle_chars": LaunchConfiguration("tool_tx_idle_chars", default="3.5"),
                "safety_limits": LaunchConfiguration("safety_limits", default="true"),
                "safety_pos_margin": LaunchConfiguration("safety_pos_margin", default="0.15"),
                "safety_k_position": LaunchConfiguration("safety_k_position", default="20"),
                "prefix": LaunchConfiguration("prefix", default=""),
                "pedestal_x": LaunchConfiguration("pedestal_x", default="2.0"),
                "pedestal_y": LaunchConfiguration("pedestal_y", default="0.8"),
                "pedestal_z": LaunchConfiguration("pedestal_z", default="0.9"),
                "plate_x": LaunchConfiguration("plate_x", default="0.60"),
                "plate_y": LaunchConfiguration("plate_y", default="0.25"),
                "pallet_x": LaunchConfiguration("pallet_x", default="-0.60"),
                "pallet_y": LaunchConfiguration("pallet_y", default="0.35"),
            }.items(),
        ),
    ])
