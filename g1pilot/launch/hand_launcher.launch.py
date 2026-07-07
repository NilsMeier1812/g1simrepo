"""
hand_launcher.launch.py
Startet die Inspire-FTP-Hand-Bridge (inspire_hand) eigenstaendig.

Die Bridge bedient beide HTML-GUIs (Controller :8766 / Viewer :8765) und
publiziert die Finger-Gelenke nach /joint_states (RViz, Stufe 1).

WICHTIG: Laeuft die Bridge, muss robot_state mit publish_hand_joints:=false
gestartet werden (sonst kollidieren beide /joint_states-Quellen). In
bringup_sim.launch.py ist das schon verdrahtet; beim manuellen Start selbst
darauf achten.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ws_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("controller_port", default_value="8766"),
        DeclareLaunchArgument("viewer_port", default_value="8765"),
        DeclareLaunchArgument("update_rate_hz", default_value="50.0"),
        # sim | mujoco | modbus (modbus = ECHTE Haende, braucht left/right_host)
        DeclareLaunchArgument("backend", default_value="sim"),
        DeclareLaunchArgument("interface", default_value="lo"),
        DeclareLaunchArgument("left_host", default_value="sim://left"),
        DeclareLaunchArgument("right_host", default_value="sim://right"),
        DeclareLaunchArgument("modbus_port", default_value="6000"),

        Node(
            package="g1pilot",
            executable="inspire_hand",
            name="inspire_ftp_bridge",
            output="screen",
            parameters=[{
                "ws_host": ParameterValue(LaunchConfiguration("ws_host"), value_type=str),
                "controller_port": ParameterValue(
                    LaunchConfiguration("controller_port"), value_type=int),
                "viewer_port": ParameterValue(
                    LaunchConfiguration("viewer_port"), value_type=int),
                "update_rate_hz": ParameterValue(
                    LaunchConfiguration("update_rate_hz"), value_type=float),
                "backend": ParameterValue(LaunchConfiguration("backend"), value_type=str),
                "interface": ParameterValue(LaunchConfiguration("interface"), value_type=str),
                "left_host": ParameterValue(LaunchConfiguration("left_host"), value_type=str),
                "right_host": ParameterValue(LaunchConfiguration("right_host"), value_type=str),
                "modbus_port": ParameterValue(
                    LaunchConfiguration("modbus_port"), value_type=int),
            }],
        ),
    ])
