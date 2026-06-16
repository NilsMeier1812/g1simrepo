from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    interface = LaunchConfiguration("interface")
    use_robot = LaunchConfiguration("use_robot")
    arm_controlled = LaunchConfiguration("arm_controlled")
    enable_arm_ui = LaunchConfiguration("enable_arm_ui")
    ik_use_waist = LaunchConfiguration("ik_use_waist")
    ik_alpha = LaunchConfiguration("ik_alpha")
    ik_max_dq_step = LaunchConfiguration("ik_max_dq_step")
    arm_velocity_limit = LaunchConfiguration("arm_velocity_limit")

    return LaunchDescription([
        DeclareLaunchArgument("interface", default_value="eth0"),
        DeclareLaunchArgument("use_robot", default_value="true"),
        DeclareLaunchArgument("arm_controlled", default_value="both"),
        DeclareLaunchArgument("enable_arm_ui", default_value="true"),
        DeclareLaunchArgument("ik_use_waist", default_value="false"),
        DeclareLaunchArgument("ik_alpha", default_value="0.2"),
        DeclareLaunchArgument("ik_max_dq_step", default_value="0.05"),
        DeclareLaunchArgument("arm_velocity_limit", default_value="2.0"),
        # Marker senden ihr Ziel sofort beim Ziehen (gruener Wuerfel). Steht das
        # auf false, sind die Wuerfel grau und stumm, bis man sie per Rechtsklick-
        # Menue "Enable publishing" aktiviert. Der Arm bewegt sich ohnehin nur,
        # wenn /g1pilot/arms/enabled true ist -> das bleibt die Sicherheitsschranke.
        DeclareLaunchArgument("marker_publish_default", default_value="true"),

        Node(
            package='g1pilot',
            executable='arm_controller',
            name='arm_controller',
            parameters=[{
                'interface': interface,
                'use_robot': ParameterValue(use_robot, value_type=bool),
            }],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='dx3_controller',
            name='dx3_controller',
            parameters=[{
                'arm_controlled': ParameterValue(LaunchConfiguration("arm_controlled"), value_type=str),
                'interface': ParameterValue(LaunchConfiguration("interface"), value_type=str)
            }],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='interactive_marker',
            name='interactive_marker',
            parameters=[{
                'interface': interface,
                'use_robot': ParameterValue(use_robot, value_type=bool),
                'publish_enabled_default': ParameterValue(
                    LaunchConfiguration("marker_publish_default"), value_type=bool),
            }],
            output='screen'
        ),
    ])
