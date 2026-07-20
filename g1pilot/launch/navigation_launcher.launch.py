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
    max_vx = LaunchConfiguration("max_vx")
    max_vy = LaunchConfiguration("max_vy")
    max_vyaw = LaunchConfiguration("max_vyaw")
    cmd_vel_timeout = LaunchConfiguration("cmd_vel_timeout")
    damp_on_init = LaunchConfiguration("damp_on_init")

    return LaunchDescription([
        DeclareLaunchArgument("interface", default_value="eth0"),
        DeclareLaunchArgument("use_robot", default_value="true"),
        DeclareLaunchArgument("arm_controlled", default_value="both"),
        DeclareLaunchArgument("enable_arm_ui", default_value="true"),
        DeclareLaunchArgument("ik_use_waist", default_value="false"),
        DeclareLaunchArgument("ik_alpha", default_value="0.2"),
        DeclareLaunchArgument("ik_max_dq_step", default_value="0.05"),
        DeclareLaunchArgument("arm_velocity_limit", default_value="2.0"),
        # Streamdeck-/PS4-Walk: Geschwindigkeits-Limits + Deadman-Timeout
        DeclareLaunchArgument("max_vx", default_value="0.4"),
        DeclareLaunchArgument("max_vy", default_value="0.3"),
        DeclareLaunchArgument("max_vyaw", default_value="0.4"),
        DeclareLaunchArgument("cmd_vel_timeout", default_value="0.5"),
        DeclareLaunchArgument("damp_on_init", default_value="false"),

        Node(
            package='g1pilot',
            executable='loco_client',
            name='loco_client',
            parameters=[{
                'interface': interface,
                'use_robot': ParameterValue(use_robot, value_type=bool),
                'arm_controlled': arm_controlled,  # string ('left'|'right'|'both')
                'enable_arm_ui': ParameterValue(enable_arm_ui, value_type=bool),
                'ik_use_waist': ParameterValue(ik_use_waist, value_type=bool),
                'ik_alpha': ParameterValue(ik_alpha, value_type=float),
                'ik_max_dq_step': ParameterValue(ik_max_dq_step, value_type=float),
                'arm_velocity_limit': ParameterValue(arm_velocity_limit, value_type=float),
                'max_vx': ParameterValue(max_vx, value_type=float),
                'max_vy': ParameterValue(max_vy, value_type=float),
                'max_vyaw': ParameterValue(max_vyaw, value_type=float),
                'cmd_vel_timeout': ParameterValue(cmd_vel_timeout, value_type=float),
                'damp_on_init': ParameterValue(damp_on_init, value_type=bool),
            }],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='nav2point',
            name='nav2point',
            parameters=[{
                'interface': interface,
                'use_robot': ParameterValue(use_robot, value_type=bool),
            }],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='dijkstra_planner',
            name='dijkstra_planner',
            parameters=[{
                'interface': interface,
                'use_robot': ParameterValue(use_robot, value_type=bool),
            }],
            output='screen'
        ),

        # Umgebungs-Objekte aus der MuJoCo-Sim (Hindernisse + Greif-Objekte)
        # -> /scene_markers. Quelle fuer create_map (/map) UND den IK-Solver
        # (Umgebungs-Kollision, siehe arm_controller.py). Siehe SCENE_BRIDGE.md.
        Node(
            package='g1pilot',
            executable='scene_bridge',
            name='scene_bridge',
            output='screen'
        ),
    ])
