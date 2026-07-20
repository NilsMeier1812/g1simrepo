from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    interface = LaunchConfiguration("interface")
    use_robot = LaunchConfiguration("use_robot")
    enable_arm_ui = LaunchConfiguration("enable_arm_ui")
    ik_use_waist = LaunchConfiguration("ik_use_waist")
    ik_alpha = LaunchConfiguration("ik_alpha")
    ik_max_dq_step = LaunchConfiguration("ik_max_dq_step")
    arm_velocity_limit = LaunchConfiguration("arm_velocity_limit")

    return LaunchDescription([
        DeclareLaunchArgument("interface", default_value="eth0"),
        DeclareLaunchArgument("use_robot", default_value="true"),
        DeclareLaunchArgument("enable_arm_ui", default_value="true"),
        DeclareLaunchArgument("ik_use_waist", default_value="false"),
        DeclareLaunchArgument("ik_alpha", default_value="0.2"),
        DeclareLaunchArgument("ik_max_dq_step", default_value="0.05"),
        # Gelenk-Speedlimit [rad/s]. Wird (anders als frueher: der Wert wurde
        # deklariert, aber NIE an den Node uebergeben -> es galt der Code-
        # Default 5.0!) jetzt wirklich durchgereicht.
        DeclareLaunchArgument("arm_velocity_limit", default_value="1.5"),
        # Kartesisches Speedlimit [m/s] fuer Hand-TCP + Ellbogen:
        # 0.25 m/s = reduced speed nach ISO 10218-1 / ISO TS 15066.
        DeclareLaunchArgument("ee_velocity_limit", default_value="0.25"),
        # Selbstkollisions-Gate (kommandierte Pose wird vor dem Senden geprueft).
        DeclareLaunchArgument("self_collision_gate", default_value="true"),
        # Umgebungs-Kollisions-Gate (Hindernisse/Greif-Objekte aus /scene_markers,
        # siehe SCENE_BRIDGE.md Abschnitt 6). Eigener Schalter, unabhaengig vom
        # Selbstkollisions-Gate oben.
        DeclareLaunchArgument("environment_collision_gate", default_value="true"),
        # Toleranz [rad] fuer die geplante Bewegung (Positionsspeicher, siehe
        # SCENE_BRIDGE.md Abschnitt 9), ab der ein Wegpunkt als erreicht gilt.
        DeclareLaunchArgument("planned_motion_tolerance", default_value="0.02"),
        # PD-Gains des arm_controller. Defaults = Sim-Tuning (MuJoCo braucht
        # hohe Daempfung); bringup_real ueberschreibt mit den Unitree-
        # Beispielwerten (kp=60, kd=1.5).
        DeclareLaunchArgument("kp_low", default_value="150.0"),
        DeclareLaunchArgument("kd_low", default_value="12.0"),
        DeclareLaunchArgument("kp_wrist", default_value="40.0"),
        DeclareLaunchArgument("kd_wrist", default_value="4.0"),
        # arm_sdk-Gewichts-Rampe (0->1 beim Enable, 1->0 beim Disable). Auf dem
        # echten G1 Pflicht fuer eine weiche Uebergabe; die Sim setzt 0.0
        # (= sofort umschalten, bisheriges Verhalten).
        DeclareLaunchArgument("arm_weight_ramp_up_s", default_value="2.0"),
        DeclareLaunchArgument("arm_weight_ramp_down_s", default_value="2.0"),
        # Marker senden ihr Ziel sofort beim Ziehen (gruener Wuerfel). Steht das
        # auf false, sind die Wuerfel grau und stumm, bis man sie per Rechtsklick-
        # Menue "Enable publishing" aktiviert. Der Arm bewegt sich ohnehin nur,
        # wenn /g1pilot/arms/enabled true ist -> das bleibt die Sicherheitsschranke.
        DeclareLaunchArgument("marker_publish_default", default_value="true"),
        # Leader-Follower: Marker folgt der Hand im Idle (per Streamdeck/Topic schaltbar).
        DeclareLaunchArgument("marker_follow_ee", default_value="true"),

        Node(
            package='g1pilot',
            executable='arm_controller',
            name='arm_controller',
            parameters=[{
                'interface': interface,
                'use_robot': ParameterValue(use_robot, value_type=bool),
                'arm_weight_ramp_up_s': ParameterValue(
                    LaunchConfiguration("arm_weight_ramp_up_s"), value_type=float),
                'arm_weight_ramp_down_s': ParameterValue(
                    LaunchConfiguration("arm_weight_ramp_down_s"), value_type=float),
                'arm_velocity_limit': ParameterValue(
                    arm_velocity_limit, value_type=float),
                'ee_velocity_limit': ParameterValue(
                    LaunchConfiguration("ee_velocity_limit"), value_type=float),
                'self_collision_gate': ParameterValue(
                    LaunchConfiguration("self_collision_gate"), value_type=bool),
                'environment_collision_gate': ParameterValue(
                    LaunchConfiguration("environment_collision_gate"), value_type=bool),
                'planned_motion_tolerance': ParameterValue(
                    LaunchConfiguration("planned_motion_tolerance"), value_type=float),
                'kp_low': ParameterValue(LaunchConfiguration("kp_low"), value_type=float),
                'kd_low': ParameterValue(LaunchConfiguration("kd_low"), value_type=float),
                'kp_wrist': ParameterValue(LaunchConfiguration("kp_wrist"), value_type=float),
                'kd_wrist': ParameterValue(LaunchConfiguration("kd_wrist"), value_type=float),
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
                'marker_follow_ee': ParameterValue(
                    LaunchConfiguration("marker_follow_ee"), value_type=bool),
            }],
            output='screen'
        ),
    ])
