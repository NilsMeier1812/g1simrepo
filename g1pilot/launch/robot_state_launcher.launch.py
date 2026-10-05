from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import os
import re

package_name = "g1pilot"
urdf_file_name = "g1_29dof_inspire_ftp.urdf"


def _low_gfx_robot(urdf, robot_desc):
    """Sparsame Grafik (G1_LOW_GFX=1): <visual>-Meshes des URDF auf vereinfachte
    Kopien umstellen (nur RViz-Anzeige; <collision> bleibt original, die IK liest
    ihr URDF ohnehin selbst). -> (urdf-Pfad, robot_description)."""
    try:
        from g1pilot.utils import lowpoly
        if not lowpoly.enabled():
            return urdf, robot_desc
        desc, n = lowpoly.lowpoly_urdf(robot_desc, get_package_share_directory)
        os.makedirs(lowpoly.CACHE_DIR, exist_ok=True)
        out = os.path.join(lowpoly.CACHE_DIR, "robot_lowgfx.urdf")
        with open(out, "w") as f:
            f.write(desc)
        print(f"[LOW_GFX] RViz-Robotermodell: {n} Visual-Mesh(es) vereinfacht.")
        return out, desc
    except Exception as e:   # Anzeige-Optimierung darf den Start nie verhindern
        print(f"[LOW_GFX] WARN: Robotermodell bleibt original ({e}).")
        return urdf, robot_desc


def _rviz_node(context):
    """RViz mit der gewaehlten Config. Bei Sparsamer Grafik eine Kopie mit
    normaler Fenstergroesse (die Configs oeffnen sonst 2560x1403 auf einem
    zweiten Monitor -- bei schwacher Grafik kostet jedes Pixel)."""
    name = LaunchConfiguration("rviz_config").perform(context)
    path = os.path.join(get_package_share_directory(package_name), "config", name)
    if os.environ.get("G1_LOW_GFX", "0").strip().lower() in ("1", "true", "yes", "on"):
        try:
            with open(path) as f:
                cfg = f.read()
            for key, val in (("Width", 1600), ("Height", 900), ("X", 0), ("Y", 0)):
                cfg = re.sub(rf"(\nWindow Geometry:(?:\n  .*)*?\n  {key}: )-?\d+",
                             rf"\g<1>{val}", cfg, count=1)
            out_dir = os.environ.get("G1_LOW_GFX_CACHE", "/tmp/g1_lowpoly")
            os.makedirs(out_dir, exist_ok=True)
            path = os.path.join(out_dir, "lowgfx_" + name)
            with open(path, "w") as f:
                f.write(cfg)
        except OSError as e:
            print(f"[LOW_GFX] WARN: RViz-Config bleibt original ({e}).")
    return [Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", path],
    )]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_robot = LaunchConfiguration("use_robot")
    publish_joint_states = LaunchConfiguration("publish_joint_states")
    publish_hand_joints = LaunchConfiguration("publish_hand_joints")
    interface = LaunchConfiguration("interface")
    sim_rate_hz = LaunchConfiguration("sim_rate_hz")
    use_rviz = LaunchConfiguration("use_rviz")
    enable_mola = LaunchConfiguration("enable_mola")

    urdf = os.path.join(
        get_package_share_directory(package_name), "description_files/urdf", urdf_file_name
    )
    with open(urdf, "r") as infp:
        robot_desc = infp.read()
    urdf, robot_desc = _low_gfx_robot(urdf, robot_desc)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Use simulation (Gazebo) clock if true"),
        DeclareLaunchArgument("use_robot", default_value="true",
                              description="Connect to real robot if true"),
        DeclareLaunchArgument("publish_joint_states", default_value="true",
                              description="Publish joint_states from node"),
        DeclareLaunchArgument("publish_hand_joints", default_value="true",
                              description="Finger-Gelenke als Default 0.0 mitpublizieren. "
                                          "Auf false setzen, wenn die Inspire-FTP-Bridge "
                                          "(inspire_hand) die Finger uebernimmt."),
        DeclareLaunchArgument("interface", default_value="eth0",
                              description="Network interface for Unitree SDK"),
        DeclareLaunchArgument("sim_rate_hz", default_value="50.0",
                              description="Simulation rate when use_robot=false"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Launch RViz2. Im Loco-Sim aus (CPU sparen: "
                                          "rviz2 frisst Kerne und laesst die 50-Hz-"
                                          "Regelschleife einbrechen)."),
        DeclareLaunchArgument("arm_controlled", default_value="both",
                                description="Which arm to control: 'left', 'right', or 'both'"),
        # MOLA-Odometrie-Fix-Node. Default an (Sim/Full-Stack). Im schlanken
        # Real-Modus OHNE LiDAR abschaltbar (bringup_real setzt false): der Node
        # wartet sonst nur idle auf /lidar_odometry/pose, das nie kommt.
        DeclareLaunchArgument("enable_mola", default_value="true",
                              description="mola_fixed-Node starten (MOLA-Odometrie-TF). "
                                          "Ohne LiDAR (schlanker Real-Modus) auf false."),
        # RViz-Config-Dateiname (in share/g1pilot/config). Nav nutzt nav.rviz
        # (Fixed Frame 'map' + Ziel-Werkzeug auf /g1pilot/goal); sonst 29dof.rviz.
        DeclareLaunchArgument("rviz_config", default_value="29dof.rviz",
                              description="RViz-Config in config/ (29dof.rviz | nav.rviz)"),

        Node(
            package='g1pilot',
            executable='robot_state',
            name='robot_state',
            parameters=[{
                'interface': interface,
                'use_robot': ParameterValue(use_robot, value_type=bool),
                'sim_rate_hz': ParameterValue(sim_rate_hz, value_type=float),
                'publish_joint_states': ParameterValue(publish_joint_states, value_type=bool),
                'publish_hand_joints': ParameterValue(publish_hand_joints, value_type=bool),
            }],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='mola_fixed',
            name='mola_fixed',
            condition=IfCondition(enable_mola),
            parameters=[{
            }],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mid360_to_livox_tf',
            arguments=['0','0','0','0','0','3.14159265','mid360_link','livox_frame']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='d435_to_camera_link',
            arguments=['0','0','0','0','0','0','d435_link','camera_link']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_odom_tf',
            arguments=['0','0','0','0','0','0','world','odom_unitree']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='pelvis_to_base_link_tf',
            arguments=['0','0','0','0','0','0','base_link','pelvis']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mrbeam_to_pelvis_tf',
            arguments=['0.0745','0.0','0.065','0','0.05236','0','waist_roll_link','mrbeam_link']
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "robot_description": robot_desc
            }],
            arguments=[urdf],
        ),

        OpaqueFunction(function=_rviz_node, condition=IfCondition(use_rviz)),
    ])
