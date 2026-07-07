"""
bringup_real.launch.py
Schlankes Bringup fuer den ECHTEN G1 (erster Hardware-Betrieb):
Arm-Manipulation (IK/rviz-Marker) + Inspire-FTP-Haende (Modbus TCP) +
Unitree-eigener Loco-Controller (Balance/Gehen via loco_client).

Unterschied zu bringup_launcher.launch.py (Alt/Voll-Stack):
  - KEIN Livox/MOLA/Nav by default (per G1_ENABLE_LIDAR=1 zuschaltbar)
  - loco_client direkt als Node (mit Walk-Safety-Parametern) statt ueber
    navigation_launcher (der auch nav2point/dijkstra hochziehen wuerde)
  - komplett env-getrieben (vom start.sh-Real-Zweig gesetzt), keine
    Launch-Argumente, die verloren gehen koennen

DDS-Konvention (utils/common.py): G1_SIM_MODE=false -> Unitree-DDS auf
Domain 0 ueber ROBOT_INTERFACE; der ROS-Graph laeuft daneben auf
ROS_DOMAIN_ID=1 (loopback, cyclonedds.xml) -- sonst kollidieren beide
CycloneDDS-Instanzen im selben Prozess ("create domain error").
"""
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
import os


def _env_bool(name, default='0'):
    return os.environ.get(name, default).strip().lower() in ('1', 'true', 'yes', 'on')


def generate_launch_description():
    pkg_share = FindPackageShare('g1pilot').find('g1pilot')

    # ── Konfiguration ueber Umgebungsvariablen (start.sh-Real-Zweig) ─────
    interface = os.environ.get('ROBOT_INTERFACE', 'eth0').strip()
    use_rviz = os.environ.get('USE_RVIZ', 'true').strip().lower()
    inspire_hands = _env_bool('G1_INSPIRE_HANDS')
    left_host = os.environ.get('G1_HAND_LEFT_HOST', '192.168.123.210').strip()
    right_host = os.environ.get('G1_HAND_RIGHT_HOST', '192.168.123.211').strip()
    hand_port = int(os.environ.get('G1_HAND_PORT', '6000'))
    joystick_name = os.environ.get('JOYSTICK_NAME', 'Wireless Controller').strip()
    enable_lidar = _env_bool('G1_ENABLE_LIDAR')
    # Konservative Geh-Limits fuer die ersten Tests (normierte UI-Kommandos
    # werden in loco_client damit skaliert; gilt auch fuer den PS4-Pfad).
    max_vx = os.environ.get('G1_MAX_VX', '0.4')
    max_vy = os.environ.get('G1_MAX_VY', '0.3')
    max_vyaw = os.environ.get('G1_MAX_VYAW', '0.4')

    nodes = [

        # ── 1. robot_state: rt/lowstate -> /joint_states + IMU + TFs + RViz ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'robot_state_launcher.launch.py')
            ),
            launch_arguments={
                'use_robot':            'true',
                'interface':            interface,
                'publish_joint_states': 'true',
                # Finger gibt robot_state nur ab, wenn die Inspire-Bridge sie uebernimmt.
                'publish_hand_joints':  'false' if inspire_hands else 'true',
                # RViz default AN: der rviz-Marker ist auf real das Interface
                # fuer die Arm-Manipulation.
                'use_rviz':             use_rviz,
            }.items()
        ),

        # Statische TF: odom_unitree -> base_link (Identity; ohne LiDAR-Odometrie
        # gibt es keine bewegte odom-Quelle -- RViz braucht den Frame trotzdem).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link_real_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'odom_unitree', 'base_link']
        ),

        # ── 2. arm_controller + interactive_marker (rt/arm_sdk) ─────────────
        #    Gewichts-Rampe AKTIV (Defaults 2.0 s) -- weiche Uebergabe zwischen
        #    Roboter-eigener Armsteuerung und arm_sdk. (Sim setzt hier 0.0.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'manipulation_launcher.launch.py')
            ),
            launch_arguments={
                'use_robot': 'true',
                'interface': interface,
            }.items()
        ),

        # ── 3. Teleoperation: Streamdeck-UI + PS4-Controller (evdev) ────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'teleoperation_launcher.launch.py')
            ),
            launch_arguments={
                'joystick_name': joystick_name,
            }.items()
        ),

        # ── 4. loco_client: Unitree-Onboard-High-Level (Balance/Gehen) ──────
        #    START -> FSM 4, START BALANCING -> BalanceStand, WALK/loco_cmd_vel
        #    bzw. PS4-Deadman -> Move(). damp_on_init bleibt false: der Node
        #    greift beim Start NICHT in den Roboter-Zustand ein.
        Node(
            package='g1pilot',
            executable='loco_client',
            name='loco_client',
            output='screen',
            parameters=[{
                'interface':       interface,
                'use_robot':       True,
                'max_vx':          float(max_vx),
                'max_vy':          float(max_vy),
                'max_vyaw':        float(max_vyaw),
                'cmd_vel_timeout': 0.5,
                'damp_on_init':    False,
            }]
        ),
    ]

    # ── 5. Inspire-FTP-Hand-Bridge: ECHTE Haende via Modbus TCP ─────────────
    #    GUIs identisch zur Sim (Controller :8766 / Viewer :8765 / HTTP :8767);
    #    Ist-Winkel/Kraefte/Taktil kommen von der Hardware, RViz spiegelt die
    #    echten Finger ueber /joint_states.
    if inspire_hands:
        nodes.append(Node(
            package='g1pilot',
            executable='inspire_hand',
            name='inspire_ftp_bridge',
            output='screen',
            parameters=[{
                'ws_host':         '0.0.0.0',
                'controller_port': 8766,
                'viewer_port':     8765,
                'update_rate_hz':  50.0,
                'backend':         'modbus',
                'left_host':       left_host,
                'right_host':      right_host,
                'modbus_port':     hand_port,
            }]
        ))

    # ── 6. Optional: LiDAR (+ Navigation) zuschalten ─────────────────────────
    #    Braucht den Livox MID360 + das VOLLE Docker-Image (g1pilot:v1.1.0 mit
    #    livox_ros_driver2/MOLA) -- im schlanken Real-Image schlaegt der Include
    #    fehl. nav2point/dijkstra direkt als Nodes (navigation_launcher wuerde
    #    loco_client ein zweites Mal starten).
    if enable_lidar:
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'livox_launcher.launch.py'))))
        nodes.append(Node(
            package='g1pilot', executable='nav2point', name='nav2point',
            output='screen',
            parameters=[{'interface': interface, 'use_robot': True}]))
        nodes.append(Node(
            package='g1pilot', executable='dijkstra_planner', name='dijkstra_planner',
            output='screen',
            parameters=[{'interface': interface, 'use_robot': True}]))

    return LaunchDescription(nodes)
