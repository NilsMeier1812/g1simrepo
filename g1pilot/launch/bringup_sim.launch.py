"""
bringup_sim.launch.py
Startet G1Pilot für MuJoCo-Simulation.

Unterschied zu bringup_launcher.launch.py:
  - robot_state + arm_controller: use_robot=true, interface=lo
    → kommunizieren über DDS Loopback mit MuJoCo
  - loco_sim: RL-Policy-Loco-/Balance-Controller (ersetzt das Unitree-Onboard-
    High-Level, das es in MuJoCo nicht gibt) → rt/lowstate→Policy→rt/lowcmd
  - KEIN livox, KEIN mola (nicht nötig im Sim)
"""
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():
    pkg_share = FindPackageShare('g1pilot').find('g1pilot')

    # ── Konfiguration ueber Umgebungsvariablen (vom Start-Menue start.sh
    #    gesetzt; Defaults = bisheriges Verhalten). ───────────────────────
    #  USE_RVIZ : "true"/"false" — RViz mitstarten. Default false. Dank LOCKSTEP
    #             (Regelrate haengt am rt/lowstate-Eingang, nicht an der Wall-Clock)
    #             bricht das Balancieren NICHT mehr ein, wenn RViz Kerne frisst —
    #             darum ist RViz jetzt wieder gefahrlos zuschaltbar.
    use_rviz = os.environ.get('USE_RVIZ', 'false').strip().lower()

    #  G1_INSPIRE_HANDS : "1"/"0" — Master-Schalter Inspire-FTP-Haende. Waehlt im
    #  MuJoCo das Finger-Modell (config.py) UND startet hier die Inspire-FTP-Hand-
    #  Bridge (HTML-GUIs + DDS rt/inspire/cmd|state, backend='mujoco'). robot_state
    #  gibt die Finger dann an die Bridge ab (publish_hand_joints=false), sonst gaebe
    #  es zwei /joint_states-Quellen fuer dieselben Gelenke.
    inspire_hands = os.environ.get('G1_INSPIRE_HANDS', '0').strip().lower() in ('1', 'true', 'yes', 'on')

    nodes = [

        # ── 1. robot_state: liest LowState_ von MuJoCo über DDS lo ──────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'robot_state_launcher.launch.py')
            ),
            launch_arguments={
                'use_robot':            'true',
                'interface':            'lo',
                'publish_joint_states': 'true',
                # Finger gibt robot_state nur ab, wenn die Inspire-Bridge sie uebernimmt.
                'publish_hand_joints':  'false' if inspire_hands else 'true',
                'sim_rate_hz':          '50.0',
                # rviz2 standardmaessig AUS (USE_RVIZ=false). Frueher brach RViz die
                # 50-Hz-Regelschleife ein; unter LOCKSTEP ist das nicht mehr der Fall
                # (Regelrate haengt am rt/lowstate-Eingang). Per Start-Menue/USE_RVIZ
                # zuschaltbar. Der Roboter ist ohnehin im MuJoCo-Fenster sichtbar.
                'use_rviz':             use_rviz,
            }.items()
        ),

        # Statische TF: odom_unitree → base_link (Identity, da Physik in MuJoCo)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link_sim_tf',
            arguments=['0','0','0','0','0','0','odom_unitree','base_link']
        ),

        # ── 2. arm_controller: schreibt LowCmd_ zu MuJoCo über DDS lo ───
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'manipulation_launcher.launch.py')
            ),
            launch_arguments={
                'use_robot': 'true',
                'interface': 'lo',
                # Sim: arm_sdk-Gewicht sofort umschalten (bisheriges Verhalten,
                # der MuJoCo-Bridge-Blend ist auf 0/1 getestet). Die Rampe ist
                # fuer den echten G1 (bringup_real nutzt die Defaults 2.0 s).
                'arm_weight_ramp_up_s': '0.0',
                'arm_weight_ramp_down_s': '0.0',
            }.items()
        ),

        # ── 3. Teleoperation (Joystick): funktioniert unverändert im Sim ─
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'teleoperation_launcher.launch.py')
            ),
        ),

        # ── 4. loco_sim: Whole-Body-Loco-/Balance-Controller (ersetzt das Onboard-
        #    High-Level, das es in MuJoCo nicht gibt). Liest rt/lowstate, schreibt
        #    rt/lowcmd (alle 29 Gelenke). Velocity-konditionierte Policy:
        #    START BALANCING -> stehen (cmd=0), loco_cmd_vel -> laufen.
        #    Real-Bringup nutzt weiter loco_client (Unitree-High-Level).
        Node(
            package='g1pilot',
            executable='loco_sim',
            name='loco_sim',
            output='screen',
            parameters=[{
                'interface': 'lo',
                'policy':    'g1_wholebody',
            }]
        ),
    ]

    # ── 5. Inspire-FTP-Hand-Bridge (nur bei Inspire-Haenden) ────────────
    #    Bedient die HTML-GUIs (Controller :8766 / Viewer :8765), bewegt die Finger in
    #    RViz (/joint_states) UND redet per DDS (rt/inspire/cmd|state) mit der MuJoCo-
    #    Sim: backend='mujoco' -> echte Finger-Winkel + Touch-Kraefte in den GUIs.
    if inspire_hands:
        nodes.append(Node(
            package='g1pilot',
            executable='inspire_hand',
            name='inspire_ftp_bridge',
            output='screen',
            parameters=[{
                'ws_host':        '0.0.0.0',
                'controller_port': 8766,
                'viewer_port':     8765,
                'update_rate_hz':  50.0,
                'backend':         'mujoco',
                'interface':       'lo',
            }]
        ))

    return LaunchDescription(nodes)
