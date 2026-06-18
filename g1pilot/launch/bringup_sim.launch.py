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

    return LaunchDescription([

        # ── 1. robot_state: liest LowState_ von MuJoCo über DDS lo ──────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'robot_state_launcher.launch.py')
            ),
            launch_arguments={
                'use_robot':            'true',
                'interface':            'lo',
                'publish_joint_states': 'true',
                'sim_rate_hz':          '50.0',
                # rviz2 AUS: es frisst auf langsamen PCs so viele Kerne, dass die
                # 50-Hz-Regelschleife von loco_sim auf ~20 Hz einbricht (Policy
                # ueberschiesst -> Roboter driftet/faellt). Der Roboter ist im
                # MuJoCo-Fenster sichtbar; rviz2 wird zum Balancieren nicht gebraucht.
                'use_rviz':             'false',
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
            }.items()
        ),

        # ── 3. Teleoperation (Joystick): funktioniert unverändert im Sim ─
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'teleoperation_launcher.launch.py')
            ),
        ),

        # ── 4. loco_sim: RL-Loco-/Balance-Controller (ersetzt das Onboard-
        #    High-Level, das es in MuJoCo nicht gibt). Liest rt/lowstate,
        #    schreibt rt/lowcmd (Beine). START BALANCING -> Stehen/Balancieren.
        #    Real-Bringup nutzt weiter loco_client (Unitree-High-Level).
        Node(
            package='g1pilot',
            executable='loco_sim',
            name='loco_sim',
            output='screen',
            parameters=[{
                'interface': 'lo',
            }]
        ),
    ])
