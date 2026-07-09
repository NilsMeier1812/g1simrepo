"""
bringup_launcher.launch.py — VOLLER Real-Stack (Livox + MOLA-Nav + alles).

HINWEIS: Fuer den ersten Hardware-Betrieb (Arme + Haende + Unitree-Loco,
ohne LiDAR) ist launch/bringup_real.launch.py der empfohlene Einstieg —
schlanker, env-getrieben und mit den Walk-Safety-Parametern verdrahtet.
Dieses Launchfile bleibt fuer den vollen Navigations-Stack (braucht das
grosse Docker-Image mit livox_ros_driver2/MOLA und den MID360-LiDAR).

Frueher wurden hier use_robot/interface zwar von docker-compose uebergeben,
aber NICHT an die Includes weitergereicht (stillschweigend verworfen ->
jeder Launcher fiel auf seine eigenen Defaults zurueck). Jetzt werden beide
deklariert und explizit durchgereicht.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg1_share = FindPackageShare('g1pilot').find('g1pilot')

    livox_launcher = os.path.join(pkg1_share, 'launch', 'livox_launcher.launch.py')
    navigation_launcher = os.path.join(pkg1_share, 'launch', 'navigation_launcher.launch.py')
    robot_state_launcher = os.path.join(pkg1_share, 'launch', 'robot_state_launcher.launch.py')
    teleoperation_launcher = os.path.join(pkg1_share, 'launch', 'teleoperation_launcher.launch.py')
    manipulation_launcher = os.path.join(pkg1_share, 'launch', 'manipulation_launcher.launch.py')

    use_robot = LaunchConfiguration('use_robot')
    interface = LaunchConfiguration('interface')
    fwd = {'use_robot': use_robot, 'interface': interface}

    return LaunchDescription([
        DeclareLaunchArgument('use_robot', default_value='true'),
        DeclareLaunchArgument('interface', default_value='eth0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(livox_launcher)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launcher),
            launch_arguments=fwd.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_state_launcher),
            launch_arguments=fwd.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleoperation_launcher)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manipulation_launcher),
            launch_arguments=fwd.items()
        ),
    ])
