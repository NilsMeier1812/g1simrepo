from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    joystick_name = LaunchConfiguration('joystick_name')
    return LaunchDescription([
        DeclareLaunchArgument(
            'joystick_name', default_value='Pro Controller',
            description='Name of the joystick device to bind to'),

        Node(
            package='g1pilot',
            executable='joystick',
            name='joystick',
            output='screen',
            parameters=[
                {'joystick_name': joystick_name}],
        ),

        Node(
            package='g1pilot',
            executable='joy_mux',
            name='joy_mux',
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='ui_interface',
            name='ui_interface',
            output='screen'
        ),

    ])
