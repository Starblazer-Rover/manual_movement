from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='manual_movement', 
            executable='rover_movement',
        ),

        Node(
            package='manual_movement',
            executable='arm'
        )
    ])