import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
 return LaunchDescription([
    Node(
        package='obstacle_detector_a',
        executable='obstacle_detector_a_node',
        name='obstacle_detector_a_node',
        parameters=[{'use_sim_time' : True}],
    )
  ])  # return LaunchDescription 