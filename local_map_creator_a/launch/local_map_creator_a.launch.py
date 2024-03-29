import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

  return LaunchDescription([

      Node(
      package='local_map_creator_a',
      executable='local_map_creator_a_node',
      name='local_map_creator_a_node',
      parameters=[{'use_sim_time' : True}],
    )
  ])  # return LaunchDescription 