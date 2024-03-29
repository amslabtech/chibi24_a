from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle_detector_a',
            namespace='obstacle_detector_a',
            executable='obstacle_detector_a_node',
            output="screen",
            parameters=[{'use_sim_time':True}],
        ),
        Node(
            package='a_localizer',
            namespace='a_localizer',
            executable='a_localizer_node',
            output="screen",
            parameters=[{'use_sim_time':True}],
        ),
        Node(
            package='global_path_planner_a',
            namespace='global_path_planner_a',
            executable='global_path_planner_a_node',
            output="screen",
            parameters=[{'use_sim_time':True}],
        ),
        Node(
            package='local_goal_creator_a',
            namespace='local_goal_creator_a',
            executable='local_goal_creator_a_node',
            output="screen",
            parameters=[{'use_sim_time':True}],
        ),
        Node(
            package='local_path_planner_a',
            namespace='local_path_planner_a',
            executable='local_path_planner_a_node',
            output="screen",
            parameters=[{'use_sim_time':True}],
        ),
    ])