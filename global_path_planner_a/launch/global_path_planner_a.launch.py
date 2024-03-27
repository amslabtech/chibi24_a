from launch import LaunchDescription
from launch_ros.actions import Node
import os

pkg_name1 = 'global_path_planner_a'

def generate_launch_description():
    ld = LaunchDescription()

    config1 = os.path.join(
        pkg_name1,
        'config',
        'param',
        'global_path_planner_a.yaml'
    )

    node1 = Node(
        package=pkg_name1,
        executable='global_path_planner_a',
        name='global_path_planner_a_node',
        output='screen',
        emulate_tty=True,
        parameters=[config1]
    )


    node2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', '(find-pkg-share global_path_planner_a)/config/rviz/global_path_planner_a.rviz'])


    node3 = Node(
        package='map_server',
        executable='map_server',
        name='map_server',
        output='screen',
    )

    ld.add_action(node1)
    ld.add_action(node2)
    ld.add_action(node3)
# 複数ノードを追加する場合は，configN,nodeNを作ってld.add_action(nodeN)?

    return ld

<launch>
    <!-- config file path  -->
    <arg name="rviz_settings" default="$(find global_path_planner_a)/config/rviz/global_path_planner_a.rviz"/>
    <arg name="map_settings" default="$(find global_path_planner_a)/map/map.yaml"/>
    <arg name="global_path_planner_a_settings" default="$(find global_path_planner_a)/config/param/global_path_planner_a.yaml"/>
<!-- findはない-->
    <!-- node launch -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(arg rviz_settings)"/>
    <node pkg="global_path_planner_a" type="global_path_planner_a_node" name="global_path_planner_a" output="screen">
        <param from = "load" file="$(arg global_path_planner_a_settings)"/>
    </node>
    <node pkg="map_server" type="map_server" name="map_server" args="$(arg map_settings)" output="screen"/>


    <node pkg="global_path_planner_a" exec="global_path_planner_a_node"/>
        <param from = "load" file="$(arg global_path_planner_a_settings)"/>
</launch>


