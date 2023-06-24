# example.launch.py

import os
import json

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    json_file_name = 'launch/config.json'
    json_file_path = os.path.join(
        get_package_share_directory('mas_solarsystem'),
        json_file_name)
    with open(json_file_path, 'r') as file_handle:
        config = json.load(file_handle)
   
    to_launch = []
    a = 0
    for name, planet in config["planetes"].items():
        dict_planet = config["planetes"][name]

        to_send = {"name": name}
        to_send.update(dict_planet)
        to_send["id"] = a
        to_send["r_scale"] = 0.001
        to_send["d_scale"] = 0.00000001


        to_launch.append(Node(
            package='mas_solarsystem',
            name=name,
            executable='celestial_body_node',
            parameters=[to_send],
            remappings=[("/marker", f"/{name}_marker")]
            )
        )
        a += 1

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('mas_solarsystem'), 'launch', 'config.rviz')],
    )

    world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'Soleil'],
    )
    to_launch.append(world)
    to_launch.append(rviz)
    return LaunchDescription(to_launch)
