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

    for planet in config["planetes"]:
        to_launch.append(Node(
            package='mas_solarsystem',
            name=planet,
            executable='mas_solarsystem',
            parameters=[config["planetes"][planet]]
#            remappings=[(planet, planet)]
        ))

    return LaunchDescription(to_launch)
