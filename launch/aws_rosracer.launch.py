import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('ucsd_robo_car_aws_deepracer'),
        'config',
        'deepracer_config.yaml')

    lane_detection_node = Node(
        package='ucsd_robo_car_aws_deepracer',
        executable='lane_detection_node',
        parameters=[config])

    lane_guidance_node = Node(
        package='ucsd_robo_car_aws_deepracer',
        executable='lane_guidance_node',
        parameters=[config])

    ld.add_action(lane_detection_node)
    ld.add_action(lane_guidance_node)
    return ld
