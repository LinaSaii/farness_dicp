# ============================================================
# File: dicp_stitcher.launch.py
# Project: Doppler ICP ROS 2 Node
# Role: Launch description for Doppler-ICP stitching pipeline
#
# Description:
#   - Starts the farness_dicp_node
#   - Loads parameters from dicp_params.yaml
#   - Publishes stitched cloud, pose, trajectory, and dynamics
#
# Author: Farness AI
# Copyright: 2025
# ============================================================

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='farness_dicp',
            executable='farness_dicp_node',
            name='doppler_icp_stitcher',
            output='screen',
            parameters=[os.path.join(
                os.path.dirname(__file__),
                '../config/dicp_params.yaml'
            )]
        )
    ])