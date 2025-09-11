# ============================================================
# File: setup.py
# Project: Doppler ICP ROS 2 Node
# Role: Python setup script for packaging and installation
#
# Description:
#   - Defines package metadata (name, version, maintainer, license)
#   - Declares dependencies (runtime + test)
#   - Registers data files (launch/config/package.xml)
#   - Provides entry point for ROS 2 executable (farness_dicp_node)
#
# Author: Farness AI
# Copyright: 2025
# ============================================================
from setuptools import find_packages, setup

package_name = 'farness_dicp'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dicp_stitcher.launch.py']),
        ('share/' + package_name + '/config', ['config/dicp_params.yaml']),
    ],
    install_requires=[
        'setuptools',
        'open3d',
        'pandas',
        'scikit-learn'
    ],
    zip_safe=True,
    maintainer='farness',
    maintainer_email='farness@todo.todo',
    description='Doppler ICP stitching and LiDAR mapping node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'farness_dicp_node = farness_dicp.main:main'
        ],
    },
)