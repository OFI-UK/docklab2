#!/usr/bin/env python

# Code for launching Air Bearing Platform 2 (ABP2)
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='docklab2',
            namespace='abp2',
            executable='relay_control3',
            name='relay_server'
        )
    ])
