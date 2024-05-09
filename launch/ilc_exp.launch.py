import imp
import os
import yaml

from launch import LaunchDescription,LaunchContext
from launch.events.process import ProcessStarted
from launch.actions import DeclareLaunchArgument,RegisterEventHandler,ExecuteProcess,LogInfo,EmitEvent
from launch.substitutions import Command,LaunchConfiguration
from launch.event_handlers import OnProcessStart,OnProcessIO,OnProcessExit,OnShutdown,OnExecutionComplete

from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xml.etree.ElementTree as ET
import subprocess
import time


def generate_launch_description():
    
    subprocess.check_output(
        ["ros2 control load_controller state_broadcaster --set-state active"]
        ,shell=True)
    subprocess.check_output(
        ["ros2 control load_controller ilc_sig_controller --set-state active "]
        ,shell=True)

    bag_node = Node(
        package="ilc_sig_node_pkg",
        executable="ilc_sig_node_pkg",
        output="screen",
        )
        
    return LaunchDescription(
        [bag_node]
  	)
      
    
