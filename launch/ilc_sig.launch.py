import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    ilc_jnt_control = Node(
        package="controller_manager",
        executable="spawner",
        #arguments=['GazeboSystem'],
        arguments=["ilc_sig_controller", "--controller-manager", "/controller_manager"],
    )

    IMU_Broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        #arguments=['GazeboSystem'],
        arguments=["IMU_Broadcaster", "--controller-manager", "/controller_manager"],
    )


    return LaunchDescription([

  joint_state_broadcaster_spawner,
  ilc_jnt_control,
  IMU_Broadcaster
     
  ])