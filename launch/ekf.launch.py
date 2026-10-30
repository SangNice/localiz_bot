import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    #"""
    #Generate a launch description for the EKF node.
 
    #This function creates and returns a LaunchDescription object that will start
    #the EKF node from the robot_localization package. The node is configured
    #using parameters from a YAML file.
 
    #Returns:
    #    LaunchDescription: A complete launch description for the EKF node
    #"""
    # Constants for paths to different files and folders

    # Set the path to different packages
    pkg_share = get_package_share_directory('localiz_bot')
    # Config file paths
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Declare the launch arguments
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odometry/filtered')]
        )
    ])