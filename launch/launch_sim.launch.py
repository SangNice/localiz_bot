import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'localiz_bot' 

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    # declare the file world path
    world_file = os.path.join(get_package_share_directory(package_name), 'worlds', 'myworld.sdf')
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                launch_arguments={'gz_args': f' -r {world_file}'}.items()
             )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'bot_name'],
                        output='screen')
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # syntax: <topic_name>@<msg_ROS2_type>[<msg_Gazebo_type>
            # the syntax '[' rule that the direction of date is one-way from Gazebo to ROS2
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        output='screen'
    )
    # Node 1: Bridge for cmd_vel to rule the robot by teleop_twist_keyboard
    cmd_vel_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )
    # Node 2: Bridge for Clock, Odometry and TF to synchronize RViz2 with Gazebo
    sync_rviz_gazebo_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/bot_name/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/bot_name/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
        ],
        remappings=[
            ('/model/bot_name/tf', '/tf'),
            ('/model/bot_name/odometry', '/odom')
        ],
        output='screen'
    )
    diff_drive_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_broad"],
    )
    # Node joint_state_publisher_gui added
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    return LaunchDescription([
        rsp,
        gazebo,
        gz_bridge_node,
        cmd_vel_bridge_node,            # Bridge velocity
        sync_rviz_gazebo_bridge_node,   #synchronize rviz
        joint_state_publisher_gui_node,
        diff_drive_spawner,
        joint_broad_spawner, # call UI node to contro joint
        TimerAction(
            period=3.0,
            actions=[spawn_entity]
        )
    ])