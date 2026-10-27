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

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                launch_arguments={'gz_args': 'empty.sdf -r'}.items()
             )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'bot_name'],
                        output='screen')
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Cú pháp: <tên_topic>@<Kiểu_msg_ROS2>[<Kiểu_msg_Gazebo>
            # Dấu '[' quy định hướng truyền dữ liệu một chiều từ Gazebo sang ROS 2
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        output='screen'
    )
    # Node 1: Bridge cho cmd_vel để điều khiển xe bằng teleop_twist_keyboard
    cmd_vel_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )
    # Node 2: Bridge cho Clock, Odometry và TF để đồng bộ RViz2 với Gazebo
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

    # Node joint_state_publisher_gui được thêm vào
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
        cmd_vel_bridge_node,            # Bridge vận tốc
        sync_rviz_gazebo_bridge_node,   #đồng bộ rviz
        joint_state_publisher_gui_node, # Gọi node giao diện điều khiển khớp
        TimerAction(
            period=3.0,
            actions=[spawn_entity]
        )
    ])