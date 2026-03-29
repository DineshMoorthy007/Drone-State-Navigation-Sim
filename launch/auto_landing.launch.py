import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    world_file_path = os.path.expanduser('~/drone_ws/src/auto_landing/worlds/landing_world.sdf')

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file_path],
        additional_env={'QT_QPA_PLATFORM': 'xcb'},
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/camera/image_annotated@sensor_msgs/msg/Image]gz.msgs.Image' # <-- Sends HUD back to Gazebo
        ],
        output='screen'
    )

    mission_commander = Node(
        package='auto_landing',
        executable='mission_commander',
        output='screen'
    )

    flight_controller = Node(
        package='auto_landing',
        executable='flight_controller',
        output='screen'
    )

    target_detector = Node(
        package='auto_landing',
        executable='target_detector',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge,
        mission_commander,
        flight_controller,
        target_detector
    ])
