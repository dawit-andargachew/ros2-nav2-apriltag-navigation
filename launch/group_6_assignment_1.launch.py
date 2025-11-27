"""
launch wrapper for entire pipeline
1. launches sim / setup ros2 launch ir_launch assignment_1.launch.py​
2. launches apriltag detection node ros2 launch group_6_assignment_1 apriltag_wrapper.launch.yml​
3. launches navigation_orchestrator, goal_detector, initial_pose_sub, table_detector_server nodes

"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get package share directories
    from ament_index_python.packages import get_package_share_directory
    ir_launch_dir = os.path.join(get_package_share_directory('ir_launch'), 'launch')
    group_6_launch_dir = os.path.join(get_package_share_directory('group_6_assignment_1'), 'launch')

    # 1. Launch sim/setup
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ir_launch_dir, 'assignment_1.launch.py')),
        launch_arguments={'autostart': 'True'}.items()
    )

    # 2. Launch apriltag detection node
    apriltag_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(group_6_launch_dir, 'apriltag_wrapper.launch.yml'))
    )

    # 3. Launch other nodes
    navigation_orchestrator = Node(
        package='group_6_assignment_1',
        executable='navigation_orchestrator',
        name='navigation_orchestrator'
    )
    # Load goal_detector parameters from package config if present
    goal_detector = Node(
        package='group_6_assignment_1',
        executable='goal_detector',
        name='goal_detector',
        parameters=[os.path.join(get_package_share_directory('group_6_assignment_1'), 'config', 'goal_detector.yaml')]
    )
    initial_pose_pub = Node(
        package='group_6_assignment_1',
        executable='initial_pose_pub',
        name='initial_pose_pub'
    )
    table_detection_server = Node(
        package='group_6_assignment_1',
        executable='table_detection_server',
        name='table_detection_server'
    )

    return LaunchDescription([
        sim_launch,
        apriltag_launch,
        navigation_orchestrator,
        goal_detector,
        initial_pose_pub,
        table_detection_server
    ])