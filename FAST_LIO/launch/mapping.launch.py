import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    # --- ADD THIS SECTION ---
    # Static transform from fast_lio's 'body' frame to Unitree H1's 'lidar_link' frame.
    # *** IMPORTANT: REPLACE THESE DUMMY VALUES WITH YOUR ACTUAL MEASURED/CONFIGURED TRANSFORM ***
    # X, Y, Z are translations in meters.
    # Yaw, Pitch, Roll are rotations in radians. (Yaw around Z, Pitch around Y, Roll around X)
    # The parent frame is 'body' (fast_lio's output frame).
    # The child frame is 'lidar_link' (Unitree H1's URDF frame for the LiDAR).
    #
    # You MUST get these values correct for your physical setup!
    # Example values (replace these!):
    # <origin xyz="0.000489 0.002797 0.20484" rpy="0 0 0"/>
    #<origin xyz="0.051307 0 0.68218" rpy="0 -2.88869 0"/>
    transform_x = 0.2
    transform_y = 0.0
    transform_z = -0.71
    transform_yaw = 0.0 # 0 degrees
    transform_pitch = -0.253099 # 0 degrees
    transform_roll = 0.0 # 0 degrees

    static_tf_broadcaster_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_lidar_link_broadcaster',
        arguments=[
            str(transform_x),
            str(transform_y),
            str(transform_z),
            str(transform_yaw),
            str(transform_pitch),
            str(transform_roll),
            'body',         # Parent frame: This should be fast_lio's output frame
            'pelvis'    # Child frame: This should be the frame name in Unitree's URDF
        ],
        output='screen'
    )
    # --- END OF NEW SECTION ---


    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)

    # --- ADD THE NEW NODE TO THE LAUNCH DESCRIPTION ---
    ld.add_action(static_tf_broadcaster_node)
    # --- END ---

    return ld


    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)

    return ld
