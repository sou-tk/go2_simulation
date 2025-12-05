import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    home_dir = os.path.expanduser('~')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    script_path = os.path.join(home_dir, 'go2_simulation', 'true_odom_publisher.py')
    params_file = os.path.join(home_dir, 'go2_simulation', 'no_map_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file_cmd = LaunchConfiguration('params_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', default_value=params_file)

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_params_file_cmd,

        # 1. PointCloud -> LaserScan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'base_link',
                'min_height': 0.1,
                'max_height': 1.0,
                'angle_min': -3.1415,
                'angle_max': 3.1415,
                'angle_increment': 0.0087,
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 30.0,
                'use_inf': True,
                'qos_reliability': 2,
                'use_sim_time': use_sim_time
            }],
            remappings=[('cloud_in', '/mid360/points')]
        ),

        # 2. TF配信 (odom -> base_link)
        ExecuteProcess(
            cmd=['python3', script_path],
            output='screen'
        ),

        # ★追加: TF配信 (map -> odom)
        # これで map フレームが存在することになります
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # 3. Nav2 (Navigationのみ起動)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file_cmd,
                'autostart': 'true'
            }.items(),
        ),

        # 4. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_nav',
            arguments=['-d', os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])