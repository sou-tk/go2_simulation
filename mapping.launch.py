import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    # ユーザーのホームディレクトリを取得 (/home/username)
    home_dir = os.path.expanduser('~')
    
    # Pythonスクリプトのパス (ここを実際の場所に合わせる)
    script_path = os.path.join(home_dir, 'go2_simulation', 'true_odom_publisher.py')

    return LaunchDescription([
        
        # --- 1. PointCloud to LaserScan (点群 -> 2Dスキャン) ---
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
                'qos_reliability': 1, # 指定された値 (もしデータ来なければ 2 に変更)
                'use_sim_time': True
            }],
            remappings=[
                ('cloud_in', '/mid360/points')
            ]
        ),

        # --- 2. TF配信スクリプト (true_odom_publisher.py) ---
        # 実行権限がなくても python3 コマンドで直接叩く方式
        ExecuteProcess(
            cmd=['python3', script_path],
            output='screen'
        ),

        # --- 3. SLAM Toolbox (地図作成) ---
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'base_frame': 'base_footprint',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'map_update_interval': 0.1,
                'minimum_travel_distance': 0.0,
                'transform_timeout': 0.5
            }]
        ),

        # --- 4. RViz2 (可視化) ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])