import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # パス設定
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # ユーザーディレクトリの取得
    home_dir = os.path.expanduser('~')
    
    # デフォルトの設定ファイルパス
    default_map_file = os.path.join(home_dir, 'go2_simulation', 'my_map.yaml')
    params_file = os.path.join(home_dir, 'go2_simulation', 'go2_nav2_params.yaml')

    # Launch引数の受け取り変数を作成
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file_cmd = LaunchConfiguration('params_file')

    # --- 引数の宣言 (ここで外部からの入力を受け付ける定義をする) ---
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    return LaunchDescription([
        # 宣言した引数を登録
        declare_use_sim_time_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,

        # Nav2のBringupを呼び出す
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_yaml_file,           # 引数で渡された地図を使う
                'params_file': params_file_cmd, # 引数で渡された設定を使う
                'use_sim_time': use_sim_time,
                'autostart': 'true'
            }.items(),
        ),

        # RViz2 (Nav2用の設定で起動)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_nav',
            arguments=['-d', os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])
