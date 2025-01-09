import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取与拼接默认路径
    task_dir = get_package_share_directory('task2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')  # 获取nav2_bringup下的文件

    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true')
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(task_dir, 'maps', 'room1.yaml'))  # map的配置参数

    # 独立的参数文件路径为每个机器人创建单独的 Nav2 配置
    robot1_nav2_param_path = launch.substitutions.LaunchConfiguration(
        'robot1_params_file', default=os.path.join(task_dir, 'config', 'nav2_params.yaml'))
    robot2_nav2_param_path = launch.substitutions.LaunchConfiguration(
        'robot2_params_file', default=os.path.join(task_dir, 'config', 'robot2_nav2_params.yaml'))

    return launch.LaunchDescription([
        # 声明新的 Launch 参数
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('robot1_params_file', default_value=robot1_nav2_param_path,
                                             description='Full path to robot1 param file to load'),
        launch.actions.DeclareLaunchArgument('robot2_params_file', default_value=robot2_nav2_param_path,
                                             description='Full path to robot2 param file to load'),

        # 启动第一个机器人
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'namespace': 'trans_robot_1',
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': robot1_nav2_param_path}.items(),
        ),

        # 启动第二个机器人
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'namespace': 'trans_robot_2',
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': robot1_nav2_param_path}.items(),
        ),

        # 启动 RViz（只需一个）
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
