import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取与拼接默认路径
    task_dir = get_package_share_directory('task2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')  # 获取nav2_bringup下的文件
    
    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    
    # 为每个机器人设置地图路径，使用相同的地图
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map_yaml', default=os.path.join(task_dir, 'maps', 'room1.yaml'))  # 共享地图
    
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(task_dir, 'config', 'nav2_params.yaml'))

    # 设置命名空间和机器人配置文件
    robot1_namespace = 'robot1'
    robot2_namespace = 'robot2'

    return launch.LaunchDescription([
        # 声明新的 Launch 参数
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map_yaml', default_value=map_yaml_path,
                                             description='Full path to map file to load for both robots'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),


        # 启动第一个机器人
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,  # 使用相同的地图
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path,
                'namespace': robot1_namespace
            }.items(),
        ),

        # 启动第二个机器人
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,  # 使用相同的地图
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path,
                'namespace': robot2_namespace
            }.items(),
        ),

        # 启动 RViz2 可视化工具
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
