import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取路径
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    task_dir = get_package_share_directory('task2')
    
    # 配置文件路径
    robot1_params_path = os.path.join(task_dir, 'config', 'nav2_params.yaml')
    robot2_params_path = os.path.join(task_dir, 'config', 'nav2_params.yaml')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 修改地图配置
    map_yaml_path = LaunchConfiguration(
        'map',
        default=os.path.join(task_dir, 'maps', 'room1.yaml')
    )
    
    # 为Robot1启动导航堆栈
    nav_robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
        launch_arguments={
            'namespace': 'robot1',
            'use_namespace': 'True',
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': robot1_params_path
        }.items()
    )
    
    # # Robot1的rviz2实例
    # rviz_robot1 = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     namespace='robot1',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_dir],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     remappings=[('/tf', 'tf'),
    #                ('/tf_static', 'tf_static')],
    #     output='screen'
    # )
    
    # 为Robot2启动导航堆栈
    nav_robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
        launch_arguments={
            'namespace': 'robot2',
            'use_namespace': 'True',
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': robot2_params_path
        }.items()
    )
    
    # # Robot2的rviz2实例
    # rviz_robot2 = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     namespace='robot2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_dir],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     remappings=[('/tf', 'tf'),
    #                ('/tf_static', 'tf_static')],
    #     output='screen'
    # )

    # 声明地图参数
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(task_dir, 'maps', 'room1.yaml'),
        description='Full path to map yaml file to load'
    )

    # 添加地图服务器节点
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(task_dir, 'maps', 'room1.yaml')},
                   {'use_sim_time': use_sim_time}]
    )

    # 添加生命周期管理节点
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                   {'autostart': True},
                   {'node_names': ['map_server']}]
    )

    return LaunchDescription([
        declare_map_cmd,  # 添加地图参数声明
        map_server,           # 添加地图服务器
        lifecycle_manager,    # 添加生命周期管理器
        nav_robot1,
        nav_robot2,
        # 启动 RViz2 可视化工具
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # namespace='robot1',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
    ])
