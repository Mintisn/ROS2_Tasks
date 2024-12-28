import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
# https://www.bilibili.com/video/BV1ho2dYNE6T?spm_id_from=333.788.videopod.sections&vd_source=6aef8a8b18d581883e4331f31b8d68f0
# https://www.bilibili.com/video/BV1k8mNYyEjW?spm_id_from=333.788.videopod.sections&vd_source=6aef8a8b18d581883e4331f31b8d68f0

def generate_launch_description():
    # 获取功能包share默认路径
    robot_name_in_model = "trans_robot"
    urdf_pkg_path = get_package_share_directory('task2')
    default_model_path = urdf_pkg_path + '/urdf/trans_robot.urdf.xacro'  #xacro-->urdf-->sdf
    default_world_path = urdf_pkg_path + '/world/custom_world.world'
    # 为 Launch 声明参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='加载的模型路径')
    # 获取文件内容生成urdf
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)
  	
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 启动gazebo的同时加载world模型
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
      	# 传递参数
        launch_arguments=[('world', default_world_path),('verbose','true')]
    )

    # 请求 Gazebo 加载机器人,把robot_state_publisher_node 发布的内容通过话题的方式接受 
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model, ])
    
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node,
    ])
