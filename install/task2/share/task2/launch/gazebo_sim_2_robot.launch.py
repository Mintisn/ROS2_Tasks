import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command


def generate_launch_description():
    # 获取功能包路径
    package_name = 'task2'  # 替换为你的功能包名称
    urdf_pkg_path = get_package_share_directory(package_name)
    
    # 路径配置
    robot_name_1 = "trans_robot_1"
    robot_name_2 = "trans_robot_2"
    default_model_path = os.path.join(urdf_pkg_path, 'urdf/trans_robot.urdf.xacro')
    default_model2_path = os.path.join(urdf_pkg_path, 'urdf/trans_robot2.urdf.xacro')
    default_world_path = os.path.join(urdf_pkg_path, 'world/new_world.world')

    # 声明参数
    declare_model_path = DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='加载的模型路径')

    declare_model2_path = DeclareLaunchArgument(
        name='model2', default_value=str(default_model2_path),
        description='加载的模型路径')

    # URDF 参数
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str)

    robot_description2 = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model2')]),
        value_type=str)

    # 启动 Gazebo 并加载 world 文件
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments=[('world', default_world_path), ('verbose', 'true')]
    )

    # 第一个机器人
    robot_state_publisher_node_1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        # namespace='robot1'
    )

    spawn_entity_node_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot1/robot_description',
            '-entity', robot_name_1,
            '-x', '0', '-y', '0', '-z', '0'
        ],
        # namespace='robot1'
    )

    load_joint_state_controller_1 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'robot1_joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller_1 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'robot1_diff_drive_controller'],
        output='screen'
    )

    # 第二个机器人
    robot_state_publisher_node_2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description2}],
        # namespace='robot2'
    )

    spawn_entity_node_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot2/robot_description',
            '-entity', robot_name_2,
            '-x', '1', '-y', '1', '-z', '0'
        ],
        # namespace='robot2'
    )

    load_joint_state_controller_2 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'robot2_joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller_2 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'robot2_diff_drive_controller'],
        output='screen'
    )

    # # 通信桥节点
    # communication_bridge_node = Node(
    #     package='your_package_name',  # 替换为包含 communication_bridge 的包名
    #     executable='communication_bridge',
    #     name='communication_bridge',
    #     output='screen'
    # )

    # 返回 Launch Description
    return LaunchDescription([
        declare_model_path,
        declare_model2_path,
        launch_gazebo,
        robot_state_publisher_node_1,
        robot_state_publisher_node_2,
        spawn_entity_node_1,
        spawn_entity_node_2,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_node_1,
                on_exit=[load_joint_state_controller_1]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller_1,
                on_exit=[load_diff_drive_controller_1]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_node_2,
                on_exit=[load_joint_state_controller_2]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller_2,
                on_exit=[load_diff_drive_controller_2]
            )
        ),
        # communication_bridge_node
    ])
