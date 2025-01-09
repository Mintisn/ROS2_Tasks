import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取功能包share默认路径
    urdf_pkg_path = get_package_share_directory('task2')
    
    # 设置两个机器人模型和world文件路径
    default_model_path_robot1 = urdf_pkg_path + '/urdf/trans_robot_ns1.urdf.xacro'
    default_model_path_robot2 = urdf_pkg_path + '/urdf/trans_robot_ns2.urdf.xacro'
    default_world_path = urdf_pkg_path + '/world/new_world.world'

    # 设置Launch参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='M_robot1', default_value=str(default_model_path_robot1),
        description='加载机器人1的模型路径'
    )
    action_declare_arg_mode_path_robot2 = launch.actions.DeclareLaunchArgument(
        name='M_robot2', default_value=str(default_model_path_robot2),
        description='加载机器人2的模型路径'
    )

    # action_declare_arg_namespace1 = launch.actions.DeclareLaunchArgument(
    #     name='robot_namespace1', default_value="robot1",
    #     description='加载机器人1的命名空间'
    # )
    # action_declare_arg_namespace2 = launch.actions.DeclareLaunchArgument(
    #     name='robot_namespace2', default_value="robot2",
    #     description='加载机器人2的命名空间'
    # )
    
    # 获取xacro文件并生成URDF描述
    robot_description_robot1 = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('M_robot1'),
            #  ' namespace:=', launch.substitutions.LaunchConfiguration('robot_namespace1')  # 传递命名空间
            ]),
        value_type=str
    )
    robot_description_robot2 = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('M_robot2'),
            #   ' namespace:=', launch.substitutions.LaunchConfiguration('robot_namespace2')  # 传递命名空间
            ]),
        value_type=str
    )

    # Robot State Publisher Nodes for both robots
    robot_state_publisher_node_robot1 = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',  # 给机器人1分配命名空间
        parameters=[{'robot_description': robot_description_robot1}]
    )
    robot_state_publisher_node_robot2 = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot2',  # 给机器人2分配命名空间
        parameters=[{'robot_description': robot_description_robot2}]
    )

    # 启动Gazebo，并加载世界文件
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
        launch_arguments=[('world', default_world_path), ('verbose', 'true')]
    )
    robot_name_in_model = "robot1"
    robot_name_in_model2 = "robot2"
    # Spawn Entities for both robots
    spawn_entity_node_robot1 = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot1/robot_description', '-entity', robot_name_in_model,'-x', '0', '-y', '0', '-z', '0']
    )
    spawn_entity_node_robot2 = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot2/robot_description', '-entity', robot_name_in_model2,'-x', '1', '-y', '1', '-z', '0']
    )

    # 控制器加载和激活，机器人1
    load_joint_state_controller_robot1 = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'robot1/fishbot_joint_state_broadcaster'],
        output='screen'
    )
    load_fishbot_diff_drive_controller_robot1 = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'robot1/fishbot_diff_drive_controller'],
        output='screen'
    )

    # 控制器加载和激活，机器人2
    load_joint_state_controller_robot2 = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'robot2/fishbot_joint_state_broadcaster'],
        output='screen'
    )
    load_fishbot_diff_drive_controller_robot2 = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'robot2/fishbot_diff_drive_controller'],
        output='screen'
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path_robot2,
        action_declare_arg_mode_path,

        # action_declare_arg_namespace1,
        # action_declare_arg_namespace2,
        robot_state_publisher_node_robot2,
        robot_state_publisher_node_robot1,

        launch_gazebo,

        spawn_entity_node_robot2,
        spawn_entity_node_robot1,

        # 为机器人2注册事件
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_entity_node_robot2,
                on_exit=[load_joint_state_controller_robot2],
            )
        ),
        # 为机器人2加载diff驱动控制器
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_joint_state_controller_robot2,
                on_exit=[load_fishbot_diff_drive_controller_robot2],
            )
        ),
        # 为机器人1注册事件
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_entity_node_robot1,
                on_exit=[load_joint_state_controller_robot1],
            )
        ),
        # 为机器人1加载diff驱动控制器
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_joint_state_controller_robot1,
                on_exit=[load_fishbot_diff_drive_controller_robot1],
            )
        ),

    ])
