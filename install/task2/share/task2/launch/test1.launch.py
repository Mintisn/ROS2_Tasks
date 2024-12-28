import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_dcp = launch.actions.DeclareLaunchArgument('robot_description', default_value=get_package_share_directory("task2") + '/urdf/test.urdf',
                                                     description='Path to the URDF file')
    rviz_config = launch.actions.DeclareLaunchArgument('rviz_config', default_value=get_package_share_directory("task2") + '/config/config3.rviz',
                                                       description='Path to the RViz config file')
    rviz2_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', launch.substitutions.LaunchConfiguration('rviz_config')],
        parameters=[{'robot_description': launch.substitutions.LaunchConfiguration('robot_description')}],
    )

    return launch.LaunchDescription([
        robot_dcp,
        rviz_config,
        rviz2_node,
    ])
