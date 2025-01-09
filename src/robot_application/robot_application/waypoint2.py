from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init()
    navigator = BasicNavigator(namespace='trans_robot_1')
    navigator2 = BasicNavigator(namespace='trans_robot_2')
    navigator.waitUntilNav2Active()
    navigator2.waitUntilNav2Active()
    # 创建点集1
    goal_poses = []
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 1.0
    goal_pose1.pose.position.y = 0.0
    goal_pose1.pose.orientation.w = 1.0
    goal_poses.append(goal_pose1)
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = -3.0
    goal_pose2.pose.position.y = 1.0
    goal_pose2.pose.orientation.w = 1.0
    goal_poses.append(goal_pose2)
    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -3.0
    goal_pose3.pose.position.y = -1.0
    goal_pose3.pose.orientation.w = 1.0
    goal_poses.append(goal_pose3)
    # 创建点集2
    goal2_poses = []
    goal2_pose1 = PoseStamped()
    goal2_pose1.header.frame_id = 'map'
    goal2_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal2_pose1.pose.position.x = 1.0
    goal2_pose1.pose.position.y = 0.0
    goal2_pose1.pose.orientation.w = 1.0
    goal2_poses.append(goal2_pose1)
    goal2_pose2 = PoseStamped()
    goal2_pose2.header.frame_id = 'map'
    goal2_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal2_pose2.pose.position.x = -3.0
    goal2_pose2.pose.position.y = 1.0
    goal2_pose2.pose.orientation.w = 1.0
    goal2_poses.append(goal2_pose2)
    goal2_pose3 = PoseStamped()
    goal2_pose3.header.frame_id = 'map'
    goal2_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal2_pose3.pose.position.x = -3.0
    goal2_pose3.pose.position.y = -1.0
    goal2_pose3.pose.orientation.w = 1.0
    goal2_poses.append(goal2_pose3)
    # 调用路点导航服务
    while(True):
        navigator.followWaypoints(goal_poses)
        navigator2.followWaypoints(goal_poses)
        # 判断结束及获取反馈
        while not navigator.isTaskComplete() or not navigator2.isTaskComplete():
            feedback = navigator.getFeedback()
            navigator.get_logger().info(
                f'当前目标编号：{feedback.current_waypoint}')
            feedback2 = navigator2.getFeedback()
            navigator2.get_logger().info(
                f'当前目标编号：{feedback2.current_waypoint}')
        # 最终结果判断 没有第二个小车
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info('导航结果：成功')
        elif result == TaskResult.CANCELED:
            navigator.get_logger().warn('导航结果：被取消')
        elif result == TaskResult.FAILED:
            navigator.get_logger().error('导航结果：失败')
        else:
            navigator.get_logger().error('导航结果：返回状态无效')

if __name__ == '__main__':
    main()
