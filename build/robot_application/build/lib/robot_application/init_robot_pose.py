from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy


def main():
    rclpy.init()
    navigator = BasicNavigator(namespace='trans_robot_1')
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()
    rclpy.spin(navigator)
    rclpy.shutdown()

    rclpy.init()
    navigator2 = BasicNavigator(namespace='trans_robot_2')
    initial_pose2 = PoseStamped()
    initial_pose2.header.frame_id = 'map'
    initial_pose2.header.stamp = navigator2.get_clock().now().to_msg()
    initial_pose2.pose.position.x = 1.0
    initial_pose2.pose.position.y = 1.0
    initial_pose2.pose.orientation.w = 1.0
    navigator2.setInitialPose(initial_pose2)
    navigator2.waitUntilNav2Active()
    rclpy.spin(navigator2)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
