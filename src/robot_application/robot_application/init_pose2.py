import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_publisher')

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

    def publish_static_transform(self):
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'map'  # 参考坐标系
        static_transform.child_frame_id = 'robot2/odom'  # 目标坐标系

        # 设置相对位置和方向（根据实际场景调整）
        static_transform.transform.translation.x = 0.0  # 示例：robot2相对于map的偏移
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0
    
        self.static_broadcaster.sendTransform(static_transform)
        self.get_logger().info('Published static transform between map and robot1/odom')


        # static_transform2 = TransformStamped()
        # static_transform2.header.stamp = self.get_clock().now().to_msg()
        # static_transform2.header.frame_id = 'map'  # 参考坐标系
        # static_transform2.child_frame_id = 'robot2/odom'  # 目标坐标系

        # # 设置相对位置和方向（根据实际场景调整）
        # static_transform2.transform.translation.x = 0.0  # 示例：robot2相对于map的偏移
        # static_transform2.transform.translation.y = 0.0
        # static_transform2.transform.translation.z = 0.0
        # static_transform2.transform.rotation.x = 0.0
        # static_transform2.transform.rotation.y = 0.0
        # static_transform2.transform.rotation.z = 0.0
        # static_transform2.transform.rotation.w = 1.0

        # self.static_broadcaster.sendTransform(static_transform2)
        # self.get_logger().info('Published static transform between map and robot2/odom')


def main():
    rclpy.init()
    node = StaticTransformPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()