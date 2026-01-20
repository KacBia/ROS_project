import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Twist


class ClickControl(Node):
    def __init__(self):
        super().__init__('click_control_node')

        self.sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.click_callback,
            10
        )

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.get_logger().info('ClickControl node started. Click in RViz.')

    def click_callback(self, msg: PointStamped):
        twist = Twist()

        if msg.point.y > 0.0:
            twist.linear.x = 0.2
            self.get_logger().info('Clicked ABOVE center → FORWARD')
        else:
            twist.linear.x = -0.2
            self.get_logger().info('Clicked BELOW center → BACKWARD')

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ClickControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
