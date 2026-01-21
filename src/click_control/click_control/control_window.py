import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import numpy as np

SPEED = 0.5

class ControlWindow(Node):
    """
    Simple GUI control (like in the PDF screenshot):
    - Black window with a center line
    - Click ABOVE center => forward
    - Click BELOW center => backward
    - Right-click => STOP
    - ESC / q => exit
    Publishes Twist on /cmd_vel.
    """
    def __init__(self):
        super().__init__('control_window_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.w = 640
        self.h = 480
        self.win = "control_window"
        self.last_cmd = "STOP"

        cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.win, self.w, self.h)
        cv2.setMouseCallback(self.win, self.on_mouse)

        # publish STOP periodically if no clicks (optional safety)
        self.timer = self.create_timer(0.2, self.keep_alive_stop)

        self.get_logger().info("Control window started. Left-click above/below center. Right-click STOP. ESC/q exit.")

    def publish_twist(self, lin_x: float):
        msg = Twist()
        msg.linear.x = float(lin_x)
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def keep_alive_stop(self):
        if self.last_cmd == "STOP":
            self.publish_twist(0.0)

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if y < self.h // 2:
                self.last_cmd = "FORWARD"
                self.publish_twist(SPEED)
                self.get_logger().info("GUI click ABOVE center -> FORWARD")
            else:
                self.last_cmd = "BACKWARD"
                self.publish_twist(-SPEED)
                self.get_logger().info("GUI click BELOW center -> BACKWARD")

        elif event == cv2.EVENT_RBUTTONDOWN:
            self.last_cmd = "STOP"
            self.publish_twist(0.0)
            self.get_logger().info("GUI right-click -> STOP")

    def draw(self):
        img = np.zeros((self.h, self.w, 3), dtype=np.uint8)

        cv2.line(img, (0, self.h // 2), (self.w, self.h // 2), (0, 255, 0), 2)

        cv2.putText(img, f"{self.last_cmd}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)

        cv2.putText(img, "Left-click: FWD/BWD   Right-click: STOP", (20, self.h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

        cv2.imshow(self.win, img)

def main(args=None):
    rclpy.init(args=args)
    node = ControlWindow()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.draw()
            k = cv2.waitKey(1) & 0xFF
            if k in [27, ord('q')]:
                break
    finally:
        node.publish_twist(0.0)
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
