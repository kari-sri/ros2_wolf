import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
import random


class SheepNode(Node):
    def __init__(self):
        super().__init__('sheep_node')
        self.publisher = self.create_publisher(Twist, '/sheep/cmd_vel', 10)
        self.create_subscription(Pose, '/sheep/pose', self.update_pose, 10)
        self.pose = None
        self.timer = self.create_timer(0.2, self.move_randomly)
        self.set_pen_color(255, 0, 0)  # Red for sheep

    def update_pose(self, msg):
        self.pose = msg

    def set_pen_color(self, r, g, b):
        client = self.create_client(SetPen, '/sheep/set_pen')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /sheep/set_pen service...')
        request = SetPen.Request()
        request.r, request.g, request.b = r, g, b
        request.width = 3
        request.off = 0
        client.call_async(request)

    def move_randomly(self):
        if not self.pose:
            return
        msg = Twist()
        # Boundary avoidance
        if self.pose.x <= 1.0 or self.pose.x >= 10.0 or self.pose.y <= 1.0 or self.pose.y >= 10.0:
            msg.angular.z = random.uniform(-2.0, 2.0)  # Turn when near walls
        else:
            # Random movement
            msg.linear.x = random.uniform(0.1, 0.2)
            msg.angular.z = random.uniform(-1.0, 1.0)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SheepNode()
    rclpy.spin(node)
    rclpy.shutdown()
