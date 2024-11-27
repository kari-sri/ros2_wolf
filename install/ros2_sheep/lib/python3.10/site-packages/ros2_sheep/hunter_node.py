import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
import math


class HunterNode(Node):
    def __init__(self):
        super().__init__('hunter_node')
        self.publisher = self.create_publisher(Twist, '/hunter/cmd_vel', 10)
        self.create_subscription(Pose, '/hunter/pose', self.update_pose, 10)
        self.create_subscription(Pose, '/wolf/pose', self.update_wolf_pose, 10)
        self.pose = None
        self.wolf_pose = None
        self.hunter_catch_count = 0
        self.timer = self.create_timer(0.2, self.move)
        self.set_pen_color(0, 0, 255)  # Blue for hunter

    def update_pose(self, msg):
        self.pose = msg

    def update_wolf_pose(self, msg):
        self.wolf_pose = msg

    def set_pen_color(self, r, g, b):
        client = self.create_client(SetPen, '/hunter/set_pen')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /hunter/set_pen service...')
        request = SetPen.Request()
        request.r, request.g, request.b = r, g, b
        request.width = 3
        request.off = 0
        client.call_async(request)

    def move(self):
        if not self.pose or not self.wolf_pose:
            return

        msg = Twist()
        distance_to_wolf = math.sqrt((self.pose.x - self.wolf_pose.x)**2 + (self.pose.y - self.wolf_pose.y)**2)

        if distance_to_wolf > 0.5:
            # Chase wolf
            angle_to_wolf = math.atan2(self.wolf_pose.y - self.pose.y, self.wolf_pose.x - self.pose.x)
            msg.linear.x = 0.5
            msg.angular.z = angle_to_wolf - self.pose.theta
        else:
            self.hunter_catch_count += 1
            self.get_logger().info(f'Hunter caught wolf! Count: {self.hunter_catch_count}')

        # Boundary avoidance
        if self.pose.x <= 1.0 or self.pose.x >= 10.0 or self.pose.y <= 1.0 or self.pose.y >= 10.0:
            msg.angular.z = 1.0
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HunterNode()
    rclpy.spin(node)
    rclpy.shutdown()
