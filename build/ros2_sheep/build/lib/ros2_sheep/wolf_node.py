import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
import math


class WolfNode(Node):
    def __init__(self):
        super().__init__('wolf_node')
        self.publisher = self.create_publisher(Twist, '/wolf/cmd_vel', 10)
        self.create_subscription(Pose, '/wolf/pose', self.update_pose, 10)
        self.create_subscription(Pose, '/sheep/pose', self.update_sheep_pose, 10)
        self.create_subscription(Pose, '/hunter/pose', self.update_hunter_pose, 10)
        self.pose = None
        self.sheep_pose = None
        self.hunter_pose = None
        self.timer = self.create_timer(0.2, self.move)
        self.wolf_reach_count = 0
        self.set_pen_color(0, 255, 0)  # Green for wolf

    def update_pose(self, msg):
        self.pose = msg

    def update_sheep_pose(self, msg):
        self.sheep_pose = msg

    def update_hunter_pose(self, msg):
        self.hunter_pose = msg

    def set_pen_color(self, r, g, b):
        client = self.create_client(SetPen, '/wolf/set_pen')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /wolf/set_pen service...')
        request = SetPen.Request()
        request.r, request.g, request.b = r, g, b
        request.width = 3
        request.off = 0
        client.call_async(request)

    def move(self):
        if not self.pose or not self.sheep_pose or not self.hunter_pose:
            return

        msg = Twist()
        distance_to_sheep = math.sqrt((self.pose.x - self.sheep_pose.x)**2 + (self.pose.y - self.sheep_pose.y)**2)
        distance_to_hunter = math.sqrt((self.pose.x - self.hunter_pose.x)**2 + (self.pose.y - self.hunter_pose.y)**2)

        if distance_to_hunter < 2.0:
            # Avoid hunter
            angle_to_hunter = math.atan2(self.hunter_pose.y - self.pose.y, self.hunter_pose.x - self.pose.x)
            msg.linear.x = -0.4  # Run away
            msg.angular.z = angle_to_hunter - self.pose.theta
        elif distance_to_sheep > 0.5:
            # Chase sheep
            angle_to_sheep = math.atan2(self.sheep_pose.y - self.pose.y, self.sheep_pose.x - self.pose.x)
            msg.linear.x = 0.4
            msg.angular.z = angle_to_sheep - self.pose.theta
        else:
            self.wolf_reach_count += 1
            self.get_logger().info(f'Wolf reached sheep! Count: {self.wolf_reach_count}')

        # Boundary avoidance
        if self.pose.x <= 1.0 or self.pose.x >= 10.0 or self.pose.y <= 1.0 or self.pose.y >= 10.0:
            msg.angular.z = 1.0
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WolfNode()
    rclpy.spin(node)
    rclpy.shutdown()
