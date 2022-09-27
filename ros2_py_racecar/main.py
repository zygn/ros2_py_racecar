import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import *

# Import your planner
from .planner.wall_follow import WallFollow


class Racecar(Node):

    def __init__(self):
        super().__init__('racecar')

        qos_profile = QoSProfile(depth=10)

        self.pub = self.create_publisher(AckermannDriveStamped, 'drive', qos_profile)
        self.sub = {
            "laser": self.create_subscription(
                LaserScan,
                "scan",
                self.laser_callback,
                qos_profile
            ),
            "odom": self.create_subscription(
                Odometry,
                "ego_racecar/odom",
                self.odom_callback,
                qos_profile
            )
        }

        self.Hz = 100
        self.timer = self.create_timer((1/self.Hz), self.publish_callback)

        self.racecar = {
            'scan': LaserScan(),
            'odom': Odometry()
        }

        # planner declaration
        self.planner = WallFollow()

        self.get_logger().info("")


    def publish_callback(self):
        msg = AckermannDriveStamped()

        if hasattr(self.planner, 'drive'):
            speed, steer = self.planner.drive(self.racecar['scan'], self.racecar['odom'])
        elif hasattr(self.planner, 'plan'):
            speed, steer = self.planner.plan(self.racecar, self.racecar['odom'])
        else:
            speed, steer = 0.0, 0.0

        msg.drive.speed = speed
        msg.drive.steering_angle = steer

        self.pub.publish(msg)

    def laser_callback(self, msg: LaserScan):
        # self.get_logger().info("scan callback msgs")
        self.racecar['scan'] = msg

    def odom_callback(self, msg: Odometry):
        # self.get_logger().info("odom callback msgs")
        self.racecar['odom'] = msg


def main(args=None):
    rclpy.init(args=args)

    racecar = Racecar()

    rclpy.spin(racecar)

    racecar.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
