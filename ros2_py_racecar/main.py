import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# Import your planner and linking to `Planner`
from .planner.example_planner import WallFollowPlanner as Planner
# from .planner.example_planner import FgPlanner as Planner

class Racecar(Node):

    def __init__(self):
        super().__init__('racecar')

        qos_profile = QoSProfile(depth=10)

        drive_topic = self.declare_parameter('drive_topic', '/drive').value
        laser_topic = self.declare_parameter('laser_topic', '/scan').value
        odom_topic = self.declare_parameter('odom_topic', '/odom').value
        hz = self.declare_parameter('hz', 100).value
        wheelbase = self.declare_parameter('wheelbase', 0.3302).value

        assert isinstance(drive_topic, str), "drive_topic parameter must be a str"
        assert isinstance(laser_topic, str), "laser_topic parameter must be a str"
        assert isinstance(odom_topic, str), "odom_topic parameter must be a str"
        assert isinstance(hz, int), "hz parameter must be a int"
        assert isinstance(wheelbase, float), "wheelbase parameter must be a float"

        self.pub = self.create_publisher(AckermannDriveStamped, drive_topic, qos_profile)
        self.sub = {
            'laser': self.create_subscription(LaserScan, laser_topic, self.laser_callback, qos_profile=qos_profile_sensor_data),
            'odom': self.create_subscription(Odometry, odom_topic, self.odom_callback, qos_profile=qos_profile_sensor_data)
        }

        self.wheelbase = wheelbase
        self.Hz = hz
        self.timer = self.create_timer((1 / self.Hz), self.publish_callback)

        self.scan_data = {
            'ranges': [],
        }
        self.odom_data = {
            'pose_x': 0.0,
            'pose_y': 0.0,
            'pose_theta': 0.0,
            'linear_vels_x': 0.0,
            'linear_vels_y': 0.0,
            'ang_vels_z': 0.0
        }

        # planner declaration
        self.planner = Planner()

    def publish_callback(self, _finally=None):
        """
        Publish to /drive topic.

        :return: None
        """
        msg = AckermannDriveStamped()

        if not self.scan_data['ranges']:
            self.get_logger().warn("LaserScan data is empty.")
            return

        if hasattr(self.planner, 'plan'):
            speed, steer = self.planner.plan(self.scan_data, self.odom_data)
        elif hasattr(self.planner, 'driving'):
            speed, steer = self.planner.driving(self.scan_data, self.odom_data)
        else:
            self.get_logger().error("Planner doesn't have `plan` or `driving` function.")
            speed, steer = 0.0, 0.0

        if _finally:
            speed, steer = 0.0, 0.0

        msg.drive.speed = speed
        msg.drive.steering_angle = steer
        self.get_logger().info(f"speed: {speed}, steer: {steer}")

        self.pub.publish(msg)

    def laser_callback(self, msg: LaserScan):
        """
        Update self LiDAR data.

        :param msg: sensor_msgs.msg/LaserScan
        :return: class variable update
        """
        self.scan_data['ranges'] = msg.ranges

    def odom_callback(self, msg: Odometry):
        """
        Update self Odomerty data.

        :param msg: nav_msgs.msg/Odometry
        :return: class variable update
        """
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)

        self.odom_data = {
            'pose_x': msg.pose.pose.position.x,
            'pose_y': msg.pose.pose.position.y,
            'pose_theta': np.arctan2(siny_cosp, cosy_cosp),
            'linear_vels_x': msg.twist.twist.linear.x,
            'linear_vels_y': msg.twist.twist.linear.y,
            'ang_vels_z': msg.twist.twist.angular.z
        }


def main(args=None):
    rclpy.init(args=args)
    racecar = Racecar()
    try:
        rclpy.spin(racecar)
    except KeyboardInterrupt:
        racecar.get_logger().info("Keyboard Interrupt")
        racecar.publish_callback(_finally=True)
    finally:
        racecar.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
