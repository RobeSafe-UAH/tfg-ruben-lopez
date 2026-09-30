#!/usr/bin/env python3

"""Node that translates AckermannDriveStamped messages to Twist messages."""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped


class AckermannToTwistBridge(Node):
    """
    Bridge node to convert AckermannDriveStamped messages to Twist.
    
    Attributes:
        wheelbase (float): Distance between the front and rear axles of the vehicle.
        subscription (Subscription): Subscriber for the AckermannDriveStamped messages.
        publisher (Publisher): Publisher for the Twist messages.
    """

    def __init__(self):
        """Initializes the AckermannToTwistBridge node."""
        super().__init__('ackermann_to_twist_bridge')

        # Parameter: Vehicle wheelbase distance.
        # Default value: 0.33m (approx. for a 1/10 scale car)
        self.declare_parameter('wheelbase', 0.33)
        self.wheelbase = self.get_parameter('wheelbase').value

        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.drive_callback,
            10
        )
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.get_logger().info(f"Node started: Translating /drive -> /cmd_vel (L={self.wheelbase}m)")

    def drive_callback(self, msg):
        """
        Callback for processing incoming AckermannDriveStamped messages.

        Args:
            msg (AckermannDriveStamped): The incoming Ackermann drive message.
        """
        twist_msg = Twist()
        
        v = msg.drive.speed
        delta = msg.drive.steering_angle

        twist_msg.linear.x = v
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0

        if self.wheelbase > 0.0:
            twist_msg.angular.z = (v / self.wheelbase) * math.tan(delta)
        else:
            twist_msg.angular.z = delta 

        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0

        self.publisher.publish(twist_msg)


def main(args=None):
    """
    Main function to initialize and run the AckermannToTwistBridge node.

    Args:
        args (list, optional): Command-line arguments passed to rclpy.init.
    """
    rclpy.init(args=args)
    node = AckermannToTwistBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()