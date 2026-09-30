import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import os

class WaypointLogger(Node):
    """
    ROS 2 Node for logging waypoints. Listens to RViz clicked points
    and saves them to a CSV file.

    Attributes:
        subscription (Subscription): Subscriber to the RViz clicked_point topic.
        file_path (str): The file path where waypoints will be saved.
    """
    def __init__(self):
        """Initializes the WaypointLogger node."""
        super().__init__('waypoint_logger_node')
        
        # Subscriber to the publish point tool topic
        self.subscription = self.create_subscription(
            PointStamped,
            '/r1/clicked_point',
            self.listener_callback,
            10)
            
        # File path (in algorithms folder to be used by map_tools)
        self.file_path = '/sim_ws/src/rbsf_mvsim/rbsf_mvsim/algorithms/centerline.csv'

        self.get_logger().info(f"Saving points to: {self.file_path}")
        self.get_logger().info("Use the 'Publish Point' tool in Rviz to save coordinates.")

    def listener_callback(self, msg):
        """
        Callback to process the clicked points and append them to the CSV.

        Args:
            msg (PointStamped): The clicked point message from RViz.
        """
        # Extract coordinates
        x = msg.point.x
        y = msg.point.y
        
        # Save in CSV format: x,y
        with open(self.file_path, 'a') as f:
            f.write(f"{x},{y}\n")
            
        self.get_logger().info(f"Point saved: x={x:.2f}, y={y:.2f}")

def main(args=None):
    """
    Main entry point for the WaypointLogger node.
    """
    rclpy.init(args=args)
    node = WaypointLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()