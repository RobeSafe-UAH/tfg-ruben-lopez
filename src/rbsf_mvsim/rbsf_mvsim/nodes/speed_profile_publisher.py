import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray
import yaml

from rbsf_mvsim.algorithms.speed_profiler import SpeedProfiler

class SpeedProfilePublisher(Node):
    """
    ROS 2 Node that calculates and publishes the target speed profile
    for a given global path.
    LEGACY, NO LONGER USED.

    Attributes:
        profiler (SpeedProfiler): The speed profiler algorithm instance.
        subscription (Subscription): Subscriber for the global path.
        speed_pub (Publisher): Publisher for the target speeds.
    """
    def __init__(self):
        """Initializes the SpeedProfilePublisher node."""
        super().__init__('speed_profile_publisher')
        
        # Get parameters from yaml
        with open('/sim_ws/src/rbsf_mvsim/rbsf_mvsim/nodes/params.yaml', 'r') as f:
            params = yaml.safe_load(f)
            
        speed_profile_params = params.get('speed_profiler', {})
        self.profiler = SpeedProfiler(
            v_min=speed_profile_params.get('v_min', 1.5),
            v_max=speed_profile_params.get('v_max', 20.0),
            lat_accel=speed_profile_params.get('lat_accel', 3.0),
            long_accel=speed_profile_params.get('long_accel', 2.0))
        
        self.subscription = self.create_subscription(
            Path, '/global_path', self.path_callback, 10)
            
        self.speed_pub = self.create_publisher(
            Float64MultiArray, '/target_speeds', 10)
            
        self.get_logger().info('Speed Profile Generator started.')

    def path_callback(self, msg):
        """
        Callback executed whenever the path is updated.
        Generates and publishes the corresponding speed profile.

        Args:
            msg (Path): The incoming global path message.
        """
        # Extract points from the path
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        
        # Generate the speed profile
        speeds = self.profiler.generate_profile(points)
        
        # Convert to array and publish
        speed_msg = Float64MultiArray()
        speed_msg.data = speeds
        self.speed_pub.publish(speed_msg)
        self.get_logger().info('Speed profile calculated and published.')
        
def main(args=None):
    """
    Main function to initialize and spin the SpeedProfilePublisher node.
    """
    rclpy.init(args=args)
    node = SpeedProfilePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()