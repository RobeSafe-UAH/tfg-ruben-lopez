import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import math
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from rbsf_mvsim.algorithms.geometric_path_optimizer import GeometricPathOptimizer
from rbsf_mvsim.algorithms.kinematic_path_optimizer import KinematicPathOptimizer
from rbsf_mvsim.algorithms.none_path_optimizer import NonePathOptimizer
import yaml

class OfflinePlannerNode(Node):
    """
    ROS 2 Node for offline path planning. Subscribes to the global path, optimizes it,
    and publishes the optimized path and the corresponding speed profile.

    Attributes:
        optimizer: Instance of the selected path optimizer.
        subscription: Subscriber to the global path.
        path_publisher_: Publisher for the optimized path.
        speed_publisher_: Publisher for the speed profile.
    """
    def __init__(self):
        """Initializes the OfflinePlannerNode and loads the chosen optimizer."""
        super().__init__('offline_planner_node')
        
        # INITIALIZE ALGORITHM OBJECT
        # Get parameters from yaml
        with open('/sim_ws/src/rbsf_mvsim/rbsf_mvsim/nodes/params.yaml', 'r') as f:
            params = yaml.safe_load(f)
        optimizer_type = params.get('path_optimizer_type', '')
        optimizer_params = params.get(optimizer_type, {})
        optimizer_params.update(params.get('generic', {}))
        
        if optimizer_type == 'geometric_optimizer':
            self.optimizer = GeometricPathOptimizer.from_params(optimizer_params)
        elif optimizer_type == 'kinematic_optimizer':
            self.optimizer = KinematicPathOptimizer.from_params(optimizer_params)
        elif optimizer_type == 'none_optimizer':
            self.optimizer = NonePathOptimizer.from_params(optimizer_params)

        self.subscription = self.create_subscription(
            Path, '/global_path', self.path_callback, 10)
        
        self.path_publisher_ = self.create_publisher(Path, '/optimized_path', 10)
        self.speed_publisher_ = self.create_publisher(Float64MultiArray, '/target_speeds', 10)
        
        self.get_logger().info('Offline planner ready. Waiting for path update...')

    def path_callback(self, msg):
        """
        Callback executed whenever a path update arrives.
        Optimizes the received path and republishes it on /optimized_path.
        Also publishes the speed profile associated with the optimized path.

        Args:
            msg (Path): The incoming global path message.
        """
        path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        optimized_path, speed_profile = self.optimizer.optimize_path(path)
        
        # Build ROS Path message
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, z in optimized_path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            
            # Position
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            
            # Orientation (Quaternion): 
            # Temporarily maintains neutral orientation. To be improved to calculate orientation if needed.
            pose.pose.orientation.w = 1.0 
            path_msg.poses.append(pose)
            
        # Wait a second before publishing to give time to initialize the simulation
        self.get_clock().sleep_for(rclpy.duration.Duration(nanoseconds=1000000000))
        
        # Publish optimized path
        self.path_publisher_.publish(path_msg)
        self.get_logger().info('Optimized path calculated and published to /optimized_path.')
        
        # Publish associated speed profile
        speed_msg = Float64MultiArray()
        speed_msg.data = speed_profile
        self.speed_publisher_.publish(speed_msg)
        self.get_logger().info('Speed profile associated with optimized path published to /target_speeds.')
    

def main(args=None):
    """
    Main function to initialize and run the OfflinePlannerNode.
    """
    rclpy.init(args=args)
    node = OfflinePlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()