import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

from rbsf_mvsim.algorithms.map_tools import MapTools

class PathPublisher(Node):
    """
    ROS 2 Node that publishes the global reference path and broadcasts 
    the TF transform for RViz visualization.

    Attributes:
        publisher_ (Publisher): Publisher for the global path.
        timer (Timer): Timer triggering the path publication.
        map_tools (MapTools): Tools for retrieving the centerline.
        tf_broadcaster (TransformBroadcaster): Broadcaster for the base link transform.
        odom_sub (Subscription): Subscriber for ground truth odometry.
    """
    def __init__(self):
        """Initializes the PathPublisher node."""
        super().__init__('path_publisher_node')
        self.publisher_ = self.create_publisher(Path, '/global_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)
        self.map_tools = MapTools()
        
        # Broadcaster to unify RViz with the controller coordinates
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/base_pose_ground_truth',
            self.gt_callback,
            10)

    def gt_callback(self, msg):
        """
        Injects the real vehicle position into the RViz visual tree.

        Args:
            msg (Odometry): The incoming odometry ground truth message.
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

    def publish_path(self):
        """
        Retrieves the centerline from MapTools and publishes it as a Path.
        """
        raw_points = self.map_tools.get_manual_centerline()
        
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in raw_points:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0 
            
            path_msg.poses.append(pose)
            
        self.publisher_.publish(path_msg)

def main(args=None):
    """
    Main function to execute the PathPublisher node.
    """
    rclpy.init(args=args)
    node = PathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()