import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64MultiArray
import math

from rbsf_mvsim.algorithms.metrics import Metrics

class MetricsEvaluator(Node):
    """
    ROS 2 Node that evaluates and logs vehicle performance metrics such as lap time,
    cross-track error (CTE), and speed error.

    Attributes:
        metrics (Metrics): Instance of the Metrics algorithm to process logic.
        path_points (list): Current path points.
        target_speeds (list): Target speed profile.
        last_x (float): Previous X position.
        last_y (float): Previous Y position.
        last_odom_time (float): Timestamp of the last odometry message.
    """
    def __init__(self):
        """Initializes the MetricsEvaluator node and subscriptions."""
        super().__init__('metrics_evaluator_node')
        
        self.metrics = Metrics()
        
        self.path_points = []
        self.target_speeds = []
        
        # State variables
        self.last_x = None
        self.last_y = None
        self.last_odom_time = None
        
        self.create_subscription(Path, '/optimized_path', self.path_cb, 10)
        self.create_subscription(Float64MultiArray, '/target_speeds', self.speed_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        self.timer = self.create_timer(1.0, self.print_stats)
        self.get_logger().info("Metrics node started (Original timestamp calculation).")

    def path_cb(self, msg):
        """
        Callback for path updates.

        Args:
            msg (Path): The incoming path message.
        """
        self.path_points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def speed_cb(self, msg):
        """
        Callback for speed profile updates.

        Args:
            msg (Float64MultiArray): The incoming speed profile message.
        """
        self.target_speeds = msg.data

    def odom_cb(self, msg):
        """
        Callback for odometry updates. Calculates current speed and updates metrics.

        Args:
            msg (Odometry): The incoming odometry message.
        """
        if not self.path_points: return
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # CRITICAL: Extract the exact timestamp generated in the simulator.
        # It is converted to floating seconds for direct calculation.
        current_time_sec = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)

        v = 0.0
        if self.last_x is not None and self.last_y is not None and self.last_odom_time is not None:
            dx = x - self.last_x
            dy = y - self.last_y
            dt = current_time_sec - self.last_odom_time
            
            # Prevent division by zero in case of duplicated messages
            if dt > 0.0:
                v = math.sqrt(dx**2 + dy**2) / dt
        
        # Update states
        self.last_x = x
        self.last_y = y
        self.last_odom_time = current_time_sec

        # Find closest point
        dists = [math.hypot(px-x, py-y) for px, py in self.path_points]
        current_idx = dists.index(min(dists))
        
        # Execute metrics logic
        result = self.metrics.update(x, y, v, self.path_points, self.target_speeds, current_idx)

        if result and result['lap_completed']:
            self.get_logger().info(f"LAP {self.metrics.lap_count} COMPLETED | Time: {result['lap_time']:.3f}s")
            if result['lap_time'] >= 499.0:
                # Destroy node to avoid generating more metrics from an invalid lap
                self.get_logger().warn("Metrics node terminated due to an invalid lap.")
                rclpy.shutdown()

    def print_stats(self):
        """Prints current summary statistics of the lap."""
        stats = self.metrics.get_summary()
        if stats:
            self.get_logger().info(f"Stats -> Avg CTE: {stats['avg_cte']:.3f}m | Avg Speed Error: {stats['avg_speed_error']:.2f} m/s")
            
            # Write current status for the tuner to use in case of timeout
            status = {
                "progress": self.metrics.get_progress(),
                "avg_cte": stats.get('avg_cte', 0.0),
                "avg_speed_error": stats.get('avg_speed_error', 0.0)
            }
            try:
                import json
                with open('/sim_ws/src/rbsf_mvsim/rbsf_mvsim/metrics/current_status.json', 'w') as f:
                    json.dump(status, f)
            except Exception:
                pass

def main(args=None):
    """
    Main entry point for MetricsEvaluator node.
    """
    rclpy.init(args=args)
    node = MetricsEvaluator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()