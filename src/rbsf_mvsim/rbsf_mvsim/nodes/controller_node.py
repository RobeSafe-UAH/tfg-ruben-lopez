import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import math
from std_msgs.msg import Float64MultiArray
from rbsf_mvsim.algorithms.pure_pursuit import PurePursuit
from rbsf_mvsim.algorithms.map_controller import MAPController
from rbsf_mvsim.algorithms.mpc_controller import MPCController
from rbsf_mvsim.algorithms.pid_controller import PIDController
from geometry_msgs.msg import PoseStamped
import yaml

class ControllerNode(Node):
    """
    ROS 2 Node that calculates and publishes drive commands based on a selected 
    control algorithm. Subscribes to odometry, optimized path, and target speeds.
    
    Attributes:
        controller: The instantiated control algorithm.
        current_path (list): The currently active path as a list of poses.
        current_speed_profile (list): The current speed profile for the path.
        car_x (float): The current x position of the car.
        car_y (float): The current y position of the car.
        prev_x (float): The previous x position of the car.
        prev_y (float): The previous y position of the car.
        car_yaw (float): The current heading (yaw) of the car.
        car_speed (float): The current calculated speed of the car.
        path_received (bool): Flag indicating if a path has been received.
    """
    def __init__(self):
        """Initializes the ControllerNode and its components."""
        super().__init__('controller_node')
        
        # INITIALIZE ALGORITHM OBJECT
        # Get parameters from yaml
        with open('/sim_ws/src/rbsf_mvsim/rbsf_mvsim/nodes/params.yaml', 'r') as f:
            params = yaml.safe_load(f)
        controller_type = params.get('controller_type', '')
        controller_params = params.get(controller_type, {})
        controller_params.update(params.get('generic', {}))
        if controller_type == 'pid_controller':
            self.controller = PIDController.from_params(controller_params)
        elif controller_type == 'pure_pursuit':
            self.controller = PurePursuit.from_params(controller_params)
        elif controller_type == 'map_controller':
            self.controller = MAPController.from_params(controller_params)
        elif controller_type == 'mpc_controller':
            self.controller = MPCController.from_params(controller_params)

        # STATE VARIABLES
        self.current_path = [] # Here we store the path line
        self.current_speed_profile = [] # Here we store the speed profile
        self.car_x = 0.0
        self.car_y = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.car_yaw = 0.0
        self.car_speed = 0.0
        self.path_received = False
        
        # SUBSCRIBERS AND PUBLISHERS
        
        # Path
        self.subscription_path = self.create_subscription(Path, '/optimized_path', self.path_callback, 10)
            
        # Odometry
        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Drive
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Speed profile
        self.speed_subscription = self.create_subscription(Float64MultiArray, '/target_speeds', self.speed_callback, 10)
        
        # START MAIN LOOP

        # Execute every 0.1 seconds (10 Hz)
        self.loop_time = 0.1
        self.timer = self.create_timer(self.loop_time, self.control_loop)
        
        self.get_logger().info('Controller ready. Waiting for odometry and path...')

    def path_callback(self, msg):
        """
        Callback executed whenever a path update arrives.
        Updates the path data in the local variable.

        Args:
            msg (Path): The incoming path message.
        """
        self.current_path = msg.poses
        self.path_received = True

    def odom_callback(self, msg):
        """
        Callback executed whenever an odometry update arrives.
        Extracts the car's position and orientation.
        Speed is calculated only during the control algorithm execution to avoid unnecessary calculations.
        Converts the data and updates it in local variables.

        Args:
            msg (Odometry): The incoming odometry message.
        """
        # Save car position directly
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y

        # Get rotation data in quaternions
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Convert to Euler and save it
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.car_yaw = math.atan2(siny_cosp, cosy_cosp)

    def speed_callback(self, msg):
        """
        Callback executed whenever a speed profile update arrives.
        Updates the local variable for the speed profile.

        Args:
            msg (Float64MultiArray): The incoming speed profile message.
        """
        self.current_speed_profile = msg.data

    def control_loop(self):
        """
        Calls the control algorithm and sends movement commands on each execution.
        """
        # Only operate if data has been received
        if not self.path_received:
            return
        
        # Start by calculating speed using previous position, current position, and elapsed time
        dx = self.car_x - self.prev_x
        dy = self.car_y - self.prev_y
        dist = math.sqrt(dx**2 + dy**2)
        self.car_speed = dist / self.loop_time if self.loop_time > 0 else 0.0

        # CONTROL ALGORITHM CALL
        steering_angle, target_speed = self.controller.calculate_control(
            self.car_x, 
            self.car_y, 
            self.car_yaw, 
            self.car_speed,
            self.current_path,
            self.current_speed_profile
        )

        speed = target_speed # Speed control algorithm removed, now handled by offline planning
        
        # PUBLISH
        self.send_drive_command(speed, steering_angle)

        # Save current position as previous for the next speed calculation
        self.prev_x = self.car_x
        self.prev_y = self.car_y


    def send_drive_command(self, speed, steering_angle):
        """
        Publishes the drive command.

        Args:
            speed (float): The desired speed.
            steering_angle (float): The desired steering angle.
        """
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)

def main(args=None):
    """
    Main function to initialize and spin the ControllerNode.
    """
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()