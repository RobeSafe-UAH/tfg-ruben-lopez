import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from tf2_ros import Buffer, TransformListener

from .libraries.LShapeDetector import LShapeDetector
from .libraries.converts_library import yaw2quaternion, quaternion2yaw

class DetectorNode(Node):
    """
        
    """
    def __init__(self):
        super().__init__('visualization_detector_node')
        
        # DECLARACIÓN DE ATRIBUTOS -----
        # Parámetro del LiDAR
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('max_range', 30.0)
        # Parámetros del DBSCAN
        self.declare_parameter('eps', 0.4)
        self.declare_parameter('min_samples', 10)
        # CONOCIMIENTO EXPERTO PARA 1:10 ---- 
        # (Coche escala 1:10 - 206mm de altura)
        self.declare_parameter('car_height', 0.206)
        # (Coche escala 1:10 - 535mm de longitud)
        self.declare_parameter('max_obs_size', 0.550) 
        self.declare_parameter('laser_topic',"/r1/laser1")

        # ASIGNACIÓN DE ATRIBUTOS -----
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.eps = self.get_parameter('eps').value
        self.min_samples = self.get_parameter('min_samples').value
        self.car_height = self.get_parameter('car_height').value
        self.max_obs_size = self.get_parameter('max_obs_size').value
        self.laser_topic = self.get_parameter('laser_topic').value

        # Declaración del buffer de transformadas
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # Instancia del Detector LShape
        self.detector = LShapeDetector(
            eps=self.eps,
            min_samples=self.min_samples,
            max_obs_size=self.max_obs_size,
            min_range=self.min_range
        )

        # Se suscribe a los datos brutos del LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan, 
            self.laser_topic, 
            self.scan_callback, 
            15
        )

        self.raw_pub = self.create_publisher(
            MarkerArray,
            '/perception/raw_obstacles',
            15
        )

        self.get_logger().info("Nodo detector_node iniciado. Esperando datos...")

    def publish_markers(self, raw_detections, header):
        
        out_msg = MarkerArray()

        marker_to_clean = Marker()
        marker_to_clean.header = header
        marker_to_clean.action = Marker.DELETEALL
        out_msg.markers.append(marker_to_clean)
        
        for det in raw_detections:
            
            marker = Marker()
            marker.header = header
            marker.ns = "raw_obstacles"
            marker.id = int(det['label'])

            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Guardamos los datos matemáticos en la pose y la escala
            marker.pose.position.x = float(det['center_x'])
            marker.pose.position.y = float(det['center_y'])
            marker.pose.position.z = 0.0

            # Calculate the yaw
            q = yaw2quaternion(det['theta'])
            marker.pose.orientation.x = float(q[0])
            marker.pose.orientation.y = float(q[1])
            marker.pose.orientation.z = float(q[2])
            marker.pose.orientation.w = float(q[3])
            
            marker.scale.x = float(det['size'])
            marker.scale.y = float(det['size'])
            marker.scale.z = float(self.car_height)

            # Damos un color básico al obstáculo crudo
            marker.color.a = 0.3
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            out_msg.markers.append(marker)
            
        self.raw_pub.publish(out_msg)


    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        mask = (ranges > self.min_range) & (ranges < self.max_range) & (~np.isinf(ranges)) & (~np.isnan(ranges))
        r = ranges[mask]
        theta = angles[mask]
        
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        points = np.column_stack((x, y))

        raw_detections = self.detector.detect(points)

        self.publish_markers(raw_detections, msg.header)
        
def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Deteniendo el nodo detector_node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()