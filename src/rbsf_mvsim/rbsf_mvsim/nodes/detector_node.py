import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from norfair import Detection, Tracker

from .libraries.LShapeDetector import LShapeDetector
from .libraries.converts_library import yaw2quaternion

class DetectorNode(Node):
    """
        
    """
    def __init__(self):
        super().__init__('perception')
        
        # DECLARACIÓN DE ATRIBUTOS -----
        # Parámetro del LiDAR
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('max_range', 30.0)
        # Parámetros del DBSCAN
        self.declare_parameter('eps', 1.0)
        self.declare_parameter('min_samples', 15)
        # CONOCIMIENTO EXPERTO PARA 1:10 ---- 
        # (Coche escala 1:10 - 206mm de altura)
        self.declare_parameter('car_height', 0.206)
        # (Coche escala 1:10 - 535mm de longitud)
        self.declare_parameter('max_obs_size', 0.550) 
        
        # ASIGNACIÓN DE ATRIBUTOS -----
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.eps = self.get_parameter('eps').value
        self.min_samples = self.get_parameter('min_samples').value
        self.car_height = self.get_parameter('car_height').value
        self.max_obs_size = self.get_parameter('max_obs_size').value

        # Instancia del Detector LShape
        self.l_shape_detector = LShapeDetector(
            eps=self.eps,
            min_samples=self.min_samples,
            max_obs_size=self.max_obs_size
        )

        # Instancia del Tracker
        self.tracker = Tracker(
            # Función distancia
            distance_function = "euclidean",
            # Distancia umbral de tracker
            distance_threshold = 3.0,
            # Frames en los que vive un obstáculo sin ser detectado
            hit_counter_max = 15,
            # Frames mínimos para confirmar que es un obstáculo
            initialization_delay = 3
        )

        # Se suscribe a los datos brutos del LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan, 
            '/r1/laser1', 
            self.scan_callback, 
            10
        )

        # Se publican los centroides de los clústeres para verlos en RViz
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/perception/clusters',
            10
        )
        self.get_logger().info("Nodo DBSCANDectector iniciado. Esperando datos...")

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        mask = (ranges > self.min_range) & (ranges < self.max_range) & (~np.isinf(ranges)) & (~np.isnan(ranges))
        r = ranges[mask]
        theta = angles[mask]
        
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        points = np.column_stack((x, y))
        
        if len(points) == 0: return

        raw_detections = self.l_shape_detector.detect(points)
        
        detection_list = []
        
        for det_data in raw_detections:
            det = Detection(
                points = np.array([[det_data['center_x'], det_data['center_y']]]),
                data={
                    'size': det_data['size'],
                    'theta': det_data['theta'],
                    'label': det_data['label']
                }
            )
            detection_list.append(det)

        tracked_objects = self.tracker.update(detections = detection_list)

        self.publish_markers(tracked_objects, msg.header)

    def publish_markers(self, tracked_objects, header):
        clear_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        clear_array.markers.append(delete_marker)
        self.marker_pub.publish(clear_array)

        marker_array = MarkerArray()

        for obj in tracked_objects:
            center_x = obj.estimate[0][0]
            center_y = obj.estimate[0][1]

            if obj.last_detection is not None:
                obj.memory_size = obj.last_detection.data['size']
                obj.memory_theta = obj.last_detection.data['theta']
            
            size = getattr(obj, 'memory_size', 1.0)
            theta_opt = getattr(obj, 'memory_theta', 0.0)

            marker = Marker()
            marker.header = header
            marker.ns = 'tracked_obstacles'
            # marker.ns = "clusters_dbscan" 
            marker.id = int(obj.id) * 2
            # marker.id = cluster_id
            
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(center_x)
            marker.pose.position.y = float(center_y)
            # Elevar ligeramente el cubo para que no se entierre en el mapa
            marker.pose.position.z = float(self.car_height) / 2.0
            
            # Aplicamos la rotación (yaw) calculada por el algoritmo
            q = yaw2quaternion(theta_opt)
            marker.pose.orientation.x = float(q[0])
            marker.pose.orientation.y = float(q[1])
            marker.pose.orientation.z = float(q[2])
            marker.pose.orientation.w = float(q[3])
            
            # ForzaETH asume que los obstáculos son cuadrados. Usamos el tamaño calculado en X, Y, Z.
            marker.scale.x = float(size)
            marker.scale.y = float(size)
            marker.scale.z = float(self.car_height)
            
            # Un verde para indicar tracking exitoso
            marker.color.a = 0.6
            if obj.live_points.any():  # Simplificación: si tiene puntos recientes
                marker.color.r = 0.0
                marker.color.g = 1.0 # Verde = Real
            else:
                marker.color.r = 1.0
                marker.color.g = 0.5 # Naranja = Memoria / Predicción
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)
            # cluster_id += 1

            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = "tracked_obstacles_ids"
            text_marker.id = (int(obj.id) * 2) + 1

            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            # Posicionamos el texto justo por encima del cubo
            text_marker.pose.position.x = float(center_x)
            text_marker.pose.position.y = float(center_y)
            text_marker.pose.position.z = float(self.car_height) + 0.5 
            
            text_marker.scale.z = 0.5
            
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0

            text_marker.text = f"ID: {obj.id}"
            
            marker_array.markers.append(text_marker)
        
        if marker_array.markers:
            self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Deteniendo el nodo...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()