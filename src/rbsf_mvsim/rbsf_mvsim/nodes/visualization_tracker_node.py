import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from norfair import Detection, Tracker

from .libraries.converts_library import quaternion2yaw, yaw2quaternion

class TrackerNode(Node):
    def __init__(self):
        super().__init__('visualization_tracker_node')

    # --- DECLARACIÓN DE PARÁMETROS DEL TRACKER ---
        self.declare_parameter('distance_function', 'euclidean')
        self.declare_parameter('distance_threshold', 0.5)
        self.declare_parameter('hit_counter_max', 15)
        self.declare_parameter('initialization_delay', 3)
        self.declare_parameter('car_height', 0.206)

        self.car_height = self.get_parameter('car_height').value
        
        self.tracker = Tracker(
            distance_function=self.get_parameter('distance_function').value,
            distance_threshold=self.get_parameter('distance_threshold').value,
            hit_counter_max=self.get_parameter('hit_counter_max').value,
            initialization_delay=self.get_parameter('initialization_delay').value
        )

        # Escuchamos al Detector
        self.raw_sub = self.create_subscription(
            MarkerArray,
            '/perception/raw_obstacles',
            self.raw_callback,
            15
        )
        
        # Publicamos los marcadores finales para RViz
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/perception/tracked_obstacles',
            15
        )
        
        self.get_logger().info("Tracker Node iniciado. Escuchando en /perception/raw_detections")

    def raw_callback(self, msg: MarkerArray):
        detection_list = []
        valid_header = None

        for m in msg.markers:
            if m.action == Marker.DELETEALL:
                # Opcional: rescatar el header de aquí si el detector ya lo manda bien
                if m.header.frame_id != '':
                    valid_header = m.header
                continue

            valid_header = m.header
            theta = quaternion2yaw(m.pose.orientation)
            size = m.scale.x

            det = Detection(
                points=np.array([[m.pose.position.x, m.pose.position.y]]),
                data={'size': size, 'theta': theta}
            )
            detection_list.append(det)

        tracked_objects = self.tracker.update(detections=detection_list)

        if valid_header is not None:
            self.publish_markers(
                tracked_objects,
                valid_header
            )
    
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
                new_size = obj.last_detection.data['size']
                new_theta = obj.last_detection.data['theta']

                if not hasattr(obj, 'memory_size'):
                    obj.memory_size = new_size
                    obj.memory_theta = new_theta
                else:
                    # Aplicamos el filtro de suavizado (EMA)
                    # 'alpha' controla la suavidad. 
                    # 0.1 = Muy suave (reacciona lento a cambios reales).
                    # 0.8 = Poco suave (sigue mucho a la detección cruda).
                    alpha = 0.05
                    
                    obj.memory_size = (1.0 - alpha) * obj.memory_size + alpha * new_size
                    obj.memory_theta = (1.0 - alpha) * obj.memory_theta + alpha * new_theta
            
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
            marker.pose.position.z = 0.0
            
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
    rclpy.spin(TrackerNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()