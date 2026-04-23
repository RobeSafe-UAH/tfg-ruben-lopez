import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
from norfair import Detection, Tracker

from .libraries.converts_library import quaternion2yaw, yaw2quaternion

class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')

    # --- DECLARACIÓN DE PARÁMETROS DEL TRACKER ---
        self.declare_parameter('distance_threshold', 3.0)
        self.declare_parameter('hit_counter_max', 15)
        self.declare_parameter('initialization_delay', 3)

        # CONOCIMIENTO EXPERTO PARA 1:10 ---- 
        # (Coche escala 1:10 - 206mm de altura)
        self.declare_parameter('car_height', 0.206)
        self.declare_parameter('static_speed_threshold', 0.5)

        self.car_height = self.get_parameter('car_height').value
        self.static_speed_threshold = self.get_parameter('static_speed_threshold').value

        self.tracker = Tracker(
            distance_function="euclidean",
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
        if not msg.markers:
            return
        
        # 1. Extraemos el tiempo exacto del sensor para calcular velocidades
        current_time = msg.markers[0].header.stamp.sec + (msg.markers[0].header.stamp.nanosec * 1e-9)
        
        detection_list = []

        for m in msg.markers:

            if m.action == Marker.DELETEALL:
                continue

            theta = quaternion2yaw(m.pose.orientation)
            size = m.scale.x

            det = Detection(
                points=np.array([[m.pose.position.x, m.pose.position.y]]),
                data={'size': size, 'theta': theta}
            )
            detection_list.append(det)

        tracked_objects = self.tracker.update(detections=detection_list)

        self.publish_tracked_markers(
            tracked_objects,
            msg.markers[0].header
        )
    
    def publish_tracked_markers(self, tracked_objects, header, current_time):
        
        out_msg = MarkerArray()

        # 1. Empaquetamos la limpieza atómica
        marker_to_clean = Marker()
        marker_to_clean.action = Marker.DELETEALL
        out_msg.markers.append(marker_to_clean)

        for obj in tracked_objects:
            center_x = obj.estimate[0][0]
            center_y = obj.estimate[0][1]

            if obj.last_detection is not None:
                obj.memory_size = obj.last_detection.data['size']
                obj.memory_theta = obj.last_detection.data['theta']
            
            size = getattr(obj, 'memory_size', 1.0)
            theta_opt = getattr(obj, 'memory_theta', 0.0)

            if not hasattr(obj, 'last_time'):
                obj.vx = 0.0
                obj.vy = 0.0
            else:
                dt = current_time - obj.last_time
                if dt > 0:
                    # Velocidad = Delta Posición / Delta Tiempo
                    obj.vx = (center_x - obj.last_cx) / dt
                    obj.vy = (center_y - obj.last_cy) / dt
            
            # Guardamos el estado actual para el siguiente frame
            obj.last_time = current_time
            obj.last_cx = center_x
            obj.last_cy = center_y

            # --- CLASIFICACIÓN (Static Flag) ---
            # np.hypot calcula la magnitud del vector (la velocidad real en m/s)
            speed = np.hypot(obj.vx, obj.vy)
            is_static = speed < self.static_threshold

            # Multiplicamos el ID base para que cada objeto tenga 3 marcadores únicos
            base_id = int(obj.id) * 3

            marker = Marker()
            marker.header = header
            marker.ns = 'tracked_obstacles' 
            marker.id = base_id
            
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
            if is_static:
                # Rojo si es un muro, cono o coche parado
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                # Verde si es un coche en movimiento
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            
            out_msg.markers.append(marker)
            
            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = "tracked_obstacles_ids"
            text_marker.id = base_id + 1
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

            state_str = "STATIC" if is_static else "DYNAMIC"
            text_marker.text = f"ID: {obj.id}\n{state_str}\n{speed:.2f} m/s"
            out_msg.markers.append(text_marker)

            if not is_static:
                arrow = Marker()
                arrow.header = marker.header
                arrow.ns = "tracked_velocity"
                arrow.id = base_id + 2
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                
                # Point 0: Origen (El centro del coche)
                p_start = Point()
                p_start.x = float(center_x)
                p_start.y = float(center_y)
                p_start.z = float(self.car_height) / 2.0
                
                # Point 1: Destino (Hacia dónde apunta, magnitud multiplicada por 1 segundo)
                p_end = Point()
                p_end.x = float(center_x + obj.vx)
                p_end.y = float(center_y + obj.vy)
                p_end.z = float(self.car_height) / 2.0
                
                arrow.points = [p_start, p_end]
                
                # Grosor de la flecha
                arrow.scale.x = 0.1 # Grosor del palo
                arrow.scale.y = 0.2 # Ancho de la cabeza
                arrow.scale.z = 0.2 # Largo de la cabeza
                
                arrow.color.a = 1.0
                arrow.color.r = 1.0; arrow.color.g = 1.0; arrow.color.b = 0.0 # Amarillo
                
                out_msg.markers.append(arrow)
                
        self.marker_pub.publish(out_msg)
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TrackerNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()