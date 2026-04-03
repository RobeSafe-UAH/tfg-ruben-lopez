import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
from sklearn.cluster import DBSCAN
from norfair import Detection, Tracker

def get_quaternion_from_yaw(yaw):
    return [0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)]

class Dectector(Node):
    """
        Este clase 'Detector' inicialmente solo implementa 'L-shape detector'.
        En la última revisión se añade, aunque provisionalmente, el tracker
        norfair. Pero el objetivo es abstraer el detector en una librería
        independiente para facilitar la legibilidad del nodo. Lo mismo con
        el tracker, si fuese necesario.
        Se marcan las líneas de código con un asterísco '*' para indicar qué
        pertenece al tracker y así identificarlo fácilmente para su posterior
        remplazo.
    """
    def __init__(self):
        super().__init__('detector_w_track')
        
        # DECLARACIÓN DE ATRIBUTOS -----
        # Parámetro del LiDAR
        self.declare_parameter('min_range', 0.01)
        self.declare_parameter('max_range', 30.0)
        # Parámetros del DBSCAN
        self.declare_parameter('eps', 1.0)
        self.declare_parameter('min_samples', 15)
        # CONOCIMIENTO EXPERTO PARA 1:10 ---- (Coche escala 1:10 - 535mm)
        # AHORA MISMO, TIPO ACKERMAN DE ~2.5m DE LARGO
        self.declare_parameter('car_height', 1)
        # Parámetro para filtrar obstáculos
        # AHORA MISMO, TIPO ACKERMAN DE ~2.5m DE LARGO
        self.declare_parameter('max_obs_size', 3) 
        
        # ASIGNACIÓN DE ATRIBUTOS -----
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.eps = self.get_parameter('eps').value
        self.min_samples = self.get_parameter('min_samples').value
        self.car_height = self.get_parameter('car_height').value
        self.max_obs_size = self.get_parameter('max_obs_size').value

        # *
        # Instancia del Detector
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

    def fit_bounding_box(self, cluster_points):
        """
        
        """
        
        # Algoritmo L-Shape Fitting

        # Evaluar posibles ángulos entre 0 y 90 grados
        # Crea un vector de 0 a 'pi/2-pi/180' con 90 valores.
        theta = np.linspace(0, np.pi/2-np.pi/180, 90)
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        
        # Productos escalares entre los puntos y el ángulo theta
        # Proyecciones ortogonales de los puntos (distancia perpendicular al vector)
        # Para el vector [cos_theta, sin_theta] obtenemos un vector ortogonal a él 
        # tal que [-sin_theta, cos_theta]. El cual forma 90º respecto a él en  
        # sentido antihorario.
        distance1 = np.dot(cluster_points, [cos_theta, sin_theta])
        distance2 = np.dot(cluster_points, [-sin_theta, cos_theta])
        
        # amax: Máximo valor del vector en el eje 0 (Eje = dimensión del array)
        # amin: Mínimo valor del vector en el eje 0
        # Cómo de alejada está cada proyección respecto de la de mayor valor (y menor)
        # Si D es pequeño = Puntos cerca del borde
        # Si D es grande = Puntos alejados del borde (dispersos)
        D10 = -distance1 + np.amax(distance1, axis=0)
        D11 = distance1 - np.amin(distance1, axis=0)
        D20 = -distance2 + np.amax(distance2, axis=0)
        D21 = distance2 - np.amin(distance2, axis=0)
        
        # argmin: devuelve el índice del valor mínimo
        # linalg.norm: calcula la norma del vector por columnas (axis = 0)
        min_array = np.argmin([np.linalg.norm(D10, axis=0), np.linalg.norm(D11, axis=0)], axis=0)
        # Matriz traspuesta
        D10 = np.transpose(D10)
        D11 = np.transpose(D11)
        D10[min_array == 1] = D11[min_array == 1]
        D10 = np.transpose(D10)
        
        # ídem
        min_array = np.argmin([np.linalg.norm(D20, axis=0), np.linalg.norm(D21, axis=0)], axis=0)
        D20 = np.transpose(D20)
        D21 = np.transpose(D21)
        D20[min_array == 1] = D21[min_array == 1]
        D20 = np.transpose(D20)
        
        # Mínimo entre vectores
        D = np.minimum(D10, D20)
        # Saturación de valor. Máscara 'D < min_dist'.
        # Sustituir por 'min_dist' cualquier valor menor a este
        D[D < self.min_range] = self.min_range

        # Extraer el ángulo óptimo y las dimensiones
        # theta_opt: Estimación de orientación del obstáculo
        # reciprocal: Calcula para cada elemento 'x' del vector D '1/x' (Cuanto menor x [menor dist.]
        # mayor será el recíproco).
        # argmax: devuelve el índice del valor máximo
        theta_opt = np.argmax(np.sum(np.reciprocal(D), axis=0)) * np.pi/180
        
        # Proyecciones ortogonales del vector con la orientación 'theta_opt' sobre los puntos
        # del cluster.
        distances1 = np.dot(cluster_points, [np.cos(theta_opt), np.sin(theta_opt)])
        distances2 = np.dot(cluster_points, [-np.sin(theta_opt), np.cos(theta_opt)])
        
        # Extrae el valor máximo del vector
        max_dist1 = np.max(distances1)
        # Extrae el valor mínimo del vector
        min_dist1 = np.min(distances1)
        max_dist2 = np.max(distances2)
        min_dist2 = np.min(distances2)

        # Detectar esquinas en sentido antihorario
        # np.var: Devuelve la varianza del vector
        # Si varianza es alta = puntos dispersos (línea larga)
        # Si varianza es baja = puntos poco dispersos (línea corta)
        if (np.var(distances2) > np.var(distances1)):
            # Si la norma
            if (np.linalg.norm(-distances1+max_dist1) < np.linalg.norm(distances1-min_dist1)):
                
                # Aplicación de rotaciones:
                #   'x = cos(theta) * d1 - sin(theta) * d2'
                #   'y = sin(theta) * d1 + cos(theta) * d2'
                corner1 = np.array([np.cos(theta_opt)*max_dist1-np.sin(theta_opt)*min_dist2,
                                    np.sin(theta_opt)*max_dist1+np.cos(theta_opt)*min_dist2])
                corner2 = np.array([np.cos(theta_opt)*max_dist1-np.sin(theta_opt)*max_dist2,
                                    np.sin(theta_opt)*max_dist1+np.cos(theta_opt)*max_dist2])
            else:
                corner1 = np.array([np.cos(theta_opt)*min_dist1-np.sin(theta_opt)*max_dist2,
                                    np.sin(theta_opt)*min_dist1+np.cos(theta_opt)*max_dist2])
                corner2 = np.array([np.cos(theta_opt)*min_dist1-np.sin(theta_opt)*min_dist2,
                                    np.sin(theta_opt)*min_dist1+np.cos(theta_opt)*min_dist2])
        else: 
            if (np.linalg.norm(-distances2+max_dist2) < np.linalg.norm(distances2-min_dist2)):
                corner1 = np.array([np.cos(theta_opt)*max_dist1-np.sin(theta_opt)*max_dist2,
                                    np.sin(theta_opt)*max_dist1+np.cos(theta_opt)*max_dist2])
                corner2 = np.array([np.cos(theta_opt)*min_dist1-np.sin(theta_opt)*max_dist2,
                                    np.sin(theta_opt)*min_dist1+np.cos(theta_opt)*max_dist2])
            else:
                corner1 = np.array([np.cos(theta_opt)*min_dist1-np.sin(theta_opt)*min_dist2,
                                    np.sin(theta_opt)*min_dist1+np.cos(theta_opt)*min_dist2])
                corner2 = np.array([np.cos(theta_opt)*max_dist1-np.sin(theta_opt)*min_dist2,
                                    np.sin(theta_opt)*max_dist1+np.cos(theta_opt)*min_dist2])
        
        # Vector que representa la cara visible del obstáculo
        colVec = np.array([corner2[0]-corner1[0], corner2[1]-corner1[1]])
        # Vector ortogonal al anterior, dirigido hacia el lado oculto
        orthVec = np.array([-colVec[1], colVec[0]])
        # Centro del obstáculo
        center = corner1 + 0.5*colVec + 0.5*orthVec
        # Tamaño del obstáculo
        size = np.linalg.norm(colVec)

        return center[0], center[1], size, theta_opt

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        mask = (ranges > self.min_range) & (ranges < self.max_range) & (~np.isinf(ranges)) & (~np.isnan(ranges))
        r = ranges[mask]
        theta = angles[mask]
        
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        points = np.column_stack((x, y))
        
        if len(points) == 0:
            return

        clustering = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(points)
        labels = clustering.labels_
        unique_labels = set(labels)
        # self.publish_markers(points, labels, msg.header)

        # cluster_id = 0
        # *
        detection_list = []

        # *
        for label in unique_labels:
            if label == -1: continue

            cluster_points = points[labels == label]
            try:
                center_x, center_y, size, theta_opt = self.fit_bounding_box(cluster_points)
            except Exception as e:
                continue
            
            if size > self.max_obs_size:
                continue
            
            det = Detection(
                points = np.array([[center_x, center_y]]),
                data = {'size': size, 'theta': theta_opt, 'label': label}
            )
            detection_list.append(det)
        
        # *
        tracked_objects = self.tracker.update(detections = detection_list)
        # *
        self.publish_markers(tracked_objects, msg.header)

    def publish_markers(self, tracked_objects, header):
        clear_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        clear_array.markers.append(delete_marker)
        self.marker_pub.publish(clear_array)

        marker_array = MarkerArray()

        # unique_labels = set(labels)
        
        # cluster_id = 0
        # for label in unique_labels:
        #     if label == -1:
        #         continue
                
        #     cluster_points = points[labels == label]
            
        #     # --- Aquí integramos el Bounding Box ---
        #     try:
        #         center_x, center_y, size, theta_opt = self.fit_bounding_box(cluster_points)
        #     except Exception as e:
        #         # Por si un clúster es demasiado pequeño y el L-Shape falla
        #         self.get_logger().debug(f"Fallo al ajustar L-Shape: {e}")
        #         continue
                
        #     # --- Comprobación de tamaño ---
        #     if size > self.max_obs_size:
        #         # Si el objeto es más grande que nuestro límite (ej. un muro), lo ignoramos
        #         continue
        
        # *
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
            q = get_quaternion_from_yaw(theta_opt)
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
    node = Dectector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Deteniendo el nodo...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()