import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import message_filters
import csv
import os

RBSF_DIR = os.path.join('/','sim_ws','src','rbsf_mvsim')

class LidarLoggerNode(Node):
    """
    Nodo de ROS2 que escribe la información de un LiDAR2D que se publica
    en el Topic '/laser1' y la escribe en un fichero .csv
    """
    # Constructor
    def __init__(self):
        super().__init__('lidar_logger_node')

        # Configuración del fichero .csv
        self.filename = os.path.join(RBSF_DIR,'LiDARs_data','lidar_data.csv')
        self.file_exists = os.path.isfile(self.filename)
        self.file = open(self.filename, 'a', newline = '')
        self.writer = csv.writer(self.file)

        # Únicamnete escribe las cabeceras si no existe el fichero previamente
        if not self.file_exists: 
            self.writer.writerow(['timestamp', 'index', 'values'])

        # Suscripcion a topic '/laser1'
        self.subscription = self.create_subscription(
            LaserScan,
            '/laser1',
            self.callback,
            10
        )

        self.get_logger().info(f"Nodo iniciado. Grabando datos en {self.filename}...")
    
    def callback(self, msg_l1):

        timestamp = f"{msg_l1.header.stamp.sec}.{msg_l1.header.stamp.nanosec}"
        
        for i, r in enumerate(msg_l1.ranges):
            self.writer.writerow([timestamp, i, r])

        self.get_logger().debug(f"Captura guardada para T={timestamp}")
    
    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args = None):
    rclpy.init(args=args)
    node = LidarLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()