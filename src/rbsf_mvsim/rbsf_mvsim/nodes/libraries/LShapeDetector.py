import numpy as np
from sklearn.cluster import DBSCAN

class LShapeDetector:
    """
        Clase independiente que encapsula el clustering y el ajuste L-Shape.
    """

    def __init__(self, eps=1.0, min_samples=15, max_obs_size=0.55):
        self.eps = eps
        self.min_samples = min_samples
        self.max_obs_size = max_obs_size

    def fit_bounding_box(self, cluster_points):
        """
            Función que implementa el algoritmo que busca el rectángulo
            que encaja con el obstáculo detectado.
        """
        
        min_dist = 0.01
        
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
        D[D < min_dist] = min_dist

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
    
    def detect(self, points):
        """
        Método principal. Ejecuta DBSCAN y L-Shape devolviendo una lista de diccionarios.
        """
        if len(points) == 0:
            return []

        clustering = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(points)
        labels = clustering.labels_
        unique_labels = set(labels)
        
        valid_detections = []
        
        for label in unique_labels:
            if label == -1: continue

            cluster_points = points[labels == label]
            try:
                center_x, center_y, size, theta_opt = self.fit_bounding_box(cluster_points)
            except Exception:
                continue
            
            if size > self.max_obs_size:
                continue
            
            # Guardamos los resultados estructurados
            valid_detections.append({
                'center_x': center_x,
                'center_y': center_y,
                'size': size,
                'theta': theta_opt,
                'label': label
            })
            
        return valid_detections
