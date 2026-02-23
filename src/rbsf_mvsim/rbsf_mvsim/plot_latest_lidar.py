import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

LOG_DIR = os.path.join('/','sim_ws','src','rbsf_mvsim','LiDARs_data')

def plot_latest_lidar(filename=os.path.join(LOG_DIR,'lidar_data.csv')):
    """
    Args: filename (Por defecto) LOG_DIR/lidar_data.csv
    Ret: ultima_captura_lidar.png
    -------------------------------------------------------
    Función de python que genera una imagen .png con la reconstrucción espacial
    del entorno de un robot que incorpora un LiDAR2D.
    """
    # Cargar los datos
    try:
        df = pd.read_csv(filename)
    except FileNotFoundError:
        print(f"Error: No se encontró el archivo {filename}")
        return

    df['values'] = pd.to_numeric(df['values'])

    # Obtener el último timestamp disponible para ver la captura más reciente
    ultimo_ts = df['timestamp'].max()
    datos_recientes = df[df['timestamp'] == ultimo_ts]

    print(f"Visualizando captura del timestamp: {ultimo_ts}")

    # Configurar la gráfica polar
    plt.figure(figsize=(10, 10))
    ax = plt.subplot(111, projection='polar')

    # Dibujar los datos del sensor
    if not datos_recientes.empty:            
        num_puntos = len(datos_recientes)
        # grados = datos_recientes['index']
        angulos_deg = np.linspace(0, 360, num_puntos, endpoint=False)
        angulos_rad = np.deg2rad(angulos_deg+180) 
        distancias = datos_recientes['values'].values
        ax.scatter(angulos_rad, distancias, s=5, c='red',label='LiDAR2D')

    # Configuración estética
    # El frente del robot suele ser el Norte
    ax.set_theta_zero_location('N')
    # Sentido horario
    ax.set_theta_direction(1)
    plt.legend()
    plt.title(f"Captura de LiDAR\n(TS: {ultimo_ts})")
    
    output_name = "ultima_captura_lidar.png"
    plt.savefig(os.path.join(LOG_DIR,output_name))
    print(f"Imagen guardada exitosamente como: {output_name}")
    plt.close()

if __name__ == "__main__":
    plot_latest_lidar()