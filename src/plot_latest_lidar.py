import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from sklearn.cluster import DBSCAN
import ast
import os

LIDAR_DIR = os.path.join('/','sim_ws','src','LiDARs_data')
FILENAME=os.path.join(LIDAR_DIR,'lidar_data.csv')

ANGLE_MIN = -np.pi * 3/4  # Ejemplo: -135 grados
ANGLE_MAX = np.pi * 3/4   # Ejemplo: 135 grados
MAX_RANGE = 10.0          # Ignorar puntos más allá de esto (o 'inf')

EPS = 1          # Distancia máxima entre puntos del mismo cluster
MIN_SAMPLES = 5    # Puntos mínimos para considerar un objeto

def on_key(event):
    current_val = slider.val
    if event.key == 'right':
        new_val = current_val + 1
    elif event.key == 'left':
        new_val = current_val - 1
    else:
        return 
    
    if 0 <= new_val < num_frames:
        slider.set_val(new_val)

def get_cartesian_clusters(frame_idx):
    data_frame = grupos[frame_idx]
    data_frame = data_frame.sort_values('index')

    ranges = data_frame['values'].values

    angles = np.linspace(ANGLE_MIN,ANGLE_MAX, len(ranges))

    mask = (ranges > 0.1) & (ranges < MAX_RANGE) & (~np.isinf(ranges))
    r = ranges[mask]
    theta = angles[mask]

    x = r * np.cos(theta)
    y = r * np.sin(theta)

    points = np.column_stack((x, y))

    if len(points) == 0:
        return np.array([]), np.array([]), np.array([])

    clustering = DBSCAN(eps=EPS, min_samples=MIN_SAMPLES).fit(points)
    return x, y, clustering.labels_

def update(val):
    frame_idx = int(slider.val)
    new_x, new_y, new_labels = get_cartesian_clusters(frame_idx)

    scatter.set_offsets(np.column_stack((new_x, new_y)))
    scatter.set_array(new_labels)

    ts = df.iloc[frame_idx]['timestamp']
    title = ax.set_title(f"Escaneo: {frame_idx} | Puntos detectados: {len(new_x)}")

    if len(new_labels) > 0:
        scatter.set_clim(vmin=min(new_labels), vmax=max(max(new_labels), 1))
    fig.canvas.draw_idle()

df = pd.read_csv(FILENAME)
df['timestamp'] = df['timestamp'].round(2)
grupos = [group for _, group in df.groupby('timestamp')]
num_frames = len(grupos)

fig, ax = plt.subplots(figsize= (8 ,8))
plt.subplots_adjust(bottom= 0.2)

x, y, labels = get_cartesian_clusters(0)
scatter = ax.scatter(x, y, c=labels, cmap='tab10', s=10)
ax.set_aspect('equal')
ax.set_xlim(-MAX_RANGE,MAX_RANGE)
ax.set_ylim(-MAX_RANGE,MAX_RANGE)
ax.grid(True)
title = ax.set_title(f"Frame: 0 | Puntos: {len(x)}")

ax_slider = plt.axes([0.2, 0.05, 0.6, 0.03])
slider = Slider(ax_slider, 'Escaneo', 0, num_frames-1, valinit=0, valfmt='%d')

slider.on_changed(update)
fig.canvas.mpl_connect('key_press_event', on_key)
plt.show()