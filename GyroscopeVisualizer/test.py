import pygame
import logging
import sys
import threading
import serial

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation



logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] [%(threadName)s] %(message)s',
)


# ---- shared data -----
latest_line = ''
lock = threading.Lock()

# ---- backgroudn thread -----
def serial_reader():
    global latest_line
    with serial.Serial("COM6", 9600, timeout=1) as ser:
        logging.info("Thread Started.")
        if ser.readline().decode('utf-8').strip() == "sensor_not_connected":
            logging.exception("Sensor is not connected properly.")
        while True:
            try:
                parts = ser.readline().decode('utf-8', errors='ignore').strip().split()
                line = [float(part) for part in parts if part.strip() != '']
                with lock:
                    latest_line = line
            except Exception as e:
                logging.exception(f"Serial error: {e}")


threading.Thread(target=serial_reader, daemon=True).start()

# ----- main loop -----



# others

# Global rotation variable updated from another thread
rotation_state = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

def create_box(center=(0, 0, 0), size=(1.2, 2, 0.5)):
    cx, cy, cz = center
    sx, sy, sz = [s / 2 for s in size]
    return np.array([
        [cx - sx, cy - sy, cz - sz],
        [cx + sx, cy - sy, cz - sz],
        [cx + sx, cy + sy, cz - sz],
        [cx - sx, cy + sy, cz - sz],
        [cx - sx, cy - sy, cz + sz],
        [cx + sx, cy - sy, cz + sz],
        [cx + sx, cy + sy, cz + sz],
        [cx - sx, cy + sy, cz + sz]
    ])

def get_faces(vertices):
    return [
        [vertices[j] for j in [0, 1, 2, 3]],
        [vertices[j] for j in [4, 5, 6, 7]],
        [vertices[j] for j in [0, 1, 5, 4]],
        [vertices[j] for j in [2, 3, 7, 6]],
        [vertices[j] for j in [1, 2, 6, 5]],
        [vertices[j] for j in [4, 7, 3, 0]]
    ]

def rotation_matrix(roll, pitch, yaw):
    r, p, y = np.radians([pitch, yaw, roll])
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(r), -np.sin(r)],
                   [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)],
                   [0, 1, 0],
                   [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0],
                   [np.sin(y), np.cos(y), 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx

def draw_live_rotating_box():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    base_vertices = create_box()
    poly = Poly3DCollection([], facecolors='lightgreen', edgecolors='black', alpha=1)
    ax.add_collection3d(poly)

    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    ax.set_box_aspect([1, 1, 1])

    def update(frame):
        with lock:
            if len(latest_line) != 0:
                current_last_line = latest_line
            else:
                if "current_last_line" not in locals():
                    current_last_line = [0, 0, 0]

        roll, pitch, yaw = current_last_line[0], current_last_line[1], current_last_line[2]
        R = rotation_matrix(roll, pitch, yaw)
        rotated = (R @ base_vertices.T).T
        poly.set_verts(get_faces(rotated))
        return poly,

    ani = FuncAnimation(fig, update, interval=1000 / 60, blit=False)
    plt.show()

# Call the visualization (it will read from `rotation_state`)
draw_live_rotating_box()

