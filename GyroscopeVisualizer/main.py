import pygame
import logging
import sys
import threading
import serial
from pygame import Vector3, Vector2
import numpy as np
from numpy import sin, cos, tan, sqrt


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
    with serial.Serial("COM4", 9600, timeout=1) as ser:
        logging.info("Thread Started.")
        if ser.readline().decode('utf-8').strip() == "sensor_not_connected":
            logging.exception("Sensor is not connected properly.")
        while True:
            try:
                parts = ser.readline().decode('utf-8', errors='ignore').strip().split()
                line = [int(part) for part in parts if part.strip() != '']
                with lock:
                    print(latest_line)
                    latest_line = line
            except Exception as e:
                logging.exception(f"Serial error: {e}")


threading.Thread(target=serial_reader, daemon=True).start()

# ----- main loop -----

pygame.init()
pygame.font.init()
screensize = [500, 400]
screen = pygame.display.set_mode(screensize, pygame.RESIZABLE)
clock = pygame.time.Clock()
loop_running = True
FPS = 60

# fonts
font = pygame.font.SysFont("Helvetica", 20)

# others
cubesize = (1, 2, 1)


def project_3d(point):
    projection_matrix = np.matrix

    projection_matrix = np.matrix([
        [1, 0, 0],
        [0, 1, 0]
    ])

    angle = 0
    rotation_z = np.matrix([
        [cos(angle), -sin(angle), 0],
        [sin(angle), cos(angle), 0],
        [0, 0, 1],
    ])

    angle = 0
    rotation_y = np.matrix([
        [cos(angle), 0, sin(angle)],
        [0, 1, 0],
        [-sin(angle), 0, cos(angle)],
    ])

    rotation_x = np.matrix([
        [1, 0, 0],
        [0, cos(angle), -sin(angle)],
        [0, sin(angle), cos(angle)],
    ])

    rotated2d = np.dot(rotation_z, point.reshape((3, 1)))
    rotated2d = np.dot(rotation_y, rotated2d)
    rotated2d = np.dot(rotation_x, rotated2d)

    return np.dot(projection_matrix, rotated2d)


def draw_projected_point(point2d, color="WHITE"):
    pygame.draw.circle(screen, color, Vector2(screensize)/2+Vector2(point2d), 5)


def draw_projected_line(start, end, color="WHITE"):
    pygame.draw.line(screen, color, Vector2(screensize)/2+Vector2(start), Vector2(screensize)/2+Vector2((end[0], -end[1])), 2)


def draw_moving_cube(angles):
    angle = angles[2]
    rotation_z = np.matrix([
        [cos(angle), -sin(angle), 0],
        [sin(angle), cos(angle), 0],
        [0, 0, 1],
    ])

    angle = angles[1]
    rotation_y = np.matrix([
        [cos(angle), 0, sin(angle)],
        [0, 1, 0],
        [-sin(angle), 0, cos(angle)],
    ])

    angle = angles[0]
    rotation_x = np.matrix([
        [1, 0, 0],
        [0, cos(angle), -sin(angle)],
        [0, sin(angle), cos(angle)],
    ])

    vertices = [np.matrix([x, y, z]) for x in (0, cubesize[0]) for y in (0, cubesize[1]) for z in (0, cubesize[2])]

    rotated_vertices = [vertex @ rotation_x @ rotation_y @ rotation_z for vertex in vertices]

    for dot in vertices:
        draw_projected_point(project_3d(dot))



logging.info("Starting PyGame loop.")
while loop_running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.VIDEORESIZE:
            screensize = [event.w, event.h]
            screen = pygame.display.set_mode(screensize, pygame.RESIZABLE)
            logging.info("Resized window to [{}, {}].".format(event.w, event.h))

    screen.fill([0, 0, 0])

    with lock:
        if len(latest_line) != 0:
            current_last_line = latest_line
        else:
            if "current_last_line" not in locals():
                current_last_line = [0, 0, 0]


    draw_moving_cube([10, 10, 10])


    pygame.display.flip()
    pygame.display.set_caption("Gyroscope Visualizer running at {:0.2f} FPS".format(clock.get_fps()))
    clock.tick(FPS)

