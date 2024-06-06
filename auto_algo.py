import random
import numpy as np
import matplotlib.pyplot as plt
from cpu import CPU
from drone import Drone
from tools import Tools, PixelState
from point import Point
from world_params import WorldParams

# Set fixed random seed for reproducibility
random.seed(80)
np.random.seed(80)


class AutoAlgo:
    def __init__(self, real_map):
        self.real_map = real_map
        self.map_size = (real_map.height, real_map.width)
        self.map = np.full(self.map_size, PixelState.UNEXPLORED)
        self.drone = Drone(real_map)
        self.drone.add_lidar(0)
        self.drone.add_lidar(90)
        self.drone.add_lidar(-90)
        self.is_rotating = 0
        self.ai_cpu = CPU(200, "Auto_AI")
        self.ai_cpu.add_function(self.update)
        self.points = []
        self.fig, self.ax = plt.subplots()
        self.following_wall = False

    def play(self):
        self.drone.play()
        self.ai_cpu.play()
        self.animate()

    def update(self, delta_time):
        self.update_visited()
        self.update_map_by_lidars()
        self.ai(delta_time)
        if self.is_rotating != 0:
            self.update_rotating(delta_time)

    def update_map_by_lidars(self):
        drone_point = self.drone.get_point_on_map()
        from_point = Point(drone_point.x, drone_point.y)
        for lidar in self.drone.lidars:
            rotation = self.drone.gyro_rotation + lidar.degrees
            for distance_in_cm in range(int(lidar.current_distance)):
                p = Tools.get_point_by_distance(from_point, rotation, distance_in_cm)
                if self.real_map.map[int(p.y), int(p.x)] == 1:  # Only update free space
                    self.set_pixel(p.x, p.y, PixelState.EXPLORED)
                else:
                    self.set_pixel(p.x, p.y, PixelState.BLOCKED)
            if lidar.current_distance > 0 and lidar.current_distance < WorldParams.LIDAR_LIMIT - WorldParams.LIDAR_NOISE:
                p = Tools.get_point_by_distance(from_point, rotation, lidar.current_distance)
                if self.real_map.map[int(p.y), int(p.x)] == 1:  # Only update free space
                    self.set_pixel(p.x, p.y, PixelState.EXPLORED)

    def update_visited(self):
        drone_point = self.drone.get_point_on_map()
        from_point = Point(drone_point.x, drone_point.y)
        if self.real_map.map[int(from_point.y), int(from_point.x)] == 1:  # Only update free space
            self.set_pixel(from_point.x, from_point.y, PixelState.VISITED)

    def set_pixel(self, x, y, state):
        xi, yi = int(x), int(y)
        if 0 <= xi < self.map_size[1] and 0 <= yi < self.map_size[0]:
            if state == PixelState.VISITED or self.map[yi, xi] == PixelState.UNEXPLORED:
                self.map[yi, xi] = state

    def ai(self, delta_time):
        self.bug_algorithm(delta_time)

    def bug_algorithm(self, delta_time):
        front_lidar = self.drone.lidars[0].current_distance
        left_lidar = self.drone.lidars[1].current_distance
        right_lidar = self.drone.lidars[2].current_distance

        if not self.following_wall:
            if front_lidar < 30:  # Immediate obstacle ahead
                self.following_wall = True
                if left_lidar > right_lidar:
                    self.drone.rotate_left(100)
                else:
                    self.drone.rotate_right(100)
                self.drone.slow_down(100)
            else:
                self.drone.speed_up(100)
        else:
            if front_lidar > 50:  # If there is no obstacle ahead, stop following the wall
                self.following_wall = False
                self.drone.speed_up(100)
            else:
                if left_lidar < 30 and right_lidar > 30:  # Follow the wall on the left side
                    self.drone.rotate_right(50)
                    self.drone.slow_down(50)
                elif right_lidar < 30 and left_lidar > 30:  # Follow the wall on the right side
                    self.drone.rotate_left(50)
                    self.drone.slow_down(50)
                else:  # Move forward while following the wall
                    self.drone.speed_up(100)

    def animate(self):
        from matplotlib.animation import FuncAnimation

        def update_plot(frame):
            self.ax.clear()
            img = np.zeros((*self.map_size, 3), dtype=np.uint8)
            img[self.real_map.map == 1] = [255, 255, 255]  # Free space in white
            img[self.map == PixelState.EXPLORED] = [255, 255, 0]  # Explored space in yellow
            img[self.map == PixelState.VISITED] = [255, 0, 0]  # Visited space in red
            self.ax.imshow(img)
            drone_point = self.drone.get_point_on_map()
            self.ax.plot(drone_point.x, drone_point.y, 'ro')  # Plot the drone's actual position

            # Draw LIDAR lines
            from_point = Point(drone_point.x, drone_point.y)
            for lidar in self.drone.lidars:
                rotation = self.drone.gyro_rotation + lidar.degrees
                lidar_distance = WorldParams.LIDAR_LIMIT
                to_point = Tools.get_point_by_distance(from_point, rotation, lidar_distance)
                self.ax.plot([from_point.x, to_point.x], [from_point.y, to_point.y], 'b-', linewidth=2)

            self.ax.set_title('Explored Map')
            self.ax.set_xlim(0, self.real_map.width)  # Set x-axis limits
            self.ax.set_ylim(self.real_map.height, 0)  # Set y-axis limits (flipped)
            self.ax.set_aspect('equal')  # Ensure equal aspect ratio

        ani = FuncAnimation(self.fig, update_plot, interval=100)
        plt.show()
