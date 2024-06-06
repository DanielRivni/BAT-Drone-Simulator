from cpu import CPU
from lidar import Lidar
from point import Point
from tools import Tools
from world_params import WorldParams

class Drone:
    def __init__(self, real_map):
        self.real_map = real_map
        self.start_point = real_map.start_point
        self.point_from_start = Point(0, 0)
        self.sensor_optical_flow = Point(0, 0)
        self.lidars = []
        self.rotation = 0
        self.gyro_rotation = 0
        self.speed = 0.2
        self.stopped = False
        self.cpu = CPU(100, "Drone")
        self.cpu.add_function(self.update)
        self.is_avoiding_collision = False

    def play(self):
        self.stopped = False
        self.cpu.play()

    def stop(self):
        self.stopped = True
        self.cpu.stop()

    def add_lidar(self, degrees):
        lidar = Lidar(self, degrees)
        self.lidars.append(lidar)
        self.cpu.add_function(lidar.get_simulation_distance)

    def get_point_on_map(self):
        x = self.start_point.x + self.point_from_start.x
        y = self.start_point.y + self.point_from_start.y
        return Point(x, y)

    def update(self, delta_time):
        if self.stopped:
            return

        distance_moved = (self.speed * 100) * (delta_time / 1000.0)
        new_position = Tools.get_point_by_distance(self.point_from_start, self.rotation, distance_moved)

        # Check collision
        new_x = int(new_position.x + self.start_point.x)
        new_y = int(new_position.y + self.start_point.y)

        if not self.real_map.is_collide(new_x, new_y):
            self.point_from_start = new_position
            noise_to_distance = Tools.noise_between(WorldParams.MIN_MOTION_ACCURACY, WorldParams.MAX_MOTION_ACCURACY, False)
            self.sensor_optical_flow = Tools.get_point_by_distance(self.sensor_optical_flow, self.rotation, distance_moved * noise_to_distance)
            noise_to_rotation = Tools.noise_between(WorldParams.MIN_ROTATION_ACCURACY, WorldParams.MAX_ROTATION_ACCURACY, False)
            self.gyro_rotation = self.format_rotation(self.gyro_rotation + (1 - noise_to_rotation) * delta_time / 60000)
            self.is_avoiding_collision = False
        else:
            self.avoid_collision(delta_time)

    def avoid_collision(self, delta_time):
        if not self.is_avoiding_collision:
            self.is_avoiding_collision = True
        # Try to rotate left or right to avoid the obstacle
        if self.lidars[1].current_distance > self.lidars[2].current_distance:
            self.rotate_left(delta_time)
        else:
            self.rotate_right(delta_time)
        self.speed_up(delta_time)

    @staticmethod
    def format_rotation(rotation_value):
        rotation_value %= 360
        if rotation_value < 0:
            rotation_value = 360 + rotation_value
        return rotation_value

    def rotate_left(self, delta_time):
        rotation_changed = WorldParams.ROTATION_PER_SECOND * delta_time / 1000.0
        self.rotation = self.format_rotation(self.rotation + rotation_changed)
        self.gyro_rotation = self.format_rotation(self.gyro_rotation + rotation_changed)

    def rotate_right(self, delta_time):
        rotation_changed = -WorldParams.ROTATION_PER_SECOND * delta_time / 1000.0
        self.rotation = self.format_rotation(self.rotation + rotation_changed)
        self.gyro_rotation = self.format_rotation(self.gyro_rotation + rotation_changed)

    def speed_up(self, delta_time):
        self.speed += WorldParams.ACCELERATE_PER_SECOND * delta_time / 1000.0
        if self.speed > WorldParams.MAX_SPEED:
            self.speed = WorldParams.MAX_SPEED

    def slow_down(self, delta_time):
        self.speed -= WorldParams.ACCELERATE_PER_SECOND * delta_time / 1000.0
        if self.speed < 0:
            self.speed = 0
