import math
import random
from point import Point
from world_params import WorldParams

class Tools:
    @staticmethod
    def get_point_by_distance(from_point, rotation, distance):
        radians = math.pi * (rotation / 180)
        i = distance / WorldParams.CM_PER_PIXEL
        xi = from_point.x + math.cos(radians) * i
        yi = from_point.y + math.sin(radians) * i
        return Point(xi, yi)

    @staticmethod
    def noise_between(min_val, max_val, is_negative):
        noise = (min_val + random.random() * (max_val - min_val)) / 100
        return 1 + noise if not is_negative else (1 + noise) if random.random() < 0.5 else (1 - noise)

    @staticmethod
    def get_distance_between_points(from_point, to_point):
        x1 = (from_point.x - to_point.x) ** 2
        y1 = (from_point.y - to_point.y) ** 2
        return math.sqrt(x1 + y1)

    @staticmethod
    def get_rotation_between_points(from_point, to_point):
        delta_x = to_point.x - from_point.x
        delta_y = to_point.y - from_point.y
        angle = math.atan2(delta_y, delta_x) * 180 / math.pi
        return angle

class PixelState:
    BLOCKED = 0
    EXPLORED = 1
    UNEXPLORED = 2
    VISITED = 3
