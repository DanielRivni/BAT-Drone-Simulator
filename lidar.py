import random
from point import Point
from tools import Tools
from world_params import WorldParams

class Lidar:
    def __init__(self, drone, degrees):
        self.drone = drone
        self.degrees = degrees
        self.current_distance = 0

    def get_distance(self, delta_time):
        from_point = self.drone.get_point_on_map()
        rotation = self.drone.rotation + self.degrees
        distance_in_cm = 1
        while distance_in_cm <= WorldParams.LIDAR_LIMIT:
            p = Tools.get_point_by_distance(from_point, rotation, distance_in_cm)
            if self.drone.real_map.is_collide(int(p.x), int(p.y)):
                break
            distance_in_cm += 1
        return distance_in_cm

    def get_simulation_distance(self, delta_time):
        if random.random() <= 0.05:
            distance_in_cm = 0
        else:
            distance_in_cm = self.get_distance(delta_time)
            distance_in_cm += random.randint(-WorldParams.LIDAR_NOISE, WorldParams.LIDAR_NOISE)
            distance_in_cm = min(max(distance_in_cm, 0), WorldParams.LIDAR_LIMIT)
        self.current_distance = distance_in_cm
        return distance_in_cm
