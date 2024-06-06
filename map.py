import numpy as np
from PIL import Image

class Map:
    def __init__(self, path, start_point):
        self.map = self.load_map(path)
        self.start_point = start_point
        self.validate_start_point()

    def load_map(self, path):
        img = Image.open(path).convert('L')  # Convert image to grayscale
        map_array = np.array(img)
        self.height, self.width = map_array.shape
        return (map_array > 128).astype(int)  # Binary map: 1 for free space, 0 for walls

    def is_collide(self, x, y):
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.map[y, x] == 0
        return True  # Treat out-of-bounds as collision

    def validate_start_point(self):
        if not (0 <= self.start_point.x < self.width and 0 <= self.start_point.y < self.height):
            raise ValueError(f"Start point {self.start_point} is out of map bounds.")