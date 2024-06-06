import matplotlib.pyplot as plt

class Painter:
    def __init__(self, algo):
        self.algo = algo
        self.figure, self.ax = plt.subplots()
        plt.ion()
        plt.show()

    def paint(self):
        self.ax.clear()
        self.ax.imshow(self.algo.map, cmap='gray')
        drone_point = self.algo.drone.get_point_on_map()
        self.ax.plot(drone_point.x + self.algo.drone_starting_point.x, drone_point.y + self.algo.drone_starting_point.y, 'ro')
        self.ax.set_title('Explored Map')
        plt.pause(0.001)
