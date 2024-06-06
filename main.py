from map import Map
from point import Point
from auto_algo import AutoAlgo

def main():
    start_point = Point(100, 100) 
    real_map = Map('C:/Users/דניאל ריבני/Desktop/first/maps/p15.png', start_point) # change with the path to your map
    auto_algo = AutoAlgo(real_map)
    auto_algo.play()

if __name__ == "__main__":
    main()
