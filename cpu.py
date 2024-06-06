import threading
import time

class CPU:
    def __init__(self, hz, name):
        self.hz = hz
        self.name = name
        self.functions_list = []
        self.is_play = False
        self.stopped = False  # Attribute to track if the CPU is stopped
        self.thread = threading.Thread(target=self.run, name=f"CPU_{name}")
        self.thread.start()

    def add_function(self, func):
        self.functions_list.append(func)

    def play(self):
        self.is_play = True
        self.stopped = False  # Reset the stopped flag

    def stop(self):
        self.is_play = False
        self.stopped = True  # Set the stopped flag

    def run(self):
        while not self.stopped:
            if self.is_play:
                start_time = time.time()
                for func in self.functions_list:
                    func(int(1000 / self.hz))
                elapsed_time = time.time() - start_time
                time.sleep(max(0, (1.0 / self.hz) - elapsed_time))
            else:
                time.sleep(0.01)  # Small sleep to prevent busy waiting when paused/stopped
        print(f"CPU {self.name} has stopped.")