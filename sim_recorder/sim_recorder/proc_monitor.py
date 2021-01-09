# Standard Library
import sys
import time
import psutil
import signal
from collections import defaultdict



class ProcMonitor():
    """
    Create the main window and connect the menu bar slots.
    """

    def __init__(self, allowed):
        self.procs = [(proc.name(), proc)
                        for proc in psutil.process_iter() if proc.name() in allowed]
        self.cpu_dict = defaultdict(list)
        self.ram_dict = defaultdict(list)


    
    def dump_values(self):
        print("Dumping")
        with open("/home/ubb/Documents/PersonalProject/VrController/sim_compare/data/cpu_out.csv", "w") as f:
            print(self.cpu_dict)
            for (name,pid), el in self.cpu_dict.items():
                f.write(f"{name},{','.join(str(v) for v in el)}\n")
        with open("/home/ubb/Documents/PersonalProject/VrController/sim_compare/data/ram_out.csv", "w") as f:
            for (name,pid), el in self.ram_dict.items():
                f.write(f"{name},{','.join(str(v) for v in el)}\n")




def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    monitor.dump_values()
    sys.exit(0)

if __name__ == "__main__":

    signal.signal(signal.SIGINT, signal_handler)

    allowed_gazebo = [
        "fake_joint_driver_node",
        "gzclient",
        "gzserver",
        "mongod",
        "move_group",
        "python3",
        "robot_state_publisher",
        "ros2",
        "rviz2",
        "static_transform_publisher",
    ]
    allowed_webots = [
        "fake_joint_driver_node",
        "mongod",
        "move_group",
        "python3",
        "robot_state_publisher",
        "ros2",
        "rviz2",
        "static_transform_publisher",
        "webots",
        "webots-bin",
        "webots_robotic_",
    ]
    monitor = ProcMonitor(allowed_webots)

    while True: 
        for idx, (name, p) in enumerate(monitor.procs):
            try:
                with p.oneshot():
                    cpu_usage = p.cpu_percent()
                    ram_usage = p.memory_info().rss / (1024*1024)
                    monitor.cpu_dict[(name,p.pid)].append(cpu_usage)
                    monitor.ram_dict[(name,p.pid)].append(ram_usage)
            except:
                print("exception")
        time.sleep(1)