# Standard Library
import os
import signal
import sys
from collections import defaultdict

import psutil
import rclpy
from rclpy.node import Node


class ProcMonitor(Node):
    """
    Create the main window and connect the menu bar slots.
    """

    def __init__(self, allowed, idx, sim_name):
        super().__init__('proccess_monitor')
        self.cpu_dict = defaultdict(list)
        self.ram_dict = defaultdict(list)
        self.allowed = allowed
        self.procs = {}
        print("start")
        self.counter = 0
        self.update_missing()
        self.timer = self.create_timer(0.1, self.animate)
        self.idx = idx
        self.sim_name = sim_name
        
    def update_missing(self):
        self.missing = [el for el in self.allowed if el not in self.procs.keys()]
        if self.counter < 10:
            print(self.missing)
            self.counter += 1 
        for proc in psutil.process_iter():
            p = proc.name()
            if p in self.missing:
                new_p = p
                counter = 0
                while new_p in self.procs.keys():
                    counter += 1
                    new_p = new_p + "_" + str(counter)
                if counter != 0:
                    p = p+"_"+str(counter)
                self.procs[p] = proc 
                proc.cpu_percent() # discard value
                proc.cpu_percent() # discard value


    def animate(self):
        if self.missing:
            self.update_missing()
        for name, p in self.procs.items():
            try:
                with p.oneshot():
                    cpu_usage = p.cpu_percent()
                    if cpu_usage > 2400:
                        print("Error : High Cpu Usage")
                        cpu_usage = 2400
                    if len(self.cpu_dict[(name, p.pid)]) == 0 and cpu_usage > 300:
                        cpu_usage = 300
                    ram_usage = p.memory_info().rss / (1024*1024)
                    self.cpu_dict[(name, p.pid)].append(cpu_usage)
                    self.ram_dict[(name, p.pid)].append(ram_usage)
            except:
                pass

    def dump_values(self):
        path = "/home/ubb/Documents/PersonalProject/VrController/sim_recorder/data/"
        with open(path + f"{self.sim_name}/cpu/cpu_{self.idx}.csv", "w") as f:
            for (name, pid), el in self.cpu_dict.items():
                f.write(f"{name},{','.join(str(v) for v in el)}\n")
        with open(path + f"{self.sim_name}/ram/ram_{self.idx}.csv", "w") as f:
            for (name, pid), el in self.ram_dict.items():
                f.write(f"{name},{','.join(str(v) for v in el)}\n")
        sys.exit(0)


allowed_gazebo = [
    "throw_moveit",
    "fake_joint_driver_node",
    "gzclient",
    "gzserver",
    "mongod",
    "move_group",
    "moveit_collision",
    "python3",
    "robot_state_publisher",
    "ros2",
    "rviz2",
    "static_transform_publisher",
    "run_recording",
    "moveit_controller",
]

allowed_webots = [
    "throw_moveit",
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
    "moveit_collision",
    "run_recording",
    "moveit_controller",
]
allowed_ignition = [
    "move_group",
    "parameter_bridge",
    "python3",
    "ros2",
    "ruby",
    "rviz2",
    "run_recording"
]


def run(simulator="webots", idx=0, args=None):

    rclpy.init(args=args)
    if "webots" == simulator:
        allowed = allowed_webots[1:]
    elif "gazebo" == simulator:
        allowed = allowed_gazebo[1:]
    elif "webots_throw" == simulator:
        allowed = allowed_webots[:-1]
    elif "gazebo_throw" == simulator:
        allowed = allowed_gazebo[:-1]
    else:
        allowed = allowed_ignition

    monitor = ProcMonitor(allowed, idx, simulator)
    signal.signal(signal.SIGINT, lambda sig, frame: monitor.dump_values())
    signal.signal(signal.SIGTERM, lambda sig, frame: monitor.dump_values())

    rclpy.spin(monitor)
    rclpy.shutdown()


def main(args=None):
    run(args=args)


if __name__ == '__main__':
    main()
