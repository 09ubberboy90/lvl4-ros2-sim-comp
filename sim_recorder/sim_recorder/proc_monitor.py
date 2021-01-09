# Standard Library
import sys, os
import psutil
from collections import defaultdict
import rclpy


from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

class ProcMonitor(Node):
    """
    Create the main window and connect the menu bar slots.
    """

    def __init__(self, allowed):
        super().__init__('proccess_monitor')
        self.procs = [(proc.name(), proc)
                        for proc in psutil.process_iter() if proc.name() in allowed]
        self.cpu_dict = defaultdict(list)
        self.ram_dict = defaultdict(list)
        self.timer = self.create_timer(0.5, self.animate)
        self.package_share_directory = get_package_share_directory('sim_recorder')


    def animate(self):
        for idx, (name, p) in enumerate(self.procs):
            try:
                with p.oneshot():
                    cpu_usage = p.cpu_percent()
                    ram_usage = p.memory_info().rss / (1024*1024)
                    self.cpu_dict[(name,p.pid)].append(cpu_usage)
                    self.ram_dict[(name,p.pid)].append(ram_usage)
            except:
                pass
    
    def dump_values(self):
        print("Dumping")
        path = os.path.join(self.package_share_directory, "data")
        with open(os.path.join(path, "cpu_out.csv"), "w") as f:
            print(self.cpu_dict)
            for (name,pid), el in self.cpu_dict.items():
                f.write(f"{name},{','.join(str(v) for v in el)}\n")
        with open(os.path.join(path , "ram_out.csv"), "w") as f:
            for (name,pid), el in self.ram_dict.items():
                f.write(f"{name},{','.join(str(v) for v in el)}\n")





def main(args=None):
    rclpy.init(args=args)


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

    rclpy.spin(monitor)
    monitor.dump_values()
    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
