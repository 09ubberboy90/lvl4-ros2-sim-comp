# Standard Library
import sys

import psutil
import rclpy

import matplotlib as plt
from PySide2 import QtWidgets, QtCore
from PySide2.QtWidgets import QMainWindow

from .ui_homescreen import Ui_MainWindow

from rclpy.node import Node


class ProcMonitorGui(QMainWindow):
    """
    Create the main window and connect the menu bar slots.
    """

    def __init__(self, app=None, allowed=None):
        super(ProcMonitorGui, self).__init__()
        self._ui = Ui_MainWindow()
        self._ui.setupUi(self)
        if allowed is not None:
            self.procs = [(proc.name(), proc)
                          for proc in psutil.process_iter() if proc.name() in allowed]
        else:
            self.procs = [(proc.name(), proc)
                          for proc in psutil.process_iter()]

        self._ui.process.addItems(sorted(self.procs, key=lambda x: x[0]))
        if allowed is not None:
            self.update_select()
        self._ui.button.clicked.connect(self.change_proc)

    def change_proc(self):
        data = self._ui.process.currentData()
        self._ui.graph.update_proc(data, self._ui.proc_info)

    def update_proc_list(self):
        proc_id = [proc.pid for name, proc in self.procs]
        self.procs = [(proc.name(), proc)
                      for proc in psutil.process_iter() if proc.pid not in proc_id]
        self._ui.process.addItems(sorted(self.procs, key=lambda x: x[0]))

    def update_select(self):
        model = self._ui.process.model()
        for i in range(model.rowCount()):
            model.item(i).setCheckState(QtCore.Qt.Checked)
        self.change_proc()

    def dump_selected(self):
        for proc in self._ui.process.currentData():
            print(proc.name())

    def closeEvent(self, event):
        self._ui.graph.dump_values()
        event.accept()



def main(args=None):
    plt.use('Qt5Agg')
    rclpy.init(args=args)

    grapher = QtWidgets.QApplication()
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

    window = ProcMonitorGui(grapher)
    window.show()
    sys.exit(grapher.exec_())
    rclpy.shutdown()

if __name__ == "__main__":
    main()