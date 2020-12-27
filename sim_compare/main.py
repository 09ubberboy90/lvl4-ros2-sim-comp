# Standard Library
import sys

import psutil

import matplotlib as plt
from PySide2 import QtWidgets
from PySide2.QtWidgets import QMainWindow

from resources.ui_homescreen import Ui_MainWindow
# First Party
class ApplicationWindow(QMainWindow):
    """
    Create the main window and connect the menu bar slots.
    """

    def __init__(self, app=None):
        super(ApplicationWindow, self).__init__()
        self._ui = Ui_MainWindow()
        self._ui.setupUi(self)
        self.procs = [(proc.name(), proc) for proc in psutil.process_iter()]
        self._ui.process.addItems(sorted(self.procs, key=lambda x: x[0]))
        self._ui.button.clicked.connect(self.change_proc)

    def change_proc(self):
        data = self._ui.process.currentData()
        self._ui.graph.update_proc(data, self._ui.proc_info)

    def update_proc_list(self):
        proc_id = [proc.pid for name, proc in self.procs]
        self.procs = [(proc.name(), proc) for proc in psutil.process_iter() if proc.pid not in proc_id]
        print(self.procs)
        self._ui.process.addItems(sorted(self.procs, key=lambda x: x[0]))


plt.use('Qt5Agg')

if __name__ == "__main__":
    grapher = QtWidgets.QApplication()
    window = ApplicationWindow(grapher)
    window.show()
    sys.exit(grapher.exec_())
