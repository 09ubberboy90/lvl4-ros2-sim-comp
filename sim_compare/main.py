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
        self.procs = [proc.name() for proc in psutil.process_iter()]
        self._ui.process.addItems(sorted(self.procs))

    def changed_proc(self, str):
        self._ui.process.changed_proc(str)
        #self._ui.proc_info.

plt.use('Qt5Agg')

if __name__ == "__main__":
    grapher = QtWidgets.QApplication()
    window = ApplicationWindow(grapher)
    window.show()
    sys.exit(grapher.exec_())
