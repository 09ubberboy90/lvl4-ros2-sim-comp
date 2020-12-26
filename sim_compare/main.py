# Standard Library
import sys

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

plt.use('Qt5Agg')

if __name__ == "__main__":
    grapher = QtWidgets.QApplication()
    window = ApplicationWindow(grapher)
    window.show()
    sys.exit(grapher.exec_())
