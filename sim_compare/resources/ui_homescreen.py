# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'homescreen.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

from resource_monitor import CpuFreqGraph


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1072, 710)
        MainWindow.setMinimumSize(QSize(1072, 710))
        font = QFont()
        font.setPointSize(11)
        MainWindow.setFont(font)
        icon = QIcon()
        icon.addFile(u"fpd_explorer/frontend/res/icon.png", QSize(), QIcon.Normal, QIcon.Off)
        MainWindow.setWindowIcon(icon)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.verticalLayout = QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.process = QComboBox(self.centralwidget)
        self.process.setObjectName(u"process")

        self.verticalLayout.addWidget(self.process)

        self.widget = CpuFreqGraph(self.centralwidget)
        self.widget.setObjectName(u"widget")
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.widget.sizePolicy().hasHeightForWidth())
        self.widget.setSizePolicy(sizePolicy)

        self.verticalLayout.addWidget(self.widget)

        self.proc_info = QPlainTextEdit(self.centralwidget)
        self.proc_info.setObjectName(u"proc_info")
        sizePolicy1 = QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Minimum)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.proc_info.sizePolicy().hasHeightForWidth())
        self.proc_info.setSizePolicy(sizePolicy1)

        self.verticalLayout.addWidget(self.proc_info)

        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        self.process.currentTextChanged.connect(MainWindow.changed_proc)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"Grapher", None))
    # retranslateUi

