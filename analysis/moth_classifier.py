# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/houjebek/code/pats/analysis/moth_classifier.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 597)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.pushButton_moth = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_moth.setGeometry(QtCore.QRect(250, 520, 89, 25))
        self.pushButton_moth.setObjectName("pushButton_moth")
        self.pushButton_noMoth = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_noMoth.setGeometry(QtCore.QRect(380, 520, 89, 25))
        self.pushButton_noMoth.setObjectName("pushButton_noMoth")
        self.frame_video = QtWidgets.QFrame(self.centralwidget)
        self.frame_video.setGeometry(QtCore.QRect(10, 100, 391, 341))
        self.frame_video.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_video.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_video.setObjectName("frame_video")
        self.video_widget = QVideoWidget(self.frame_video)
        self.video_widget.setGeometry(QtCore.QRect(0, 0, 391, 341))
        self.video_widget.setObjectName("video_widget")
        self.frame_plot = QtWidgets.QFrame(self.centralwidget)
        self.frame_plot.setGeometry(QtCore.QRect(400, 100, 391, 341))
        self.frame_plot.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_plot.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_plot.setObjectName("frame_plot")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Pats Moth Classifier"))
        self.pushButton_moth.setText(_translate("MainWindow", "Moth"))
        self.pushButton_noMoth.setText(_translate("MainWindow", "No Moth"))

import os
import sys

#from PyQt5 import QtWidgets, QtMultimedia, uic, QtCore
from PyQt5.QtMultimediaWidgets import QVideoWidget

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

