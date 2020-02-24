#!/usr/bin/env python
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5 import QtWidgets, QtMultimedia, uic, QtCore
from PyQt5.QtMultimedia import QMediaContent, QMediaPlayer
import sys


class Ui(QtWidgets.QMainWindow):
    def __init__(self):
        super(Ui, self).__init__()
        uic.loadUi('moth_classifier.ui', self)
        
        file = "/home/houjebek/Bla/pats/monitoring/koppert/4/data/00010/logging/insect10.mp4"
        
        
        self.mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)
        self.mediaPlayer.setVideoOutput(self.video_widget)
        self.mediaPlayer.setMedia(QtMultimedia.QMediaContent(QtCore.QUrl.fromLocalFile(file)))
        self.mediaPlayer.play()


        self.resize(640, 480)
        self.showMaximized()


app = QtWidgets.QApplication(sys.argv)
window = Ui()
sys.exit(app.exec_())



