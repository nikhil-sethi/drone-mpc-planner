# PyQt5 Video player
#!/usr/bin/env python

from PyQt5.QtCore import QDir, Qt, QUrl
from PyQt5.QtMultimedia import QMediaContent, QMediaPlayer
from PyQt5.QtMultimediaWidgets import QVideoWidget
from PyQt5.QtWidgets import (QApplication, QFileDialog, QHBoxLayout, QLabel,
        QPushButton, QSizePolicy, QSlider, QStyle, QVBoxLayout, QGridLayout, QWidget)
from PyQt5.QtWidgets import QMainWindow,QWidget, QPushButton, QAction,QFrame
from PyQt5.QtGui import QIcon, QPixmap

from pathlib import Path
from matplotlib.backends.backend_qt5agg import (FigureCanvas, NavigationToolbar2QT as NavigationToolbar)
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

import processloglib as lib

import random,sys,os


class CommandCenterWindow(QMainWindow):

    def __init__(self, parent=None):
        super(CommandCenterWindow, self).__init__(parent)
        self.setWindowTitle("Pats Command Center") 
        wid = QWidget(self)
        self.setCentralWidget(wid)
        layout = QGridLayout()

        #find all status files in ~/Downloads/pats_status
        source_folder = '~/Downloads/pats_status'
        source_folder = os.path.expanduser(source_folder.strip(" "))
        files = lib.list_txt_files(source_folder)

        #for each status file, create a new SingleSystem 
        controls = []
        i=0
        for f in files:
            source_txt_file = Path(source_folder,f)
            source_im_file = source_txt_file.stem + ".jpg"
            ctrl = controls.append(SingleSystemControl(source_txt_file,source_im_file))
            layout.addLayout(ctrl,0,i)
            i = i + 1
            controls.append(ctrl)

        wid.setLayout(layout)


class SingleSystemControl():

    def __init__(self,parent=None,source_txt_file='',source_im_file=''):
        
        self.im_file = source_im_file
        self.txt_file = source_txt_file

        self.txtlabel = QLabel()
        self.txtlabel.setText(source_txt_file)
        
        self.imlabel = QLabel()
        pixmap = QPixmap(source_im_file)
        self.imlabel.setPixmap(pixmap)
        

        controlLayout = QVBoxLayout()
        controlLayout.setContentsMargins(0, 0, 0, 0)
        controlLayout.addWidget(self.imlabel)
        controlLayout.addWidget(self.txtlabel)



if __name__ == '__main__':
    app = QApplication(sys.argv)
    cc = CommandCenterWindow()
    cc.showMaximized()

    sys.exit(app.exec_())
