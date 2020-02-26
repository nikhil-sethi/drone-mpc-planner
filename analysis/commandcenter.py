# PyQt5 Video player
#!/usr/bin/env python

from PyQt5.QtCore import QDir, Qt, QUrl,QRect,QTimer
from PyQt5.QtMultimedia import QMediaContent, QMediaPlayer
from PyQt5.QtMultimediaWidgets import QVideoWidget
from PyQt5.QtWidgets import (QApplication, QFileDialog, QHBoxLayout, QLabel,
        QPushButton, QSizePolicy, QSlider, QStyle, QVBoxLayout, QGridLayout, QWidget)
from PyQt5.QtWidgets import QMainWindow,QWidget, QPushButton, QAction,QFrame
from PyQt5.QtGui import QIcon, QPixmap, QPalette

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
        self.setSizePolicy(QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed))
        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.black)
        self.setPalette(p)
        layout = QGridLayout()

        #find all status files in ~/Downloads/pats_status
        source_folder = '~/Downloads/pats_status'
        source_folder = os.path.expanduser(source_folder.strip(" "))
        files = lib.list_txt_files(source_folder)
        n = sum(1 for i in lib.list_txt_files(source_folder)) #python=KUT
        np = n/3
        #for each status file, create a new SingleSystem 
    
        self.imlabels = []
        self.txtlabels = []
        self.files = []
        self.source_folder = source_folder
        i=0
        for f in files:
            self.files.append(f)
            source_txt_file = Path(source_folder,f)
            source_im_file = Path(source_folder,source_txt_file.stem + ".jpg")

            data = ''
            with open (source_txt_file, "r") as status_txt_file:
                data=status_txt_file.read()
            txtlabel = QLabel()
            txtlabel.setText(data)


            pal = QPalette(txtlabel.palette())
            pal.setColor(QPalette.WindowText, Qt.red)
            txtlabel.setPalette(pal)
            
            self.txtlabels.append(txtlabel)
            #.setText("<font color='red'>" + data + "</font>")
        
            imlabel = QLabel()
            pixmap = QPixmap(str(source_im_file))
            imlabel.setPixmap(pixmap)
            imlabel.setPixmap(pixmap.scaled(imlabel.size(),Qt.KeepAspectRatio, Qt.SmoothTransformation))
            imlabel.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Expanding)
            imlabel.setAlignment(Qt.AlignCenter)
            imlabel.setMinimumSize(100, 100)
            self.imlabels.append(imlabel)


            sub_layout = QVBoxLayout()
            sub_layout.setContentsMargins(0, 0, 0, 0)
            sub_layout.addWidget(imlabel)
            sub_layout.addWidget(txtlabel)
            
            
            layout.addLayout(sub_layout,i/np,i % np)
            i = i + 1


        self.refreshButton = QPushButton()
        self.refreshButton.setIcon(self.style().standardIcon(QStyle.SP_BrowserReload))
        self.refreshButton.clicked.connect(self.refresh)
        control_layout = QVBoxLayout()
        control_layout.setContentsMargins(0, 0, 0, 0)
        control_layout.addWidget(self.refreshButton)
        layout.addLayout(control_layout,i/np,i % np)

        layout.setGeometry(QRect(0, 0, 640 , 480))
        
        wid.setLayout(layout)

   

        timer = QTimer(self)
        timer.timeout.connect(self.refresh)
        timer.start(1000)


        self.show()
        
         
    def refresh(self):
        i = 0
        for f in self.files:
            source_txt_file = Path(self.source_folder,f)
            source_im_file = Path(self.source_folder,source_txt_file.stem + ".jpg")

            data = ''
            with open (source_txt_file, "r") as status_txt_file:
                data=status_txt_file.read()
            self.txtlabels[i].setText(data)

            pixmap = QPixmap(str(source_im_file))
            self.imlabels[i].setPixmap(pixmap)
            self.imlabels[i].setPixmap(pixmap.scaled(self.imlabels[i].size(),Qt.KeepAspectRatio, Qt.SmoothTransformation))

            i = i + 1



if __name__ == '__main__':
    app = QApplication(sys.argv)
    cc = CommandCenterWindow()
    cc.show()

    sys.exit(app.exec_())
