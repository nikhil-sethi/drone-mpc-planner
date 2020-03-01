#!/usr/bin/env python
from PyQt5.QtCore import QDir, Qt, QUrl,QRect,QTimer
from PyQt5.QtMultimedia import QMediaContent, QMediaPlayer
from PyQt5.QtMultimediaWidgets import QVideoWidget
from PyQt5.QtWidgets import (QMainWindow,QWidget, QPushButton, QCheckBox, QAction,QFrame,
    QApplication, QFileDialog, QHBoxLayout, QLabel,QPushButton, QSizePolicy, QSlider, QStyle,
    QVBoxLayout, QGridLayout, QWidget)
from PyQt5.QtGui import QIcon, QPixmap, QPalette,QColor

from pathlib import Path
from datetime import datetime
import random,sys,os,re

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
        files = list_txt_files(source_folder)
        n = sum(1 for i in list_txt_files(source_folder)) #python=KUT
        np = n/3
        
        self.sys_widgets = []
        self.files = []
        self.source_folder = source_folder
        
        for f in files:
            self.files.append(f)
        self.files.sort(key=natural_keys)

        i=0
        for f in self.files:
            s = SystemWidget(source_folder,f)
            self.sys_widgets.append(s)
            layout.addWidget(s,i/np,i % np)
            i = i + 1

        layout.setGeometry(QRect(0, 0, 640 , 480))
        wid.setLayout(layout)

        timer = QTimer(self)
        timer.timeout.connect(self.refresh)
        timer.start(1000)

        self.refresh()
        self.show() 

    def refresh(self):
        for sys in self.sys_widgets:
            sys.refresh()

class SystemWidget(QWidget):
    def __init__(self, source_folder,status_fn,parent=None):
        QWidget.__init__(self, parent=parent)

        self.status_fn = status_fn
        self.source_folder = source_folder

        self.chk_enable = QCheckBox()
        self.chk_enable.setStyleSheet("background-color:rgb(128,0,0)")
        self.chk_enable.setChecked(True)
        btn_restart = QPushButton()
        btn_restart.setIcon(self.style().standardIcon(QStyle.SP_BrowserReload))
        btn_restart.setMaximumSize(30, 30)
        btn_restart.setStyleSheet("background-color:rgb(128,0,0)")
        btn_restart.clicked.connect(self.restart)

        btn_takeoff = QPushButton()
        btn_takeoff.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        btn_takeoff.setMaximumSize(30, 30)
        btn_takeoff.setStyleSheet("background-color:rgb(128,0,0)")
        btn_takeoff.clicked.connect(self.takeoff)

        control_layout = QHBoxLayout()
        control_layout.setContentsMargins(0, 0, 0, 0)
        control_layout.addWidget(self.chk_enable)
        control_layout.addWidget(btn_restart)
        control_layout.addWidget(btn_takeoff)
        
        self.txt_label = QLabel()

        self.im_label = QLabel()
        self.im_label.mouseReleaseEvent = self.takeoshow_im_big
        self.im_label.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Expanding)
        self.im_label.setAlignment(Qt.AlignCenter)
        self.im_label.setMinimumSize(100, 100)

        sub_layout = QVBoxLayout(self)
        sub_layout.setContentsMargins(0, 0, 0, 0)
        sub_layout.addWidget(self.im_label)
        sub_layout.addWidget(self.txt_label)
        sub_layout.addLayout(control_layout)
    
    def restart(self):
        print('command restart')
    def takeoff(self):
        print('command take off')
    def takeoshow_im_big(self,event):
        print('command show im big')

    def refresh(self):
        if self.chk_enable.isChecked():
            # try:
            txt,system_color = self.get_lbl_txt()
            self.txt_label.setText(txt)
            pal = QPalette(self.txt_label.palette())
            pal.setColor(QPalette.WindowText, system_color)
            self.txt_label.setPalette(pal)   
            # except:
            #     print('error')
            source_im_file = Path(self.source_folder,self.status_fn[:-4] + ".jpg")
            pixmap = QPixmap(str(source_im_file))
            self.im_label.setPixmap(pixmap)
            self.im_label.setPixmap(pixmap.scaled(self.im_label.size(),Qt.KeepAspectRatio, Qt.SmoothTransformation))
        else:
            pal = QPalette(self.txt_label.palette())
            pal.setColor(QPalette.WindowText, QColor(60,0,0))
            self.txt_label.setPalette(pal)  

    def get_lbl_txt(self):
        source_status_txt_file = Path(self.source_folder,self.status_fn)
        source_system_txt_file = Path(self.source_folder,str(self.status_fn).replace('status','system'))

        res_txt = ''
        system_has_problem = QColor(100,100,100)

        sysinfo_txt = ''
        with open (source_system_txt_file, "r") as sysinf_txt_file:
            sysinfo_txt=sysinf_txt_file.readlines()
        hostid = sysinfo_txt[0].split(':')[1].strip().replace('pats-proto','')
        droneid = sysinfo_txt[1].split(':')[1].strip()
        sha = sysinfo_txt[2].split(':')[1].strip()[-6:]
        
        hd = ''
        for line in sysinfo_txt:
            if line.find("nvme0n1p2") != -1:
                hd = line.split()[4]
        
        res_txt = "Pats-" + hostid + '->' + droneid + ' @' + sha + '\n'
        res_txt = res_txt + 'hd: ' + hd + '\n'
        hd = float(hd.replace('%',''))
        if hd > 95:
            system_has_problem = QColor(200,0,0)
        
        status_txt = []
        with open (source_status_txt_file, "r") as status_txt_file:
            status_txt=status_txt_file.readlines()

        date_time = datetime.strptime(status_txt[0].strip(), '%Y/%m/%d %H:%M:%S') #e.g. 2020/02/29 23:45:46
        time_since_update = datetime.now() - date_time
        
        if time_since_update.total_seconds() > 300:
            system_has_problem = QColor(200,0,0)
        
        res_txt += status_txt[1].strip() + '. upd: ' + str(int(time_since_update.total_seconds())) + 's\n'
        res_txt += status_txt[2]
        
        return res_txt.strip(),system_has_problem        

def atoi(text):
    return int(text) if text.isdigit() else text
    
def natural_keys(text):
    '''
    alist.sort(key=natural_keys) sorts in human order
    http://nedbatchelder.com/blog/200712/human_sorting.html
    (See Toothy's implementation in the comments)
    '''
    return [ atoi(c) for c in re.split(r'(\d+)', text) ]  

from os import listdir
def list_txt_files(directory):
    return (f for f in listdir(directory) if f.endswith('status.txt'))

if __name__ == '__main__':
    app = QApplication(sys.argv)
    cc = CommandCenterWindow()
    cc.show()

    sys.exit(app.exec_())
