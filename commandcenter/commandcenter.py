#!/usr/bin/env python3
from PyQt5.QtCore import QDir, Qt, QUrl,QRect,QTimer
from PyQt5.QtMultimedia import QMediaContent, QMediaPlayer
from PyQt5.QtMultimediaWidgets import QVideoWidget
from PyQt5.QtWidgets import (QMainWindow,QDialog,QWidget, QPushButton, QCheckBox, QComboBox, QAction,QFrame,
    QApplication, QFileDialog, QHBoxLayout, QLabel,QPushButton, QSizePolicy, QSlider, QStyle, QListWidget,QListWidgetItem,
    QVBoxLayout, QGridLayout, QWidget,QMessageBox,QPlainTextEdit,QMenu,QTabWidget,QToolBar)
from PyQt5.QtGui import QIcon,QPixmap, QPalette,QColor,QKeyEvent,QFont
import shutil

from pathlib import Path
from datetime import datetime
import random,sys,os,re,subprocess,math
class CommandCenterWindow(QMainWindow):

    def __init__(self,screen_size,parent=None):
        super(CommandCenterWindow, self).__init__(parent)
        self.setWindowTitle("Pats Command Center")
        self.setWindowIcon(QIcon("./icon.png"))
        self.wid = QWidget(self)
        self.setCentralWidget(self.wid)
        self.setSizePolicy(QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed))
        self.setAutoFillBackground(True)

        select_systems_Action = QAction(self.style().standardIcon(QStyle.SP_FileDialogDetailedView), 'Select systems', self)
        select_systems_Action.setShortcut('Ctrl+K')
        select_systems_Action.triggered.connect(self.select_systems)

        request_updates_Action = QAction(self.style().standardIcon(QStyle.SP_ArrowDown), 'More status more NOW!', self)
        request_updates_Action.triggered.connect(self.request_updates)
        reboot_rtc_Action = QAction(self.style().standardIcon(QStyle.SP_DialogResetButton), 'Reboot all systems', self)
        reboot_rtc_Action.triggered.connect(self.reboot_systems)
        git_update_Action = QAction(self.style().standardIcon(QStyle.SP_FileDialogToParent), 'Update all systems', self)
        git_update_Action.triggered.connect(self.git_update_systems)
        restart_Action = QAction(self.style().standardIcon(QStyle.SP_BrowserReload), 'Restart all systems', self)
        restart_Action.triggered.connect(self.restart_systems)
        fly_flightplan_Action = QAction(self.style().standardIcon(QStyle.SP_MediaPlay), 'Trigger waypoint flight', self)
        fly_flightplan_Action.triggered.connect(self.fly_flightplan_flight)
        fly_replay_moth_Action = QAction(self.style().standardIcon(QStyle.SP_MediaPlay), 'Trigger replay moths', self)
        fly_replay_moth_Action.triggered.connect(self.fly_replay_moth)

        self.toolbar = self.addToolBar('Pats Menu')
        self.toolbar.setMovable(False)
        self.toolbar.setFloatable(False)
        self.toolbar.addAction(select_systems_Action)
        self.toolbar.addSeparator()
        self.toolbar.addAction(request_updates_Action)
        self.toolbar.addSeparator()
        self.toolbar.addAction(restart_Action)
        self.toolbar.addAction(git_update_Action)
        self.toolbar.addAction(reboot_rtc_Action)
        self.toolbar.addSeparator()
        self.toolbar.addAction(fly_flightplan_Action)
        self.toolbar.addAction(fly_replay_moth_Action)
        self.toolbar.addSeparator()
        self.combo_mode = QComboBox()
        self.combo_mode.addItem("Monitoring")
        self.combo_mode.addItem("Waypoint")
        self.combo_mode.addItem("Hunt")
        self.combo_mode.setCurrentIndex(-1)
        self.combo_mode.currentIndexChanged.connect(self.mode_change)
        self.combo_mode.setToolTip('Select mode for all systems')
        self.toolbar.addWidget(self.combo_mode)
        self.combo_sub_mode = QComboBox()
        self.combo_sub_mode.addItem("Default")
        self.combo_sub_mode.addItem("Koppert")
        self.combo_sub_mode.addItem("Holstein")
        self.combo_sub_mode.addItem("WUR")
        self.combo_sub_mode.addItem("Vogel")
        self.combo_sub_mode.addItem("Erp")
        self.combo_sub_mode.setCurrentIndex(-1)
        self.combo_sub_mode.currentIndexChanged.connect(self.sub_mode_change)
        self.combo_sub_mode.setToolTip('Select sub mode for all systems')
        self.toolbar.addWidget(self.combo_sub_mode)

        source_folder = '~/Downloads/pats_status'
        source_folder = os.path.expanduser(source_folder.strip(" "))
        # shutil.rmtree(source_folder)
        self.source_folder = source_folder
        self.download(True)


        self.layout = QGridLayout()
        self.populate_systems()

        self.setMaximumSize(screen_size.width(), screen_size.height())


        timer = QTimer(self)
        timer.timeout.connect(self.refresh)
        timer.start(1000)


        self.refresh()
        self.show()
        self.refresh()

    def populate_systems(self):

        for i in reversed(range(self.layout.count())):
            self.layout.itemAt(i).widget().setParent(None)

        systems = next(os.walk(self.source_folder))[1]
        systems.sort(key=natural_keys)
        self.system_folders = systems

        n = 0
        for f in self.system_folders:
            if os.path.exists(Path(self.source_folder,f,'.select')):
                n = n+1

        if n>1:
            div = math.floor(math.sqrt(n))
            np = int(math.ceil(n/div))
        else:
            div = 1
            np=1

        self.sys_widgets = []
        i=0
        for f in self.system_folders:
            if os.path.exists(Path(self.source_folder,f,'.select')):
                s = SystemWidget(self.source_folder,f)
                self.sys_widgets.append(s)
                self.layout.addWidget(s,int(i/np),i % np)
                i = i + 1

        self.update_combos()
        self.wid.setLayout(self.layout)

    def select_systems(self):
        SelectSystemsDialog(self,self.source_folder,self.system_folders)
        self.populate_systems()
        self.update()
    def request_updates(self):
        for sys in self.sys_widgets:
            sys.request_update()
    def reboot_systems(self):
        buttonReply = QMessageBox.question(self, 'Maar, weet je dat eigenlijk wel zekerdepeter?!', "Reboot, really?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if buttonReply == QMessageBox.Yes:
            for sys in self.sys_widgets:
                sys.reboot_dont_ask()
    def git_update_systems(self):
        buttonReply = QMessageBox.question(self, 'Maar, weet je dat eigenlijk wel zekerdepeter?!', "Update, really?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if buttonReply == QMessageBox.Yes:
            for sys in self.sys_widgets:
                sys.git_update()
    def restart_systems(self):
        for sys in self.sys_widgets:
            sys.restart()
    def fly_flightplan_flight(self):
        for sys in self.sys_widgets:
            sys.flightplan_takeoff()
    def fly_replay_moth(self):
        for sys in self.sys_widgets:
            sys.insect_replay_takeoff()
    def mode_change(self):
        if self.combo_mode.currentIndex() >= 0:
            for sys in self.sys_widgets:
                sys.combo_mode.setCurrentIndex(self.combo_mode.currentIndex())
            self.combo_mode.setCurrentIndex(-1)
    def sub_mode_change(self):
        if self.combo_sub_mode.currentIndex() >= 0:
            for sys in self.sys_widgets:
                sys.combo_sub_mode.setCurrentIndex(self.combo_sub_mode.currentIndex())
            self.combo_sub_mode.setCurrentIndex(-1)

    def download(self,wait=False):
        rsync_src='dash:/home/pats/status/'
        subprocess.call(['mkdir -p ' + self.source_folder ], shell=True)
        cmd = ['rsync -zva --timeout=3 --exclude \'*.jpg.*\' --exclude \'*.xml.*\' --exclude \'*.txt.*\' ' + rsync_src + ' '+ self.source_folder]
        if wait:
            subprocess.call(cmd, shell=True,stdout=subprocess.PIPE)
        else:
            subprocess.Popen(cmd, shell=True,stdout=subprocess.PIPE)

    def refresh(self):
        theme = subprocess.check_output(["/usr/bin/gsettings get org.gnome.desktop.interface gtk-theme"], shell=True) #need to use full path /usr/bin because conda screws things up
        darkmode = str(theme).find('dark') != -1
        if darkmode:
            self.combo_mode.setStyleSheet("background-color:rgb(128,0,0)")
            self.combo_sub_mode.setStyleSheet("background-color:rgb(128,0,0)")
        else:
            self.combo_mode.setStyleSheet("background-color:rgb(160,128,128)")
            self.combo_sub_mode.setStyleSheet("background-color:rgb(160,128,128)")

        self.download()
        for sys in self.sys_widgets:
            sys.refresh()

    def update_combos(self):
        self.download()
        for sys in self.sys_widgets:
            sys.update_combos()

class SystemWidget(QWidget):
    def __init__(self, source_folder,system_folder,parent=None):
        QWidget.__init__(self, parent=parent)
        self.dark_mode = True
        self.refresh_darkmode = True
        self.max_cam_roll = 0.5

        self.system_folder = system_folder
        self.source_folder = source_folder

        self.combo_mode = QComboBox()
        self.combo_mode.addItem("Monitoring")
        self.combo_mode.addItem("Waypoint")
        self.combo_mode.addItem("Hunt")
        self.combo_mode.currentIndexChanged.connect(self.mode_change)
        self.combo_sub_mode = QComboBox()
        self.combo_sub_mode.addItem("Default")
        self.combo_sub_mode.addItem("Koppert")
        self.combo_sub_mode.addItem("Holstein")
        self.combo_sub_mode.addItem("WUR")
        self.combo_sub_mode.addItem("TomatoWorld")
        self.combo_sub_mode.addItem("Vogel")
        self.combo_sub_mode.addItem("Erp")
        self.combo_sub_mode.currentIndexChanged.connect(self.sub_mode_change)

        self.setContextMenuPolicy(Qt.ActionsContextMenu)
        calib_IMU_Action = QAction("Calibrate IMU", self)
        calib_IMU_Action.setIcon(self.style().standardIcon(QStyle.SP_DesktopIcon))
        calib_IMU_Action.triggered.connect(self.calib_imu)
        self.addAction(calib_IMU_Action)

        self.setContextMenuPolicy(Qt.ActionsContextMenu)
        calib_thrust_Action = QAction("Calibrate Thrust", self)
        calib_thrust_Action.setIcon(self.style().standardIcon(QStyle.SP_DesktopIcon))
        calib_thrust_Action.triggered.connect(self.flightplan_calib_thrust_takeoff)
        self.addAction(calib_thrust_Action)

        blink_Action = QAction("Blink", self)
        blink_Action.setIcon(self.style().standardIcon(QStyle.SP_FileDialogContentsView))
        blink_Action.triggered.connect(self.blink)
        self.addAction(blink_Action)

        self.setContextMenuPolicy(Qt.ActionsContextMenu)
        clear_calib_Action = QAction("Clear thrust & blink", self)
        clear_calib_Action.setIcon(self.style().standardIcon(QStyle.SP_DialogCancelButton))
        clear_calib_Action.triggered.connect(self.clear_calib)
        self.addAction(clear_calib_Action)

        reboot_rtc_Action = QAction("Reboot", self)
        reboot_rtc_Action.setIcon(self.style().standardIcon(QStyle.SP_DialogResetButton))
        reboot_rtc_Action.triggered.connect(self.reboot)
        self.addAction(reboot_rtc_Action)

        git_update_Action = QAction("Git update", self)
        git_update_Action.setIcon(self.style().standardIcon(QStyle.SP_FileDialogToParent))
        git_update_Action.triggered.connect(self.git_update)
        self.addAction(git_update_Action)

        shakeAction = QAction("Shake", self)
        shakeAction.setIcon(self.style().standardIcon(QStyle.SP_DriveNetIcon))
        shakeAction.triggered.connect(self.shake)
        self.addAction(shakeAction)

        beep_Action = QAction("Beep", self)
        beep_Action.setIcon(self.style().standardIcon(QStyle.SP_MessageBoxWarning))
        beep_Action.triggered.connect(self.beep)
        self.addAction(beep_Action)

        btn_restart = QPushButton()
        btn_restart.setToolTip('Restart pats process')
        btn_restart.setIcon(self.style().standardIcon(QStyle.SP_BrowserReload))
        btn_restart.setMaximumSize(30, 30)
        btn_restart.clicked.connect(self.restart)
        self.btn_restart = btn_restart

        btn_request_update = QPushButton()
        btn_request_update.setToolTip('Request update NOW')
        btn_request_update.setIcon(self.style().standardIcon(QStyle.SP_ArrowDown))
        btn_request_update.setMaximumSize(30, 30)
        btn_request_update.clicked.connect(self.request_update)
        self.btn_request_update = btn_request_update

        btn_download_current_log = QPushButton()
        btn_download_current_log.setToolTip('Restart & download the current log')
        btn_download_current_log.setIcon(self.style().standardIcon(QStyle.SP_DriveFDIcon))
        btn_download_current_log.setMaximumSize(30, 30)
        btn_download_current_log.clicked.connect(self.download_current_log)
        self.btn_download_current_log = btn_download_current_log

        btn_flightplan_takeoff = QPushButton()
        btn_flightplan_takeoff.setToolTip('Trigger flightplan mission')
        btn_flightplan_takeoff.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        btn_flightplan_takeoff.setMaximumSize(30, 30)
        btn_flightplan_takeoff.clicked.connect(self.flightplan_takeoff)
        self.btn_flightplan_takeoff = btn_flightplan_takeoff

        btn_insect_replay_takeoff = QPushButton()
        btn_insect_replay_takeoff.setToolTip('Trigger insect replay')
        btn_insect_replay_takeoff.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        btn_insect_replay_takeoff.setMaximumSize(30, 30)
        btn_insect_replay_takeoff.clicked.connect(self.insect_replay_takeoff)
        self.btn_insect_replay_takeoff = btn_insect_replay_takeoff

        control_layout = QHBoxLayout()
        control_layout.setContentsMargins(0, 0, 0, 0)
        control_layout.addWidget(self.combo_mode)
        control_layout.addWidget(self.combo_sub_mode)
        control_layout.addWidget(btn_request_update)
        control_layout.addWidget(btn_restart)
        control_layout.addWidget(btn_download_current_log)
        control_layout.addWidget(btn_flightplan_takeoff)
        control_layout.addWidget(btn_insect_replay_takeoff)

        self.txt_label = QLabel()
        self.txt_label.setFont(QFont('Arial', 8))

        self.im_label = QLabel()
        self.im_label.mouseReleaseEvent = self.takeoshow_im_big
        self.im_label.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Expanding)
        self.im_label.setAlignment(Qt.AlignCenter)
        self.im_label.setMinimumSize(10, 10)
        self.im_label.setToolTip('Click to enlarge!')
        self.setMinimumSize(10, 10)

        sub_layout = QVBoxLayout(self)
        sub_layout.setContentsMargins(0, 0, 0, 0)
        sub_layout.addWidget(self.im_label)
        sub_layout.addWidget(self.txt_label)
        sub_layout.addLayout(control_layout)

    def mode_change(self):
        if (self.updating_combos):
            return
        self.pats_xml_path = Path(self.source_folder,self.system_folder,'pats_deploy.xml')
        with open (self.pats_xml_path, "r") as path_xml:
            xml_lines = path_xml.readlines()
            new_xml_lines = ''
            for line in xml_lines:
                if line.find('\"op_mode\"') != -1:
                    if self.combo_mode.currentText() == "Hunt":
                        line = '    <Member Name=\"op_mode\">op_mode_hunt</Member>\n'
                    elif self.combo_mode.currentText() == "Waypoint":
                        line = '    <Member Name=\"op_mode\">op_mode_waypoint</Member>\n'
                    elif self.combo_mode.currentText() == "Monitoring":
                        line = '    <Member Name=\"op_mode\">op_mode_monitoring</Member>\n'
                elif line.find('\"live_image_frq\"') != -1:
                    if self.combo_mode.currentText() == "Hunt" or self.combo_mode.currentText() == "Waypoint":
                        line = '    <Member Name=\"live_image_frq\">1</Member>\n'
                    else:
                        line = '    <Member Name=\"live_image_frq\">30</Member>\n'
                elif line.find('\"darkness_threshold\"') != -1:
                    if self.combo_mode.currentText() == "Monitoring":
                        if self.combo_sub_mode.currentText() == 'Koppert':
                            line = '    <Member Name=\"darkness_threshold\">9500</Member>\n'
                        elif self.combo_sub_mode.currentText() == 'Vogel':
                            line = '    <Member Name=\"darkness_threshold\">300</Member>\n'
                        else:
                            line = '    <Member Name=\"darkness_threshold\">5000</Member>\n'
                    elif self.combo_mode.currentText() == "Hunt":
                        if self.combo_sub_mode.currentText() == 'Koppert':
                            line = '    <Member Name=\"darkness_threshold\">9500</Member>\n'
                        else:
                            line = '    <Member Name=\"darkness_threshold\">0</Member>\n'
                    else:
                           line = '    <Member Name=\"darkness_threshold\">0</Member>\n'
                elif line.find('\"max_brightness\"') != -1:
                    if self.combo_mode.currentText() == "Monitoring":
                        if self.combo_sub_mode.currentText() == 'Koppert':
                            line = '    <Member Name=\"max_brightness\">50</Member>\n'
                        elif self.combo_sub_mode.currentText() == 'Holstein':
                            line = '    <Member Name=\"max_brightness\">100</Member>\n'
                        else:
                            line = '    <Member Name=\"max_brightness\">128</Member>\n'
                    elif self.combo_mode.currentText() == "Hunt":
                        if self.combo_sub_mode.currentText() == 'Koppert':
                            line = '    <Member Name=\"max_brightness\">50</Member>\n'
                        else:
                            line = '    <Member Name=\"max_brightness\">255</Member>\n'
                    else:
                           line = '    <Member Name=\"max_brightness\">255</Member>\n'
                elif line.find('\"close_after_n_images\"') != -1:
                    if self.combo_mode.currentText() == "Hunt":
                        line = '    <Member Name=\"close_after_n_images\">10800</Member>\n'
                    else:
                        line = '    <Member Name=\"close_after_n_images\">324000</Member>\n'
                elif line.find('\"max_cam_roll\"') != -1:
                    self.max_cam_roll = 0.5
                    if self.combo_mode.currentText() == "Monitoring":
                        if self.combo_sub_mode.currentText() == 'TomatoWorld':
                            self.max_cam_roll = 360
                        else:
                            self.max_cam_roll = 5
                    else:
                        self.max_cam_roll = 0.5
                    line = '    <Member Name=\"max_cam_roll\">' + str(self.max_cam_roll) + '</Member>\n'
                elif line.find('\"watchdog\"') != -1:
                    line = '    <Member Name="watchdog">true</Member>\n'
                elif line.find('\"sub_mode\"') != -1:
                    line = '    <Member Name="sub_mode">' + self.combo_sub_mode.currentText() + '</Member>\n'

                new_xml_lines = new_xml_lines + line

        pats_xml_tmp_file = open("pats.tmp", "w")
        pats_xml_tmp_file.write(new_xml_lines)
        pats_xml_tmp_file.close()
        subprocess.Popen(['./change_settings_system.sh', 'pats'+self.host_id ])

        xml_file = open(self.pats_xml_path, "w")
        xml_file.write(new_xml_lines)
        xml_file.close()
    def sub_mode_change(self):
        self.mode_change()

    def calib_imu(self):
        subprocess.Popen(['./calib_imu_system.sh', 'pats'+self.host_id])
    def beep(self):
        subprocess.Popen(['./beep_system.sh', 'pats'+self.host_id])
    def clear_calib(self):
        subprocess.Popen(['./clear_calib_system.sh', 'pats'+self.host_id])
    def blink(self):
        subprocess.Popen(['./blink_system.sh', 'pats'+self.host_id])
    def shake(self):
        subprocess.Popen(['./shake_system.sh', 'pats'+self.host_id])

    def restart(self):
        subprocess.Popen(['./restart_system.sh', 'pats'+self.host_id])
    def request_update(self):
        subprocess.Popen(['./req_update_system.sh', 'pats'+self.host_id])
    def download_current_log(self):
        subprocess.Popen(['./download_log_system.sh', 'pats'+self.host_id])
    def git_update(self):
        subprocess.Popen(['./update_system.sh', 'pats'+self.host_id])
        self.updating_combos = True
        self.combo_mode.setCurrentIndex(0)
        self.combo_sub_mode.setCurrentIndex(0)
        self.updating_combos = False
    def reboot_dont_ask(self):
        subprocess.Popen(['./reboot_system.sh', 'pats'+self.host_id])
    def reboot(self):
        buttonReply = QMessageBox.question(self, 'Maar, weet je dat eigenlijk wel zekerdepeter?!', "Reboot, really?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if buttonReply == QMessageBox.Yes:
            subprocess.Popen(['./reboot_system.sh', 'pats'+self.host_id])
    def flightplan_takeoff(self):
        self.pats_xml_path = Path(self.source_folder,self.system_folder,'pats_deploy.xml')
        with open (self.pats_xml_path, "r") as path_xml:
            xml_lines = path_xml.readlines()
            for line in xml_lines:
                if line.find('\"flightplan\"') != -1:
                    self.flightplan_xml_path = Path(self.source_folder,self.system_folder,line.split('\">../xml/')[1].split('<')[0])
                    subprocess.Popen(['./demo_system.sh', 'pats'+self.host_id, self.flightplan_xml_path])
    def flightplan_calib_thrust_takeoff(self):
        self.pats_xml_path = Path(self.source_folder,self.system_folder,'pats_deploy.xml')
        with open (self.pats_xml_path, "r") as path_xml:
            xml_lines = path_xml.readlines()
            for line in xml_lines:
                if line.find('\"flightplan_calib_thrust\"') != -1:
                    self.flightplan_xml_path = Path(self.source_folder,self.system_folder,line.split('\">../xml/')[1].split('<')[0])
                    subprocess.Popen(['./demo_system.sh', 'pats'+self.host_id, self.flightplan_xml_path])
    def insect_replay_takeoff(self):
        subprocess.Popen(['./insect_replay_system.sh', 'pats'+self.host_id])
    def takeoshow_im_big(self,event):
        ImDialog(self,self.source_folder,self.system_folder,self.host_id,self.dark_mode)

    updating_combos = False
    def update_combos(self):
        self.updating_combos = True
        self.pats_xml_path = Path(self.source_folder,self.system_folder,'pats_deploy.xml')
        with open (self.pats_xml_path, "r") as path_xml:
            xml_lines = path_xml.readlines()
            new_xml_lines = ''
            for line in xml_lines:
                if line.find('op_mode_hunt') != -1:
                    self.combo_mode.setCurrentIndex(2)
                elif line.find('op_mode_waypoint') != -1:
                    self.combo_mode.setCurrentIndex(1)
                elif line.find('op_mode_monitoring') != -1:
                    self.combo_mode.setCurrentIndex(0)

                if line.find('sub_mode') != -1:
                    sub_mode_str=line.split('>')[1].split('<')[0]
                    self.combo_sub_mode.setCurrentText(sub_mode_str)

        self.updating_combos = False

    def refresh(self):
        self.check_theme()
        self.pats_xml_path = Path(self.source_folder,self.system_folder,'pats_deploy.xml')
        with open (self.pats_xml_path, "r") as path_xml:
            xml_lines = path_xml.readlines()
            new_xml_lines = ''
            for line in xml_lines:
                if line.find('op_mode_hunt') != -1 and self.combo_mode.currentText() != "Hunt":
                    self.combo_mode.setStyleSheet("background-color:rgb(40,40,40)")
                elif line.find('op_mode_waypoint') != -1 and self.combo_mode.currentText() != "Waypoint":
                    self.combo_mode.setStyleSheet("background-color:rgb(40,40,40)")
                elif line.find('op_mode_monitoring') != -1 and self.combo_mode.currentText() != "Monitoring":
                    self.combo_mode.setStyleSheet("background-color:rgb(40,40,40)")
                elif line.find('op_mode_') != -1:
                    if self.dark_mode:
                        self.combo_mode.setStyleSheet("background-color:rgb(128,0,0)")
                    else:
                        self.combo_mode.setStyleSheet("background-color:rgb(160,128,128)")
                if line.find('sub_mode') != -1:
                    sub_mode_str=line.split('>')[1].split('<')[0]
                    if self.combo_sub_mode.currentText() != sub_mode_str:
                        self.combo_sub_mode.setStyleSheet("background-color:rgb(40,40,40)")
                    else:
                        if self.dark_mode:
                            self.combo_sub_mode.setStyleSheet("background-color:rgb(128,0,0)")
                        else:
                            self.combo_sub_mode.setStyleSheet("background-color:rgb(160,128,128)")

        source_im_file = Path(self.source_folder,self.system_folder,'status.jpg')
        if os.path.exists(source_im_file):
            pixmap = QPixmap(str(source_im_file))
            self.im_label.setPixmap(pixmap)
            self.im_label.setPixmap(pixmap.scaled(self.im_label.size(),Qt.KeepAspectRatio, Qt.SmoothTransformation))
        try:
            txt,system_color = self.get_lbl_txt()
            self.txt_label.setText(txt)
        except:
            system_color = QColor(0,0,0)
        pal = QPalette(self.txt_label.palette())
        pal.setColor(QPalette.WindowText, system_color)
        self.txt_label.setPalette(pal)
        if self.combo_mode.currentText() == "Hunt":
            self.btn_insect_replay_takeoff.setEnabled(True)
        else:
            self.btn_insect_replay_takeoff.setEnabled(False)

        if self.combo_mode.currentText() == "Waypoint":
            self.btn_flightplan_takeoff.setEnabled(True)
        else:
            self.btn_flightplan_takeoff.setEnabled(False)

    def check_theme(self):
        theme = subprocess.check_output(["/usr/bin/gsettings get org.gnome.desktop.interface gtk-theme"], shell=True) #need to use full path /usr/bin because conda screws things up
        new_darkmode = str(theme).find('dark') != -1
        if new_darkmode and not self.dark_mode:
            self.refresh_darkmode = True
            self.dark_mode = True
        elif not new_darkmode and self.dark_mode:
            self.dark_mode = False
            self.refresh_darkmode = True
        syswidget_palette = self.palette()

        if self.dark_mode and self.refresh_darkmode:
            app.setStyle("Fusion")
            dark_palette = QPalette()
            dark_palette.setColor(QPalette.Window, QColor(15, 15, 15))
            dark_palette.setColor(QPalette.WindowText, QColor(230, 230, 230))
            app.setPalette(dark_palette)

            self.im_label.setStyleSheet("QToolTip { color: #000000; background-color: #800000; border: 1px solid black; }")
            self.combo_mode.setStyleSheet("background-color:rgb(128,0,0)")
            self.combo_sub_mode.setStyleSheet("background-color:rgb(128,0,0)")
            syswidget_palette.setColor(self.backgroundRole(), QColor(15,15,15))
            self.btn_insect_replay_takeoff.setStyleSheet("background-color:rgb(128,0,0)")
            self.btn_flightplan_takeoff.setStyleSheet("background-color:rgb(128,0,0)")
            self.btn_restart.setStyleSheet("background-color:rgb(128,0,0)")
            self.btn_request_update.setStyleSheet("background-color:rgb(128,0,0)")
            self.btn_download_current_log.setStyleSheet("background-color:rgb(128,0,0)")
        elif self.refresh_darkmode:

            light_palette = QPalette()
            light_palette.setColor(QPalette.Window, QColor(230, 230, 230))
            light_palette.setColor(QPalette.WindowText, QColor(15, 15, 15))
            app.setPalette(light_palette)

            self.im_label.setStyleSheet("QToolTip { color: #000000; background-color: #9d8080; border: 1px solid black; }")
            self.combo_mode.setStyleSheet("background-color:rgb(160,128,128)")
            self.combo_sub_mode.setStyleSheet("background-color:rgb(160,128,128)")
            syswidget_palette.setColor(self.backgroundRole(), QColor(230,230,230))
            self.btn_insect_replay_takeoff.setStyleSheet("background-color:rgb(160,128,128)")
            self.btn_flightplan_takeoff.setStyleSheet("background-color:rgb(160,128,128)")
            self.btn_restart.setStyleSheet("background-color:rgb(160,128,128)")
            self.btn_request_update.setStyleSheet("background-color:rgb(160,128,128)")
            self.btn_download_current_log.setStyleSheet("background-color:rgb(160,128,128)")

        self.refresh_darkmode = False
        self.setPalette(syswidget_palette)

    def check_if_flying(self,navstatus):
        return (navstatus =="ns_takeoff" or
            navstatus =="ns_taking_off" or
            navstatus =="ns_take_off_completed" or
            navstatus =="ns_start_the_chase" or
            navstatus =="ns_chasing_insect_ff" or
            navstatus =="ns_chasing_insect" or
            navstatus =="ns_set_waypoint" or
            navstatus =="ns_wp" or
            navstatus =="ns_flower_of_fire" or
            navstatus =="ns_brick_of_fire" or
            navstatus =="ns_goto_landing" or
            navstatus =="ns_goto_yaw" or
            navstatus =="ns_initial_reset_yaw" or
            navstatus =="ns_wait_reset_yaw" or
            navstatus =="ns_land" or
            navstatus =="ns_landing" or
            navstatus =="ns_landed" or
            navstatus =="ns_start_shaking" or
            navstatus =="ns_shaking_drone" or
            navstatus =="ns_wait_after_landing" or
            navstatus.startswith('wp_') or
            navstatus.startswith('wp ') or
            navstatus.startswith('ns_wp'))

    def get_lbl_txt(self):
        source_status_txt_file = Path(self.source_folder,self.system_folder,'status.txt')
        source_system_txt_file = Path(self.source_folder,self.system_folder,'system.txt')

        res_txt = ''
        system_has_problem = QColor(0,0,0)

        sysinfo_txt = ''
        if os.path.exists(source_system_txt_file):
            with open (source_system_txt_file, "r") as sysinf_txt_file:
                sysinfo_txt=sysinf_txt_file.readlines()
            self.host_id = sysinfo_txt[0].split(':')[1].strip().replace('pats-proto','').replace('pats','')
            self.drone_id = sysinfo_txt[1].split(':')[1].strip()
            sha = sysinfo_txt[2].split(':')[1].strip()[:6]

            hd = ''
            ip = ''
            for line in sysinfo_txt:
                if line.find("nvme0n1p2") != -1:
                    try:
                        hd = line.split()[4]
                    except:
                        pass
                if line.find('wlp58s0') != -1:
                    try:
                        ip = line.split()[3].split('/')[0]
                    except:
                        pass

            res_txt = "Pats-" + self.host_id + '->' + self.drone_id + ' @' + sha + '\n'
            res_txt = res_txt + 'hd: ' + hd + ' ip: ' + ip + '\n'
            if hd != '':
                hd = float(hd.replace('%',''))
                if hd > 95:
                    system_has_problem = QColor(255,0,0)
        else:
            res_txt = 'Error: system info file not found'
            system_has_problem = QColor(255,0,0)

        if os.path.exists(source_status_txt_file):
            status_txt = []
            with open (source_status_txt_file, "r") as status_txt_file:
                status_txt=status_txt_file.readlines()

            try:
                date_time = datetime.strptime(status_txt[0].strip(), '%Y/%m/%d %H:%M:%S') #e.g. 2020/02/29 23:45:46
                time_since_update = datetime.now() - date_time
                if time_since_update.total_seconds() > 3600*24:
                    system_has_problem = QColor(200,0,0)

                res_txt += status_txt[1].strip() + '. upd: ' + str(int(time_since_update.total_seconds())) + 's\n'
            except:
                pass

            if len(status_txt) > 2:

                if len(status_txt) >= 5:
                    if (status_txt[4].startswith("arming: ")):
                        arm_state = int(status_txt[4][8:-1])
                        res_txt += " Arm: " + status_txt[4][8:-1] + ", "
                    if (status_txt[3].startswith("cell v: ")):
                        cell_v = float(status_txt[3][8:-1])
                        navstatus = status_txt[2].strip()
                        if cell_v > 1 and not self.check_if_flying(navstatus) and not navstatus == "ns_drone_problem":
                            if cell_v < 3.8:
                                system_has_problem = QColor(255,165,0)
                            if cell_v < 3.1:
                                system_has_problem = QColor(255,0,0)
                        res_txt += status_txt[3][8:-1] + 'V, '
                    if (status_txt[5].startswith("rssi: ")):
                        res_txt += " rssi: " + status_txt[5][6:-1] + ", "

                if (status_txt[2].startswith("ns_")):
                    res_txt += "state: " + status_txt[2][3:]
                else:
                    res_txt += status_txt[2]
                navstatus = status_txt[2].strip()
                if system_has_problem.getRgb()[0] == 0:

                    if (navstatus ==  "ns_init" or
                        navstatus =="ns_locate_drone_init" or
                        navstatus =="ns_locate_drone_led"  or
                        navstatus =="ns_located_drone" or
                        navstatus =="ns_calibrating_motion" or
                        navstatus =="ns_calibrating_drone" or
                        navstatus.startswith('Starting') or
                        navstatus.startswith('Resetting') or
                        navstatus.startswith('Closed') or
                        navstatus.startswith('Waiting.') or
                        navstatus.startswith('Closing')) :
                            system_has_problem = QColor(128,128,128)

                    elif navstatus.startswith("ns_wait_locate_drone"):
                        if int(navstatus.split(' ')[1]) == 1:
                            system_has_problem = QColor(128,128,128)
                        elif int(navstatus.split(' ')[1]) == 2:
                            system_has_problem = QColor(128,128,0)
                        elif int(navstatus.split(' ')[1]) > 2:
                            system_has_problem = QColor(255,0,0)
                        else:
                            system_has_problem = QColor(128,128,128)
                    elif (navstatus =="ns_wait_for_takeoff" or navstatus =="ns_manual" ):
                            system_has_problem = QColor(0,255,255)
                    elif (navstatus =="ns_monitoring"):
                        system_has_problem = QColor(255,192,203)
                    elif ( navstatus =="ns_wait_for_insect"):
                            system_has_problem = QColor(0,128,0)
                    elif(self.check_if_flying(navstatus)):
                        system_has_problem = QColor(0,255,0)
                    elif (navstatus == "ns_drone_problem" or
                        navstatus == "ns_batlow" or
                        navstatus == 'no RealSense connected'):
                        system_has_problem = QColor(255,0,0)
                    elif navstatus.startswith('Roll'):
                        roll = float(navstatus.split(' ')[1])
                        if (abs(roll) > self.max_cam_roll):
                            system_has_problem = QColor(255,0,0)
                        else:
                            system_has_problem = QColor(128,128,128)
        else:
            res_txt = 'Error: status info file not found'
            system_has_problem = QColor(255,0,0)

        if system_has_problem.getRgb()[0] == 0 and system_has_problem.getRgb()[1] == 0 and system_has_problem.getRgb()[2] == 0:
            system_has_problem = QColor(100,100,100)

        res_txt = res_txt.strip()
        return res_txt,system_has_problem

class ImDialog(QDialog):
    def __init__(self,parent,source_folder,system_folder,host_id,dark_mode):
        super().__init__(parent)

        self.source_folder = source_folder
        self.system_folder = system_folder
        self.host_id = host_id
        self.source_im_file = Path(self.source_folder,self.system_folder,'status.jpg')
        self.pats_xml_path = Path(self.source_folder,self.system_folder,'pats_deploy.xml')

        self.pats_xml_txt = ''
        self.drone_xml_txt = ''
        self.flightplan_xml_txt = ''
        if os.path.exists(self.pats_xml_path):

            with open (self.pats_xml_path, "r") as path_xml:
                self.pats_xml_txt=path_xml.read()
            with open (self.pats_xml_path, "r") as path_xml:
                xml_lines = path_xml.readlines()
                for line in xml_lines:
                    if line.find('\"drone\"') != -1:
                        self.drone_xml_path = Path(self.source_folder,self.system_folder,line.split('\">')[1].split('<')[0] + '.xml')
                        if os.path.exists(self.drone_xml_path):
                            with open (self.drone_xml_path, "r") as drone_xml:
                                self.drone_xml_txt=drone_xml.read()
                    if line.find('\"flightplan\"') != -1:
                        self.flightplan_xml_path = Path(self.source_folder,self.system_folder,line.split('\">../xml/')[1].split('<')[0])
                        if os.path.exists(self.flightplan_xml_path):
                            with open (self.flightplan_xml_path, "r") as flightplan_xml:
                                self.flightplan_xml_txt=flightplan_xml.read()


        self.im_label = QLabel()

        self.status_lbl = QLabel()
        self.system_lbl = QLabel()
        self.pats_xml_textBox = QPlainTextEdit()
        self.drone_xml_textBox = QPlainTextEdit()
        self.flightplan_xml_textBox = QPlainTextEdit()

        if dark_mode:
            self.status_lbl.setStyleSheet("color: rgb(200, 0, 0); background-color: rgb(25, 25, 25);")
        self.status_lbl.setMaximumHeight(300)
        if dark_mode:
            self.system_lbl.setStyleSheet("color: rgb(200, 0, 0); background-color: rgb(25, 25, 25);")
        self.system_lbl.setMaximumHeight(300)

        self.pats_xml_textBox.textChanged.connect(self.xml_txt_chng_event)
        if dark_mode:
            self.pats_xml_textBox.setStyleSheet("color: rgb(200, 0, 0); background-color: rgb(25, 25, 25);")
        self.pats_xml_textBox.setPlainText(self.pats_xml_txt)
        self.pats_xml_textBox.setMaximumHeight(300)

        self.drone_xml_textBox.textChanged.connect(self.xml_txt_chng_event)
        if dark_mode:
            self.drone_xml_textBox.setStyleSheet("color: rgb(200, 0, 0); background-color: rgb(25, 25, 25);")
        self.drone_xml_textBox.setPlainText(self.drone_xml_txt)
        self.drone_xml_textBox.setMaximumHeight(300)

        self.flightplan_xml_textBox.textChanged.connect(self.xml_txt_chng_event)
        if dark_mode:
            self.flightplan_xml_textBox.setStyleSheet("color: rgb(200, 0, 0); background-color: rgb(25, 25, 25);")
        self.flightplan_xml_textBox.setPlainText(self.flightplan_xml_txt)
        self.flightplan_xml_textBox.setMaximumHeight(300)

        self.tabs = QTabWidget()
        self.tab_status = QWidget()
        self.tab_pats = QWidget()
        self.tab_drone = QWidget()
        self.tab_flightplan = QWidget()
        self.tabs.resize(300,200)

        self.tab_status.layout = QHBoxLayout(self.tab_status)
        self.tab_status.layout.addWidget(self.status_lbl)
        self.tab_status.layout.addWidget(self.system_lbl)
        self.tabs.addTab(self.tab_status,"Status")

        self.tab_pats.layout = QVBoxLayout(self.tab_pats)
        self.tab_pats.layout.addWidget(self.pats_xml_textBox)
        self.tabs.addTab(self.tab_pats,"Pats")

        self.tab_drone.layout = QVBoxLayout(self.tab_drone)
        self.tab_drone.layout.addWidget(self.drone_xml_textBox)
        self.tabs.addTab(self.tab_drone,"Drone")

        self.tab_flightplan.layout = QVBoxLayout(self.tab_flightplan)
        self.tab_flightplan.layout.addWidget(self.flightplan_xml_textBox)
        self.tabs.addTab(self.tab_flightplan,"Flightplan")

        self.tabs.setMaximumHeight(300)
        if dark_mode:
            self.tabs.setStyleSheet("color: rgb(200, 0, 0); background-color: rgb(25, 25, 25);")

        v_box = QVBoxLayout()
        v_box.addWidget(self.im_label)
        v_box.addWidget(self.tabs)

        self.setLayout(v_box)

        timer = QTimer(self)
        timer.timeout.connect(self.refresh)
        timer.start(1000)

        self.refresh()

        self.setAutoFillBackground(True)
        p = self.palette()
        if dark_mode:
            p.setColor(self.backgroundRole(), QColor(15,15,15))
        self.setPalette(p)

        self.setWindowTitle(self.host_id)
        self.setWindowFlags(Qt.Window)
        self.show()
        self.refresh() #work around for small initial zooming issue

    def xml_txt_chng_event(self):
        new_pats_xml = self.pats_xml_textBox.toPlainText()
        new_drone_xml = self.drone_xml_textBox.toPlainText()
        new_flightplan_xml = self.flightplan_xml_textBox.toPlainText()
        if self.pats_xml_txt != new_pats_xml or self.drone_xml_txt != new_drone_xml or self.flightplan_xml_txt != new_flightplan_xml:
            self.setWindowTitle(self.host_id + '*')
        else:
            self.setWindowTitle(self.host_id)


    def refresh(self):
        pixmap = QPixmap(str(self.source_im_file))
        self.setMinimumSize(pixmap.width(),pixmap.height())
        self.im_label.setPixmap(pixmap)
        self.im_label.setPixmap(pixmap.scaled(self.im_label.size(),Qt.KeepAspectRatio, Qt.SmoothTransformation))

        source_status_txt_file = Path(self.source_folder,self.system_folder,'status.txt')
        status_txt = ''
        if os.path.exists(source_status_txt_file):
            status_txt = []
            with open (source_status_txt_file, "r") as status_txt_file:
                status_txt=status_txt_file.read()
        source_system_txt_file = Path(self.source_folder,self.system_folder,'system.txt')
        if os.path.exists(source_system_txt_file):
            with open (source_system_txt_file, "r") as sysinf_txt_file:
                sysinfo_txt=sysinf_txt_file.read()

        self.status_lbl.setText('Status:\n' + status_txt)
        self.system_lbl.setText('System:\n' + sysinfo_txt)

    def keyPressEvent(self, event: QKeyEvent):
        if event.key() in {Qt.Key_Space, Qt.Key_Escape}:
            self.close()
        elif event.modifiers() & Qt.ControlModifier and event.key() == Qt.Key_S:
            self.save_xml()

    def save_xml(self):
        new_pats_xml = self.pats_xml_textBox.toPlainText()
        pats_xml_tmp_file = open("pats.tmp", "w")
        pats_xml_tmp_file.write(new_pats_xml)
        pats_xml_tmp_file.close()

        new_drone_xml = self.drone_xml_textBox.toPlainText()
        drone_xml_tmp_file = open("drone.tmp", "w")
        drone_xml_tmp_file.write(new_drone_xml)
        drone_xml_tmp_file.close()

        new_flightplan_xml = self.flightplan_xml_textBox.toPlainText()
        flightplan_xml_tmp_file = open("flightplan.tmp", "w")
        flightplan_xml_tmp_file.write(new_flightplan_xml)
        flightplan_xml_tmp_file.close()

        self.setWindowTitle(self.host_id)
        subprocess.Popen(['./change_settings_system.sh', 'pats'+self.host_id,os.path.basename(self.drone_xml_path),os.path.basename(self.flightplan_xml_path) ])
        xml_file = open(self.pats_xml_path, "w")
        xml_file.write(new_pats_xml)
        xml_file.close()

        xml_file = open(self.drone_xml_path, "w")
        xml_file.write(new_drone_xml)
        xml_file.close()

        xml_file = open(self.flightplan_xml_path, "w")
        xml_file.write(new_flightplan_xml)
        xml_file.close()

class SelectSystemsDialog(QDialog):

    def __init__(self,parent,source_folder,system_folders):
        super().__init__(parent)
        layout = QGridLayout()
        self.setLayout(layout)
        self.source_folder = source_folder

        self.listwidget = QListWidget()
        i=0
        for f in system_folders:

            item = QListWidgetItem(f)
            if os.path.exists(Path(source_folder,f,'.select')):
                item.setCheckState(Qt.Checked)
            else:
                item.setCheckState(Qt.Unchecked)
            self.listwidget.addItem(item)


        select_all_PB = QPushButton()
        select_all_PB.setToolTip('Select all')
        select_all_PB.setText('Select all')
        select_all_PB.clicked.connect(self.select_all)
        self.select_all_PB = select_all_PB

        deselect_all_PB = QPushButton()
        deselect_all_PB.setToolTip('Deselect all')
        deselect_all_PB.setText('Deselect all')
        deselect_all_PB.clicked.connect(self.deselect_all)
        self.deselect_all_PB = deselect_all_PB

        layout.addWidget(self.select_all_PB)
        layout.addWidget(self.deselect_all_PB)
        layout.addWidget(self.listwidget)

        self.exec()

    def deselect_all(self,event):
        for i in range(self.listwidget.count()):
            item = self.listwidget.item(i)
            item.setCheckState(Qt.Unchecked)

    def select_all(self,event):
        for i in range(self.listwidget.count()):
            item = self.listwidget.item(i)
            item.setCheckState(Qt.Checked)

    def closeEvent(self, event):
        for i in range(self.listwidget.count()):
            item = self.listwidget.item(i)
            p = Path(self.source_folder,item.text(),'.select')
            if (item.checkState()):
                if (not os.path.exists(p)):
                    open(p,'a').close()
            else:
                if (os.path.exists(p)):
                    os.remove(p)

    def keyPressEvent(self, event: QKeyEvent):
        if event.key() in {Qt.Key_Space, Qt.Key_Escape}:
            self.close()

def atoi(text):
    return int(text) if text.isdigit() else text

def natural_keys(text):
    '''
    alist.sort(key=natural_keys) sorts in human order
    http://nedbatchelder.com/blog/200712/human_sorting.html
    (See Toothy's implementation in the comments)
    '''
    return [ atoi(c) for c in re.split(r'(\d+)', text) ]

if __name__ == '__main__':
    app = QApplication(sys.argv)
    cc = CommandCenterWindow(app.primaryScreen().size())
    cc.show()

    sys.exit(app.exec_())
