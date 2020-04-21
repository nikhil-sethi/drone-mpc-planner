#!/usr/bin/env python3
from PyQt5.QtCore import QDir, Qt, QUrl,QRect,QTimer
from PyQt5.QtMultimedia import QMediaContent, QMediaPlayer
from PyQt5.QtMultimediaWidgets import QVideoWidget
from PyQt5.QtWidgets import (QMainWindow,QDialog,QWidget, QPushButton, QCheckBox, QAction,QFrame,
    QApplication, QFileDialog, QHBoxLayout, QLabel,QPushButton, QSizePolicy, QSlider, QStyle,
    QVBoxLayout, QGridLayout, QWidget,QMessageBox,QPlainTextEdit,QMenu,QTabWidget)
from PyQt5.QtGui import QIcon, QPixmap, QPalette,QColor,QKeyEvent

from pathlib import Path
from datetime import datetime
import random,sys,os,re,subprocess,math
class CommandCenterWindow(QMainWindow):

    def __init__(self, parent=None):
        super(CommandCenterWindow, self).__init__(parent)
        self.setWindowTitle("Pats Command Center")
        self.setWindowIcon(QIcon("./icon.png"))
        wid = QWidget(self)
        self.setCentralWidget(wid)
        self.setSizePolicy(QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed))
        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), QColor(15,15,15))
        self.setPalette(p)
        layout = QGridLayout()

        source_folder = '~/Downloads/pats_status'
        source_folder = os.path.expanduser(source_folder.strip(" "))
        self.source_folder = source_folder
        self.download(True)

        systems = next(os.walk(source_folder))[1]
        systems.sort(key=natural_keys)
        self.system_folders = systems
        n = len(systems)
        np = int(math.ceil(n/4))

        self.sys_widgets = []
        i=0
        for f in self.system_folders:
            s = SystemWidget(source_folder,f)
            self.sys_widgets.append(s)
            layout.addWidget(s,int(i/np),i % np)
            i = i + 1

        layout.setGeometry(QRect(0, 0, 640 , 480))
        wid.setLayout(layout)

        timer = QTimer(self)
        timer.timeout.connect(self.refresh)
        timer.start(1000)

        self.refresh()
        self.show()
        self.refresh()

    def download(self,wait=False):
        rsync_src='mavlab-gpu:/home/pats/status/'
        subprocess.call(['mkdir -p ' + self.source_folder ], shell=True)
        cmd = ['rsync -zva --timeout=3 --exclude \'*.jpg.*\' --exclude \'*.xml.*\' --exclude \'*.txt.*\' ' + rsync_src + ' '+ self.source_folder]
        if wait:
            subprocess.call(cmd, shell=True,stdout=subprocess.PIPE)
        else:
            subprocess.Popen(cmd, shell=True,stdout=subprocess.PIPE)

    def refresh(self):
        self.download()
        for sys in self.sys_widgets:
            sys.refresh()

class SystemWidget(QWidget):
    def __init__(self, source_folder,system_folder,parent=None):
        QWidget.__init__(self, parent=parent)
        self.disable_chk_enable_events = False

        self.system_folder = system_folder
        self.source_folder = source_folder

        self.chk_enable = QCheckBox()
        self.chk_enable.setStyleSheet("background-color:rgb(128,0,0)")
        self.chk_enable.setChecked(True)
        self.chk_enable.stateChanged.connect(self.activate)

        self.setContextMenuPolicy(Qt.ActionsContextMenu)
        calibAction = QAction("Calibrate", self)
        calibAction.setIcon(self.style().standardIcon(QStyle.SP_DesktopIcon))
        calibAction.triggered.connect(self.calib)
        self.addAction(calibAction)

        reboot_rtc_Action = QAction("Reboot with rtc timeout", self)
        reboot_rtc_Action.setIcon(self.style().standardIcon(QStyle.SP_DialogResetButton))
        reboot_rtc_Action.triggered.connect(self.reboot)
        self.addAction(reboot_rtc_Action)

        btn_restart = QPushButton()
        btn_restart.setToolTip('Restart pats process')
        btn_restart.setIcon(self.style().standardIcon(QStyle.SP_BrowserReload))
        btn_restart.setMaximumSize(30, 30)
        btn_restart.setStyleSheet("background-color:rgb(128,0,0)")
        btn_restart.clicked.connect(self.restart)
        self.btn_restart = btn_restart

        btn_save_bag = QPushButton()
        btn_save_bag.setToolTip('Restart & save the bag')
        btn_save_bag.setIcon(self.style().standardIcon(QStyle.SP_DriveFDIcon))
        btn_save_bag.setMaximumSize(30, 30)
        btn_save_bag.setStyleSheet("background-color:rgb(128,0,0)")
        btn_save_bag.clicked.connect(self.save_bag)
        self.btn_save_bag = btn_save_bag

        btn_beep = QPushButton()
        btn_beep.setToolTip('Beep & blink')
        btn_beep.setIcon(self.style().standardIcon(QStyle.SP_MediaVolume))
        btn_beep.setMaximumSize(30, 30)
        btn_beep.setStyleSheet("background-color:rgb(128,0,0)")
        btn_beep.clicked.connect(self.beep)
        self.btn_beep = btn_beep

        btn_update = QPushButton()
        btn_update.setToolTip('Update from git')
        btn_update.setIcon(self.style().standardIcon(QStyle.SP_FileDialogToParent))
        btn_update.setMaximumSize(30, 30)
        btn_update.setStyleSheet("background-color:rgb(128,0,0)")
        btn_update.clicked.connect(self.update)
        self.btn_update = btn_update

        btn_takeoff = QPushButton()
        btn_takeoff.setToolTip('Fly waypoint mission')
        btn_takeoff.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        btn_takeoff.setMaximumSize(30, 30)
        btn_takeoff.setStyleSheet("background-color:rgb(128,0,0)")
        btn_takeoff.clicked.connect(self.takeoff)
        self.btn_takeoff = btn_takeoff

        btn_insect_replay_takeoff = QPushButton()
        btn_insect_replay_takeoff.setToolTip('Hunt insect replay')
        btn_insect_replay_takeoff.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        btn_insect_replay_takeoff.setMaximumSize(30, 30)
        btn_insect_replay_takeoff.setStyleSheet("background-color:rgb(128,0,0)")
        btn_insect_replay_takeoff.clicked.connect(self.insect_replay_takeoff)
        self.btn_insect_replay_takeoff = btn_insect_replay_takeoff

        control_layout = QHBoxLayout()
        control_layout.setContentsMargins(0, 0, 0, 0)
        control_layout.addWidget(self.chk_enable)
        control_layout.addWidget(btn_restart)
        control_layout.addWidget(btn_save_bag)
        control_layout.addWidget(btn_update)
        control_layout.addWidget(btn_beep)
        control_layout.addWidget(btn_takeoff)
        control_layout.addWidget(btn_insect_replay_takeoff)

        self.txt_label = QLabel()

        self.im_label = QLabel()
        self.im_label.mouseReleaseEvent = self.takeoshow_im_big
        self.im_label.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Expanding)
        self.im_label.setAlignment(Qt.AlignCenter)
        self.im_label.setMinimumSize(100, 100)
        self.im_label.setToolTip('Click to enlarge!')

        sub_layout = QVBoxLayout(self)
        sub_layout.setContentsMargins(0, 0, 0, 0)
        sub_layout.addWidget(self.im_label)
        sub_layout.addWidget(self.txt_label)
        sub_layout.addLayout(control_layout)

    def activate(self):
        if self.disable_chk_enable_events:
            return
        self.pats_xml_path = Path(self.source_folder,self.system_folder,'pats_deploy.xml')
        with open (self.pats_xml_path, "r") as path_xml:
            xml_lines = path_xml.readlines()
            new_xml_lines = ''
            for line in xml_lines:
                if line.find('\"op_mode\"') != -1:
                    if self.chk_enable.isChecked():
                        line = '    <Member Name=\"op_mode\">op_mode_deployed</Member>\n'
                    else:
                        line = '    <Member Name=\"op_mode\">op_mode_crippled</Member>\n'
                elif line.find('\"live_image_frq\"') != -1:
                    if self.chk_enable.isChecked():
                        line = '    <Member Name=\"live_image_frq\">1</Member>\n'
                    else:
                        line = '    <Member Name=\"live_image_frq\">30</Member>\n'
                elif line.find('\"darkness_threshold\"') != -1:
                    if self.chk_enable.isChecked():
                        line = '    <Member Name=\"darkness_threshold\">0</Member>\n'
                    else:
                        line = '    <Member Name=\"darkness_threshold\">1000</Member>\n'
                new_xml_lines = new_xml_lines + line

        pats_xml_tmp_file = open("pats.tmp", "w")
        pats_xml_tmp_file.write(new_xml_lines)
        pats_xml_tmp_file.close()
        subprocess.Popen(['./change_settings_system.sh', 'pats'+self.host_id ])

        xml_file = open(self.pats_xml_path, "w")
        xml_file.write(new_xml_lines)
        xml_file.close()

    def calib(self):
        subprocess.Popen(['./calib_system.sh', 'pats'+self.host_id])
    def beep(self):
        subprocess.Popen(['./beep_system.sh', 'pats'+self.host_id])

    def restart(self):
        subprocess.Popen(['./restart_system.sh', 'pats'+self.host_id])
    def save_bag(self):
        subprocess.Popen(['./savebag_system.sh', 'pats'+self.host_id])
    def update(self):
        subprocess.Popen(['./update_system.sh', 'pats'+self.host_id])
    def reboot(self):
        buttonReply = QMessageBox.question(self, 'Maar, weet je dat eigenlijk wel zekerdepeter?!', "Reboot, really?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if buttonReply == QMessageBox.Yes:
            print('Yes clicked.')
            subprocess.Popen(['./reboot_system.sh', 'pats'+self.host_id])
    def takeoff(self):
        self.pats_xml_path = Path(self.source_folder,self.system_folder,'pats_deploy.xml')
        with open (self.pats_xml_path, "r") as path_xml:
            xml_lines = path_xml.readlines()
            for line in xml_lines:
                if line.find('\"flightplan\"') != -1:
                    self.flightplan_xml_path = Path(self.source_folder,self.system_folder,line.split('\">../../xml/')[1].split('<')[0])
                    subprocess.Popen(['./demo_system.sh', 'pats'+self.host_id, self.flightplan_xml_path])
    def insect_replay_takeoff(self):
        subprocess.Popen(['./insect_replay_system.sh', 'pats'+self.host_id])
    def takeoshow_im_big(self,event):
        ImDialog(self,self.source_folder,self.system_folder,self.host_id)

    def refresh(self):
        self.pats_xml_path = Path(self.source_folder,self.system_folder,'pats_deploy.xml')
        with open (self.pats_xml_path, "r") as path_xml:
            xml_lines = path_xml.readlines()
            new_xml_lines = ''
            for line in xml_lines:
                if line.find('op_mode_deployed') != -1 and not self.chk_enable.isChecked():
                    self.disable_chk_enable_events = True
                    self.chk_enable.setChecked(True)
                    self.disable_chk_enable_events = False
                if line.find('op_mode_crippled') != -1 and self.chk_enable.isChecked():
                    self.disable_chk_enable_events = True
                    self.chk_enable.setChecked(False)
                    self.disable_chk_enable_events = False

        source_im_file = Path(self.source_folder,self.system_folder,'status.jpg')
        pixmap = QPixmap(str(source_im_file))
        self.im_label.setPixmap(pixmap)
        self.im_label.setPixmap(pixmap.scaled(self.im_label.size(),Qt.KeepAspectRatio, Qt.SmoothTransformation))
        txt,system_color = self.get_lbl_txt()
        self.txt_label.setText(txt)
        if self.chk_enable.isChecked():
            pal = QPalette(self.txt_label.palette())
            pal.setColor(QPalette.WindowText, system_color)
            self.txt_label.setPalette(pal)
            self.btn_insect_replay_takeoff.setEnabled(True)
            self.btn_takeoff.setEnabled(True)
        else:
            # pixmap = QPixmap('./darkdummy.jpg')
            # self.im_label.setPixmap(pixmap)
            # self.im_label.setPixmap(pixmap.scaled(self.im_label.size(),Qt.KeepAspectRatio, Qt.SmoothTransformation))
            pal = QPalette(self.txt_label.palette())
            pal.setColor(QPalette.WindowText, QColor(60,0,0))
            self.txt_label.setPalette(pal)
            self.btn_insect_replay_takeoff.setEnabled(False)
            self.btn_takeoff.setEnabled(False)

    def get_lbl_txt(self):
        source_status_txt_file = Path(self.source_folder,self.system_folder,'status.txt')
        source_system_txt_file = Path(self.source_folder,self.system_folder,'system.txt')

        res_txt = ''
        system_has_problem = QColor(0,0,0)

        sysinfo_txt = ''
        if os.path.exists(source_system_txt_file):
            with open (source_system_txt_file, "r") as sysinf_txt_file:
                sysinfo_txt=sysinf_txt_file.readlines()
            self.host_id = sysinfo_txt[0].split(':')[1].strip().replace('pats-proto','')
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
                if time_since_update.total_seconds() > 300:
                    system_has_problem = QColor(200,0,0)

                res_txt += status_txt[1].strip() + '. upd: ' + str(int(time_since_update.total_seconds())) + 's\n'
            except:
                pass

            if len(status_txt) > 2:
                res_txt += status_txt[2]
                navstatus = status_txt[2].strip()
                if system_has_problem.getRgb()[0] == 0:
                    if navstatus == 'ns_wait_for_insect':
                        system_has_problem = QColor(0,128,0)
                    elif navstatus.startswith('ns_approach_wp') or navstatus == 'ns_taking_off' or navstatus == 'ns_chasing_insect'or navstatus == 'ns_landing':
                        system_has_problem = QColor(0,255,0)
                    elif navstatus == 'ns_wait_locate_drone':
                        system_has_problem = QColor(255,165,0)
                    elif navstatus.startswith('Roll') or navstatus.startswith('Starting') or navstatus == 'ns_init' or navstatus.startswith('Resetting') or navstatus.startswith('Closed') or navstatus.startswith('Closing'):
                        system_has_problem = QColor(180,180,0)
                if navstatus == 'ns_drone_problem':
                    system_has_problem = QColor(255,0,0)
                elif navstatus == 'no RealSense connected':
                    system_has_problem = QColor(255,0,0)
        else:
            res_txt = 'Error: status info file not found'
            system_has_problem = QColor(255,0,0)

        if system_has_problem.getRgb()[0] == 0 and system_has_problem.getRgb()[1] == 0 and system_has_problem.getRgb()[2] == 0:
            system_has_problem = QColor(100,100,100)

        return res_txt.strip(),system_has_problem

class ImDialog(QDialog):
    def __init__(self,parent,source_folder,system_folder,host_id):
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
                        self.flightplan_xml_path = Path(self.source_folder,self.system_folder,line.split('\">../../xml/')[1].split('<')[0])
                        if os.path.exists(self.flightplan_xml_path):
                            with open (self.flightplan_xml_path, "r") as flightplan_xml:
                                self.flightplan_xml_txt=flightplan_xml.read()


        self.im_label = QLabel()

        self.pats_xml_textBox = QPlainTextEdit()
        self.drone_xml_textBox = QPlainTextEdit()
        self.flightplan_xml_textBox = QPlainTextEdit()

        self.pats_xml_textBox.textChanged.connect(self.xml_txt_chng_event)
        self.pats_xml_textBox.setStyleSheet("color: rgb(200, 0, 0); background-color: rgb(25, 25, 25);")
        self.pats_xml_textBox.setPlainText(self.pats_xml_txt)
        self.pats_xml_textBox.setMaximumHeight(300)

        self.drone_xml_textBox.textChanged.connect(self.xml_txt_chng_event)
        self.drone_xml_textBox.setStyleSheet("color: rgb(200, 0, 0); background-color: rgb(25, 25, 25);")
        self.drone_xml_textBox.setPlainText(self.drone_xml_txt)
        self.drone_xml_textBox.setMaximumHeight(300)


        self.flightplan_xml_textBox.textChanged.connect(self.xml_txt_chng_event)
        self.flightplan_xml_textBox.setStyleSheet("color: rgb(200, 0, 0); background-color: rgb(25, 25, 25);")
        self.flightplan_xml_textBox.setPlainText(self.flightplan_xml_txt)
        self.flightplan_xml_textBox.setMaximumHeight(300)

        self.tabs = QTabWidget()
        self.tab_pats = QWidget()
        self.tab_drone = QWidget()
        self.tab_flightplan = QWidget()
        self.tabs.resize(300,200)

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
    cc = CommandCenterWindow()
    cc.show()

    sys.exit(app.exec_())
