#!/usr/bin/env python3

import os,socket,subprocess,time,sys,glob
import re, datetime, argparse, shutil
from pathlib import Path
from tqdm import tqdm

def execute(cmd):
    popen = subprocess.Popen(cmd, shell=True,stderr=subprocess.STDOUT,stdout=subprocess.PIPE)
    for stdout_line in iter(popen.stdout.readline, ""):
        if popen.poll() != None:
            break
        print(stdout_line.decode('utf-8'),end ='')
    popen.stdout.close()

def system_was_monitoring_in_folder(folder):
    pats_xml_mode = ''
    pats_xml_path = Path(folder,'logging','pats.xml')
    if not os.path.exists(pats_xml_path):
        print("Error: pats.xml not found in: " + folder)
        return False
    with open (pats_xml_path, "r") as pats_xml:
            xml_lines = pats_xml.readlines()
            for line in xml_lines:
                if line.find('op_mode') != -1:
                    if line.find('op_mode_monitoring') != -1:
                        return True
                    else:
                        print("Not a monitoring folder")
                        return False
    print("Error: op_mode not found in pats.xml not found in: " + folder)
    return False

def natural_sort(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key=alphanum_key)


parser = argparse.ArgumentParser(description='Script that cuts the raw video into moths based on the log')
parser.add_argument('-i', help="Path to the folder", required=True)
args = parser.parse_args()
folder = args.i

frames_fn = folder + '/logging/frames.csv'
if not os.path.exists(frames_fn):
    print("Frames.csv not found")
    exit()
with open(frames_fn, 'r') as flog:

    if system_was_monitoring_in_folder(folder):
        files = tqdm(natural_sort([fp for fp in glob.glob(os.path.join(folder, "logging", "log_itrk*.csv")) if "itrk0" not in fp]))

        video_in_file = folder + '/logging/videoRawLR.mkv'
        if os.path.exists(video_in_file):
            for file in files:
                files.set_description(os.path.basename(file))
                with open(file, 'r') as ilog:
                    lines = ilog.read().splitlines()
                    fn = os.path.basename(file)
                    file_id = fn.split('.')[0][8:]
                    if len(lines) > 40: # only cut a video if it is of substantial length...
                        video_start_rs_id = int(lines[1].split(';')[0])
                        while True:
                            fline = flog.readline()
                            if not fline or len(fline.split(';')) !=4:
                                print('Error: could not find frame in frames.csv')
                                exit(0)
                            if int(fline.split(';')[2]) == video_start_rs_id:
                                video_start_video_id = int(fline.split(';')[1])
                                video_start_time = float(video_start_video_id)/90
                                break

                        video_start_time = video_start_time - 2
                        if (video_start_time<0):
                            video_start_time = 0
                        video_end_rs_id = int(lines[-2].split(';')[0])
                        video_end_video_id = video_end_rs_id - (video_start_rs_id-video_start_video_id)
                        video_end_time = float(video_end_video_id)/90 + 2
                        video_duration = video_end_time - video_start_time
                        print (str(file_id) + ': from ' + str(video_start_time) + ' to ' + str(video_end_time))
                        log_folder = folder + '/logging/'

                        video_out_file = log_folder + '/moth' + file_id + '.mkv'

                        cmd = 'ffmpeg -y -i ' + video_in_file + ' -ss ' + str(video_start_time) + ' -t ' + str(video_duration) + ' -c:v copy -an ' + video_out_file
                        execute(cmd)
            os.remove(video_in_file)
        else:
            print("VideoRawLR not found")
