#!/usr/bin/env python3

import os,socket,subprocess,time,sys,glob
import re, datetime, argparse, shutil
from tqdm import tqdm
from pathlib import Path

def todatetime(string):
    return datetime.datetime.strptime(string, "%Y%m%d_%H%M%S")

def execute(cmd,render_process_dir):
    popen = subprocess.Popen(cmd, shell=True,stderr=subprocess.STDOUT,stdout=subprocess.PIPE,cwd=render_process_dir)
    for stdout_line in iter(popen.stdout.readline, ""):
        if popen.poll() != None:
            break
        print(stdout_line.decode('utf-8'),end ='')
    popen.stdout.close()

parser = argparse.ArgumentParser(description='Script that renders video result in all pats log folders in a directory, bound by the minimum and maximum date\n\
    example: ./render_video.py -i "{path_directory_of_log_dirs}" -s "20200729_190000" -e "20200729_230000"')

parser.add_argument('-i', help="Path to the folder with logs", required=False, default='~/data/')
parser.add_argument('-s', help="Directory date to start from", required=False, default='20000101_000000')
parser.add_argument('-e', help="Directory date to end on", required=False, default='30000101_000000') #possible milenium bug

args = parser.parse_args()

min_date = todatetime(args.s)
max_date = todatetime(args.e)

original_process_dir = os.path.expanduser('~/code/pats/pc/build/')
render_process_dir = os.path.expanduser('~/code/pats/pc/build_render/')

if os.path.exists(render_process_dir):
    shutil.rmtree(render_process_dir)
shutil.copytree(original_process_dir,render_process_dir,ignore=shutil.ignore_patterns('*logging*'))
found_dirs = glob.glob(os.path.expanduser(args.i) + "/202*_*")
filtered_dirs = [d for d in found_dirs if todatetime(os.path.basename(os.path.normpath(d))) >= min_date and todatetime(os.path.basename(os.path.normpath(d))) <= max_date] # filter the list of dirs to only contain dirs between certain dates
for folder in filtered_dirs:
    print(f"Processing {folder}")
    pats_xml_path = Path(folder,'logging','pats.xml')
    results_txt_path = Path(folder,'logging','results.txt')
    target_path = Path(Path(os.path.expanduser('~/data_rendered/')),os.path.basename(folder) + '_' + socket.gethostname() + '.mkv')
    if not os.path.exists(target_path) and os.path.exists(pats_xml_path) and os.path.exists(results_txt_path):
        hunt_mode=False
        n_hunts = 0
        n_takeoffs=0
        with open (pats_xml_path, "r") as pats_xml:
            xml_lines = pats_xml.readlines()
            for line in xml_lines:
                if line.find('op_mode_hunt') != -1 or line.find('op_mode_deploy') != -1:
                    hunt_mode = True
        with open (results_txt_path, "r") as results_txt:
            results_lines = results_txt.readlines()
            for line in results_lines:
                if line.find('n_hunts') != -1:
                    n_hunts = int(line.split(':')[1])
                if line.find('n_takeoffs') != -1:
                    n_takeoffs = int(line.split(':')[1])
        if hunt_mode and n_hunts > 0 and n_takeoffs > 0:
            cmd = './pats --log ' +folder + '/logging --render'
            execute(cmd,render_process_dir)
            video_result_path = Path(render_process_dir, 'logging/replay/videoResult.mkv')
            if os.path.exists(video_result_path):
                Path(os.path.expanduser('~/data_rendered/')).mkdir(parents=True, exist_ok=True)
                shutil.copyfile(video_result_path,target_path)
                shutil.move(str(video_result_path), Path(folder,'videoResult.mkv'))

        #render insects:
        files = tqdm(os.listdir(folder + '/logging/'))
        for file in files:
            if file.startswith('moth') and file.endswith(".mkv"):
                video_target_path =  folder + '/logging/render_' + os.path.splitext(file)[0] + '.mkv'
                log_target_path =  folder + '/logging/render_' + os.path.splitext(file)[0] + '.txt'
                video_src_path = folder + '/logging/' + file
                if not os.path.isfile(video_target_path) and os.stat(video_src_path).st_size > 30000:
                    cmd = './pats --log ' + folder + '/logging --monitor-render ' + video_src_path + ' 2>&1 | /usr/bin/tee ' + log_target_path
                    execute(cmd,render_process_dir)
                    video_result_path = render_process_dir + '/logging/replay/videoResult.mkv'

                    if os.path.exists(video_result_path):
                        shutil.move(video_result_path, video_target_path)
