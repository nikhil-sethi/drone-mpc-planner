#!/usr/bin/env python3
import os, glob, json,math,re,datetime, argparse, socket, pickle
import numpy as np
import pandas as pd
pd.options.mode.chained_assignment = None
from tqdm import tqdm
from pathlib import Path

version = "1.6"

from scipy.interpolate import interp1d
from sklearn.base import BaseEstimator, TransformerMixin
from sklearn.linear_model import LinearRegression
from sklearn.pipeline import Pipeline

def natural_sort(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key=alphanum_key)

def todatetime(string):
    return datetime.datetime.strptime(string, "%Y%m%d_%H%M%S")

def process_system_status_in_folder(folder):
    pats_xml_mode = ''
    pats_xml_path = Path(folder,'logging','pats.xml')
    if not os.path.exists(pats_xml_path):
        return ([],'error: xml does not exist','')
    with open (pats_xml_path, "r") as pats_xml:
            xml_lines = pats_xml.readlines()
            for line in xml_lines:
                if line.find('op_mode') != -1:
                    pats_xml_mode = line.split('_')[3].split('<')[0]
                    break

    results_path = Path(folder,'logging','results.txt')
    if not os.path.exists(results_path):
        return ([],'error results.txt does not exist','')
    runtime = -1
    with open (results_path, "r") as results_txt:
            result_lines = results_txt.readlines()
            for line in result_lines:
                if line.find('Run_time') != -1:
                    runtime = float(line.split(':')[1])
                    break

    #open terminal log and check whether we were waiting for darkness
    terminal_log_path = Path(folder,'terminal.log')
    log_start = ''
    daylight_start = ''
    daylight_end = ''
    prev_line = ''
    with open (terminal_log_path, "r") as terminal_log:
        log_start = terminal_log.readline()
        while not daylight_end:
            line = terminal_log.readline()
            if 'Measured exposure' in line:
                daylight_start = line.split('Measured')[0]
                prev_line = line
                while True:
                    line = terminal_log.readline()
                    if not 'Measured exposure' in line:
                        daylight_end = prev_line.split('Measured')[0]
                        break
                    prev_line = line
            if line == '' and prev_line == '':
                daylight_start = log_start
                daylight_end = log_start
                break

    daylight_start = daylight_start.strip().replace('/','').replace(':','').replace(' ', '_')
    daylight_end = daylight_end.strip().replace('/','').replace(':','').replace(' ', '_')

    if log_start.startswith('Error'):
        return ([],'error: ' + log_start,'')
    if 'Resetting cam' in log_start or log_start == '':
        return ([],'error: log_start: ' + log_start,'')
    log_start_datetime = datetime.datetime.strptime(log_start.strip(), '%d/%m/%Y %H:%M:%S')
    log_end_datetime = datetime.datetime.strptime( os.path.basename(folder), '%Y%m%d_%H%M%S')

    log_start = log_start_datetime.strftime('%Y%m%d_%H%M%S')
    operational_log_start = (log_end_datetime -  datetime.timedelta(seconds=runtime)).strftime('%Y%m%d_%H%M%S')

    if daylight_start != daylight_end:
            data_status1 = {"from" : daylight_start,
            "till" : daylight_end,
            "mode" : 'wait_for_dark'
            }
            data_status2 = {"from" : operational_log_start,
            "till" : os.path.basename(folder),
            "mode" : pats_xml_mode
            }
            return ([data_status1, data_status2],pats_xml_mode,operational_log_start)
    else:
        data_status = {"from" : operational_log_start,
        "till" : os.path.basename(folder),
        "mode" : pats_xml_mode
        }
        return (data_status,pats_xml_mode,operational_log_start)

def process_hunts_in_folder(folder,operational_log_start):
    results_path = Path(folder,'logging','results.txt')
    if not os.path.exists(results_path):
        return []
    drone_flights = 0
    n_drone_detects = 0
    n_insects = 0
    n_takeoffs = 0
    n_landings = 0
    n_hunts = 0
    n_replay_hunts = 0
    best_interception_distance = -1
    drone_problem = 0
    with open (results_path, "r") as results_txt:
            result_lines = results_txt.readlines()
            for line in result_lines:
                if line.find('n_drone_detects') != -1:
                    n_drone_detects = int(line.strip().split(':')[1])
                if line.find('n_insects') != -1:
                    n_insects = int(line.strip().split(':')[1])
                if line.find('n_takeoffs') != -1:
                    n_takeoffs = int(line.strip().split(':')[1])
                if line.find('n_landings') != -1:
                    n_landings = int(line.strip().split(':')[1])
                if line.find('n_hunts') != -1:
                    n_hunts = int(line.strip().split(':')[1])
                if line.find('n_replay_hunts') != -1:
                    n_replay_hunts = int(line.strip().split(':')[1])
                if line.find('best_interception_distance') != -1:
                    best_interception_distance = float(line.strip().split(':')[1])
                if line.find('drone problem') != -1 or line.find('drone_problem') != -1 :
                    drone_problem = int(line.strip().split(':')[1])

    data_hunt = {"from" : operational_log_start,
    "till" : os.path.basename(folder),
    "drone_flights" : drone_flights,
    "n_drone_detects" : n_drone_detects,
    "n_insects" : n_insects,
    "n_takeoffs" : n_takeoffs,
    "n_landings" : n_landings,
    "n_hunts" : n_hunts,
    "n_replay_hunts" : n_replay_hunts,
    "best_interception_distance" : best_interception_distance,
    "drone_problem" : drone_problem
    }

    return data_hunt


def process_detections_in_folder(folder,operational_log_start,mode):

    #concat all csv files containing dates
    detection_fns = natural_sort([fp for fp in glob.glob(os.path.join(folder, "logging", "log_i*.csv")) if "itrk0" not in fp])
    monitoring_start_datetime = datetime.datetime.strptime(operational_log_start,'%Y%m%d_%H%M%S')

    valid_detections = []
    prev_RS_ID = -1
    pbar_detections = tqdm(detection_fns, leave=False)
    for detection_fn in pbar_detections:
        with open (detection_fn, "r") as detection_log:
            try:
                log = pd.read_csv(detection_fn, sep=";")
            except Exception as e:
                print(detection_fn + ': ' + str(e))
                continue

        elapsed_time = log["time"].values
        lost = log['n_frames_lost_insect'].values > 0
        remove_ids = [i for i, x in enumerate(lost) if x]
        if len(elapsed_time) < 20 or len(elapsed_time) - len(remove_ids) < 5:
            continue

        #there can be infs in the velocity columns, due to some weird double frameid occurance from the realsense. #540
        vxs = log['svelX_insect'].values
        vys = log['svelY_insect'].values
        vzs = log['svelZ_insect'].values
        inf_ids = [i for i, x in enumerate(vxs) if math.isinf(x)]
        if (len(inf_ids)):
            remove_ids.extend(inf_ids)

        time = elapsed_time.astype('float64')
        RS_ID = log['RS_ID'].values

        xs = log['sposX_insect'].values
        ys = log['sposY_insect'].values
        zs = log['sposZ_insect'].values
        x_tracking = np.delete(xs,remove_ids)
        y_tracking = np.delete(ys,remove_ids)
        z_tracking = np.delete(zs,remove_ids)

        pos_start = [x_tracking[0],y_tracking[0],z_tracking[0]]
        pos_end =  [x_tracking[-1],y_tracking[-1],z_tracking[-1]]
        diff = np.array(pos_end) - np.array(pos_start)
        dist_traveled = math.sqrt(np.sum(diff*diff))
        dist_traject = math.sqrt(np.sum((x_tracking[1:] - x_tracking[:-1])**2 + (y_tracking[1:] - y_tracking[:-1])**2 + (z_tracking[1:] - z_tracking[:-1])**2))

        mid_id = int(round(len(x_tracking)/2))
        pos_middle =  [x_tracking[mid_id],y_tracking[mid_id],z_tracking[mid_id]]
        alpha_horizontal_start = math.atan2(pos_middle[0] - pos_start[0],pos_middle[2] - pos_start[2])
        alpha_horizontal_end = math.atan2(pos_end[0] - pos_middle[0],pos_end[2] - pos_middle[2])
        alpha_vertical_start = math.atan2(pos_middle[0] - pos_start[0],pos_middle[1] - pos_start[1])
        alpha_vertical_end = math.atan2(pos_end[0] - pos_middle[0],pos_end[1] - pos_middle[1])

        vx_tracking = np.delete(vxs,remove_ids)
        vy_tracking = np.delete(vys,remove_ids)
        vz_tracking = np.delete(vzs,remove_ids)
        v = np.sqrt(vx_tracking**2+vy_tracking**2+vz_tracking**2)
        v_mean = v.mean()
        v_std = v.std()

        if math.isinf(v_mean) or math.isinf(v_mean):
            print(v_mean)

        sizes = np.delete(log['radius_insect'].values,remove_ids)
        size = np.mean(sizes)*2

        filtered_elepased = np.delete(elapsed_time,remove_ids)
        start = filtered_elepased[0]
        end = filtered_elepased[-1]
        duration = end - start
        first_RS_ID = str(RS_ID[0])
        filename = os.path.basename(detection_fn)
        video_filename = os.path.dirname(detection_fn) + '/' + filename.replace('log_itrk','moth').replace('csv','mkv')
        if not os.path.exists(video_filename):
            video_filename = 'NA'
        else:
            video_filename = os.path.basename(video_filename)

        detection_time = (monitoring_start_datetime + datetime.timedelta(seconds=elapsed_time[0])).strftime('%Y%m%d_%H%M%S')

        if args.v:
            print(filename)
            print(f'RS_ID: {RS_ID[0]} - {RS_ID[-1]} - {prev_RS_ID}')
            print(f'Insect flight length: {"{:.2f}".format(duration)} s')
            fig = plt.figure()
            ax = fig.gca(projection='3d')
            ax.plot(xs, -ys, zs, label='Insect flight')
            ax.scatter([-5], [5], [-5], label='anchor', marker="o")
            ax.legend()
            ax.set_xlabel('X axis')
            ax.set_ylabel('Y axis')
            ax.set_zlabel('Z axis')
            plt.show()

        detection_data = {"time" : detection_time,
                        "duration": duration,
                        "RS_ID" : first_RS_ID,
                        "Dist_traveled":dist_traveled,
                        "Dist_traject":dist_traject,
                        "Size": size,
                        "Vel_mean" : v_mean,
                        "Vel_std" : v_std,
                        "Vel_max" : v.max(),
                        "Alpha_horizontal_start": alpha_horizontal_start,
                        "Alpha_horizontal_end": alpha_horizontal_end,
                        "Alpha_vertical_start": alpha_vertical_start,
                        "Alpha_vertical_end": alpha_vertical_end,
                        "Filename" : filename,
                        "Folder" : os.path.basename(folder),
                        "Video_Filename" : video_filename,
                        "Mode" : mode,
                        "Version": version
                    }
        valid_detections.append(detection_data)

        prev_RS_ID = first_RS_ID

    return valid_detections

parser = argparse.ArgumentParser(description='Script that counts the number of valid insect detections in a directory, bound by the minimum and maximum date.')
parser.add_argument('-i', help="Path to the folder with logs", required=True)
parser.add_argument('-s', help="Directory date to start from", default="20000101_000000", required=False)
parser.add_argument('-e', help="Directory date to end on", default="30000101_000000", required=False)
parser.add_argument('-v', help="View the path of the found insect flights in a plot", required=False, default=False, action='store_true')
parser.add_argument('--filename', help="Path and filename to store results in", default="./detections.json")
parser.add_argument('--system', help="Override system name", default=socket.gethostname())
args = parser.parse_args()

min_date = todatetime(args.s)
max_date = todatetime(args.e)

if args.v:
    import matplotlib as mpl
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
    mpl.rcParams['legend.fontsize'] = 10

found_dirs = glob.glob(args.i + "/202*_*")
filtered_dirs = [d for d in found_dirs if todatetime(os.path.basename(os.path.normpath(d))) >= min_date and todatetime(os.path.basename(os.path.normpath(d))) <= max_date] # filter the list of dirs to only contain dirs between certain dates

detections = []
statuss = []
hunts = []
errors = []

pbar = tqdm(filtered_dirs,leave=False)
for folder in pbar:
    pbar.set_description(folder)

    top_folder = os.path.basename(folder)
    dts = datetime.datetime.strptime(top_folder,'%Y%m%d_%H%M%S')
    if dts > min_date and dts <= max_date:

        status_in_folder,mode,operational_log_start = process_system_status_in_folder(folder)
        if status_in_folder != []:
            statuss.append(status_in_folder)
        if mode == 'monitoring' or mode == 'hunt':
            detections_in_folder = process_detections_in_folder(folder,operational_log_start,mode)
            if detections_in_folder != []:
                [detections.append(detection) for detection in detections_in_folder]
        if mode == 'hunt' or mode == 'deploy':
            hunts_in_folder = process_hunts_in_folder(folder,operational_log_start)
            if hunts_in_folder != []:
                hunts.append(hunts_in_folder)
        if mode.startswith('error'):
            errors.append(mode + '(' + folder + ')')

data_detections = {"from" : args.s,
        "till" : args.e,
        "moth_counts" : len(detections),
        "moths" : detections,
        "hunts" : hunts,
        "mode" : statuss,
        "errors" : errors,
        "system" : args.system,
        "version" : version}
with open(args.filename, 'w') as outfile:
    json.dump(data_detections, outfile)
print("Counting complete, saved in {0}".format(args.filename))
