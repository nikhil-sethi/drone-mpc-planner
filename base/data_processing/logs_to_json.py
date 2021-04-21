#!/usr/bin/env python3
import os,glob,json,math,argparse,socket,logging,subprocess,shutil
#import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pathlib import Path
import lib_base as lb
from lib_base import datetime_to_str,natural_sort,str_to_datetime
from datetime import datetime, timedelta


version = "1.8"

def process_system_status_in_folder(folder):
    logging.getLogger('logs_to_json').info('Processing status...')
    pats_xml_mode = ''
    pats_xml_path = Path(folder,'logging','pats.xml')
    terminal_log_path = Path(folder,'terminal.log')
    if not os.path.exists(pats_xml_path):
        if os.path.exists(Path(folder,'logging','cam_roll_problem_flag')):
            return ([],'Error: Cam roll angle problem!','')
        if not os.path.exists(terminal_log_path):
            return ([],'Error: xml and terminal log do not exist','')
        else:
            with open (terminal_log_path, "r") as terminal_log:
                log_start = terminal_log.readline()
                if log_start.strip() == 'Resetting cam':
                    return ([],'Resetting cam','')
        line = subprocess.check_output(['tail', '-1', terminal_log_path]).decode("utf8").strip()
        return ([],'Error: xml does not exist. Last terminal line: ' + line,'')
    with open (pats_xml_path, "r") as pats_xml:
            xml_lines = pats_xml.readlines()
            for line in xml_lines:
                if line.find('op_mode') != -1:
                    pats_xml_mode = line.split('_')[3].split('<')[0]
                    break

    results_path = Path(folder,'logging','results.txt')
    if not os.path.exists(results_path):
        return ([],'Error: results.txt does not exist','')
    runtime = -1
    with open (results_path, "r") as results_txt:
            result_lines = results_txt.readlines()
            for line in result_lines:
                if line.find('Run_time') != -1:
                    runtime = float(line.split(':')[1])
                    break
    if runtime<0:
        #something bad must have happened. A crash of the program prevents the writing of results.txt
        return ([],'Error: results.txt does not contain run_time','')

    #open terminal log and check whether we were waiting for darkness
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
        return ([],'Error: ' + log_start,'')
    if log_start == '':
        return ([],'Error: log_start empty!?','')
    if log_start.startswith('Resetting cam'):
        return ([],'Error: ' + log_start,'')

    log_start_datetime = datetime.strptime(log_start.strip(), '%d/%m/%Y %H:%M:%S')
    log_end_datetime = str_to_datetime( os.path.basename(folder))

    log_start = datetime_to_str(log_start_datetime)
    operational_log_start = datetime_to_str(log_end_datetime -  timedelta(seconds=runtime))

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
    logging.getLogger('logs_to_json').info('Processing hunts...')
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

def autocorr(x,lags):
    '''numpy.corrcoef, partial'''

    corr=[1. if l==0 else np.corrcoef(x[l:],x[:-l])[0][1] for l in lags]
    return np.array(corr)

def process_log(detection_fn,folder,mode,monitoring_start_datetime):

    try:
        log = pd.read_csv(detection_fn, sep=";")
    except Exception as e:
        logger.info(detection_fn + ': ' + str(e))
        return {}

    elapsed_time = log["time"].values
    lost = log['n_frames_lost_insect'].values > 0
    remove_ids = [i for i, x in enumerate(lost) if x]
    if len(elapsed_time) < 20 or len(elapsed_time) - len(remove_ids) < 5:
        return {}

    #there can be infs in the velocity columns, due to some weird double frameid occurance from the realsense. #540
    vxs = log['svelX_insect'].values
    vys = log['svelY_insect'].values
    vzs = log['svelZ_insect'].values
    inf_ids = [i for i, x in enumerate(vxs) if math.isinf(x) or math.isnan(x)]
    if (len(inf_ids)):
        remove_ids.extend(inf_ids)

    time = elapsed_time.astype('float64')
    RS_ID = log['RS_ID'].values

    xs = log['sposX_insect'].values
    ys = log['sposY_insect'].values
    zs = log['sposZ_insect'].values
    x_tracking = np.delete(xs,remove_ids).astype('float64')
    y_tracking = np.delete(ys,remove_ids).astype('float64')
    z_tracking = np.delete(zs,remove_ids).astype('float64')

    pos_start = [x_tracking[0],y_tracking[0],z_tracking[0]]
    pos_end =  [x_tracking[-1],y_tracking[-1],z_tracking[-1]]
    diff = np.array(pos_end) - np.array(pos_start)
    dist_traveled = math.sqrt(np.sum(diff*diff))
    dist_traject = np.sum(np.sqrt((x_tracking[1:] - x_tracking[:-1])**2 + (y_tracking[1:] - y_tracking[:-1])**2 + (z_tracking[1:] - z_tracking[:-1])**2))

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

    radiuss = np.delete(log['radius_insect'].values,remove_ids)
    size = np.mean(radiuss)*2

    if 'motion_sum_insect' in log:
        motion_sums = np.delete(log['motion_sum_insect'].values,remove_ids)
        d_motion_sums = motion_sums[1:] - motion_sums[:-1]

        ps = np.abs(np.fft.rfft(d_motion_sums))
        ps_max_id = np.argmax(ps)
        freqs = np.fft.rfftfreq(d_motion_sums.size, 1/90)
        wing_beat = freqs[ps_max_id]
        # idx = np.argsort(freqs)
        # plt.plot(freqs[idx], ps[idx]**2)
        # plt.show()
    else:
        wing_beat = -1

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

    detection_time = datetime_to_str(monitoring_start_datetime + timedelta(seconds=elapsed_time[0]))

    detection_data = {"time" : detection_time,
                    "duration": duration,
                    "RS_ID" : first_RS_ID,
                    "Dist_traveled":dist_traveled,
                    "Dist_traject":dist_traject,
                    "Size": size,
                    "Wing_beat": wing_beat,
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
    return detection_data

def process_detections_in_folder(folder,operational_log_start,mode):

    logger = logging.getLogger('logs_to_json')
    detection_fns = natural_sort([fp for fp in glob.glob(os.path.join(folder, "logging", "log_i*.csv")) if "itrk0" not in fp])
    monitoring_start_datetime = str_to_datetime(operational_log_start)

    valid_detections = []

    for detection_fn in detection_fns:
        logger.info("Processing insects in " + detection_fn)
        detection_data = process_log(detection_fn,folder,mode,monitoring_start_datetime)
        if detection_data:
            valid_detections.append(detection_data)

    return valid_detections

def logs_to_json(start_datetime,end_datetime,json_fn,data_folder,sys_str):

    Path(data_folder + '/processed').mkdir(parents=True, exist_ok=True)
    Path(data_folder + '/junk').mkdir(parents=True, exist_ok=True)
    Path(lb.json_dir).mkdir(parents=True, exist_ok=True)

    found_dirs = glob.glob(data_folder + "/202*_*")
    filtered_dirs = [d for d in found_dirs if str_to_datetime(os.path.basename(os.path.normpath(d))) >= start_datetime and str_to_datetime(os.path.basename(os.path.normpath(d))) <= end_datetime] # filter the list of dirs to only contain dirs between certain dates
    filtered_dirs = natural_sort(filtered_dirs)

    detections = []
    statuss = []
    hunts = []
    errors = []
    cam_resets = 0

    logger = logging.getLogger('logs_to_json')
    for folder in filtered_dirs:
        logger.info("Processing " + folder)

        top_folder = os.path.basename(folder)
        dts = str_to_datetime(top_folder)
        if dts > start_datetime and dts <= end_datetime:

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
            if mode.startswith('Error:'):
                errors.append(mode + '(' + folder + ')')
            if mode.startswith('Resetting cam'):
                cam_resets+=1

            if mode == 'monitoring' or mode == 'hunt' or mode == 'waypoint' or mode == 'deploy':
                Path(folder+ '/OK').touch()
            else:
                Path(folder+ '/junk').touch()

    data_detections = {"from" : lb.datetime_to_str(start_datetime),
            "till" : lb.datetime_to_str(end_datetime),
            "moth_counts" : len(detections),
            "moths" : detections,
            "hunts" : hunts,
            "mode" : statuss,
            "errors" : errors,
            "cam_resets" : cam_resets,
            "system" : sys_str,
            "version" : version}
    with open(json_fn, 'w') as outfile:
        json.dump(data_detections, outfile)
    logger.info("Counting complete, saved in {0}".format(json_fn))

    #move the folders so that we know if they were processed
    #moving is done after writing the json, so that if anything before that fails no data gets lost between the cracks
    for folder in filtered_dirs:
        top_folder = os.path.basename(folder)
        if os.path.exists(folder+ '/OK'):
            processed_folder = folder[0:len(folder)-len(top_folder)] + 'processed/' + top_folder
            shutil.move(folder,processed_folder)
        elif os.path.exists(folder+ '/junk'):
            junk_folder = folder[0:len(folder)-len(top_folder)] + 'junk/' + top_folder
            shutil.move(folder,junk_folder)
    logger.info("Data folders moved")

def process_all_logs_to_jsons():
    now = datetime.today()
    local_json_file = lb.json_dir + datetime_to_str(now) + '.json'
    logs_to_json(datetime.min,now,local_json_file,lb.data_dir,socket.gethostname())

def send_all_jsons():
    logger = logging.getLogger('logs_to_json')
    Path(lb.json_dir + '/sent').mkdir(parents=True, exist_ok=True)
    for json_fn in glob.glob(lb.json_dir + '/*.json'):
        remote_json_file='jsons/' + socket.gethostname() + '_' + os.path.basename(json_fn)
        cmd = 'rsync -puz ' + json_fn +' dash:' + remote_json_file
        if lb.execute(cmd,3,'logs_to_json') == 0:
            json_sent_fn = lb.json_dir + '/sent/' + os.path.basename(json_fn)
            os.rename(json_fn,json_sent_fn)
            logger.info("Json sent: {0}".format(json_fn))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script that counts the number of valid insect detections in a directory, bound by the minimum and maximum date.')
    parser.add_argument('-i', help="Path to the folder with logs", required=False,default=lb.data_dir)
    parser.add_argument('-s', help="Directory date to start from", required=False)
    parser.add_argument('-e', help="Directory date to end on", required=False)
    parser.add_argument('-o',  help="Process just this one log", required=False)
    parser.add_argument('--filename', help="Path and filename to store results in", default="./detections.json")
    parser.add_argument('--system', help="Override system name", default=socket.gethostname())
    args = parser.parse_args()

    logging.basicConfig( )
    logger = logging.getLogger('logs_to_json')
    logger.setLevel(logging.DEBUG)

    json_fn = args.filename
    data_folder = args.i
    sys_str = args.system

    if not args.s and not args.e:
        process_all_logs_to_jsons()
        send_all_jsons()
    elif not args.s:
        logs_to_json(lb.str_to_datetime('20000101_000000'),str_to_datetime(args.e),json_fn,data_folder,sys_str)
    elif not args.e:
        logs_to_json(str_to_datetime(args.s),lb.str_to_datetime('30000101_000000'),json_fn,data_folder,sys_str)
    else:
        logs_to_json(str_to_datetime(args.s),str_to_datetime(args.e),json_fn,data_folder,sys_str)
