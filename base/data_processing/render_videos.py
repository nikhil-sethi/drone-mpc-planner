#!/usr/bin/env python3

import os
import socket
import glob
import argparse
import shutil
import logging
from datetime import datetime, timedelta
from pathlib import Path
import lib_base as lb


def render_last_day(data_dir=lb.data_dir, abort_deadline=datetime.strptime('30000101_100000', '%Y%m%d_%H%M%S')):
    now = datetime.today()
    yesterday = now - timedelta(days=1)
    render(yesterday, now, data_dir, abort_deadline)


def check_if_hunt_mode(pats_xml_path):
    with open(pats_xml_path, "r") as pats_xml:
        xml_lines = pats_xml.readlines()
        for line in xml_lines:
            if line.find('op_mode_hunt') != -1 or line.find('op_mode_deploy') != -1:
                return True
    return False


def read_results_txt(results_txt_path):
    n_hunts = 0
    n_takeoffs = 0
    with open(results_txt_path, "r") as results_txt:
        results_lines = results_txt.readlines()
        for line in results_lines:
            if line.find('n_hunts') != -1:
                n_hunts = int(line.split(':')[1])
            if line.find('n_takeoffs') != -1:
                n_takeoffs = int(line.split(':')[1])
    return n_hunts, n_takeoffs


def check_for_hunt_render(pats_xml_path, results_txt_path, target_path):
    if not os.path.exists(target_path):
        hunt_mode = False
        n_hunts = 0
        n_takeoffs = 0
        hunt_mode = check_if_hunt_mode(pats_xml_path)
        n_hunts, n_takeoffs = read_results_txt(results_txt_path)
        if hunt_mode and n_hunts > 0 and n_takeoffs > 0:
            return True
    else:
        return False


def create_render_lists(data_folder, start_datetime, end_datetime):
    logger = logging.getLogger('render')

    found_dirs = glob.glob(os.path.expanduser(data_folder) + "processed/202*_*")
    filtered_dirs = [d for d in found_dirs if lb.str_to_datetime(os.path.basename(os.path.normpath(d))) >= start_datetime and lb.str_to_datetime(os.path.basename(os.path.normpath(d))) <= end_datetime]  # filter the list of dirs to only contain dirs between certain dates

    monitor_detections = []
    hunt_detections = []
    for folder in filtered_dirs:
        logger.info(f"Preprocessing folder: {folder}")
        pats_xml_path = Path(folder, 'logging', 'pats.xml')
        results_txt_path = Path(folder, 'logging', 'results.txt')

        if os.path.exists(pats_xml_path) and os.path.exists(results_txt_path):

            # check hunting render:
            target_path = Path(Path(lb.renders_dir), os.path.basename(folder) + '_' + socket.gethostname() + '.mkv')
            if check_for_hunt_render(pats_xml_path, results_txt_path, target_path):
                hunt_detections.append({'folder': folder, 'target_path': target_path})

            # check monitoring renders:
            files = os.listdir(folder + '/logging/')
            for file in files:
                if file.startswith('moth') and file.endswith(".mkv"):
                    video_target_path = folder + '/logging/render_' + os.path.splitext(file)[0] + '.mkv'
                    log_target_path = folder + '/logging/render_' + os.path.splitext(file)[0] + '.txt'
                    video_src_path = folder + '/logging/' + file
                    tag_path = folder + '/logging/' + file.replace('.mkv', '.render_tag')  # the render tag is set in logs_to_json using similar filtering as what PATS-C is supposed to use
                    if not os.path.isfile(video_target_path) and os.path.isfile(tag_path):
                        file_to_be_rendered = {
                            "folder": folder,
                            "video_target_path": video_target_path,
                            "log_target_path": log_target_path,
                            "video_src_path": video_src_path
                        }
                        monitor_detections.append(file_to_be_rendered)

    return monitor_detections, hunt_detections


def render(start_datetime, end_datetime, data_folder, abort_deadline=datetime.strptime('30000101_100000', '%Y%m%d_%H%M%S')):
    logger = logging.getLogger('render')

    monitor_detections, hunt_detections = create_render_lists(data_folder, start_datetime, end_datetime)

    if not len(monitor_detections) + len(hunt_detections):
        return
    logger.info(f"Number of hunt renders: {str(len(hunt_detections))}")
    logger.info(f"Number of monitor renders: {str(len(monitor_detections))}")

    original_process_dir = os.path.expanduser('~/code/pats/base/build/')
    render_process_dir = os.path.expanduser('~/code/pats/base/build_render/')
    if os.path.exists(render_process_dir):
        shutil.rmtree(render_process_dir)
    shutil.copytree(original_process_dir, render_process_dir, ignore=shutil.ignore_patterns('*logging*'))
    shutil.move(render_process_dir + '/pats', render_process_dir + '/pats_render')

    for detection in hunt_detections:
        if datetime.now() > abort_deadline:
            logger.warning('Time for rendering exceeded. Aborting rendering prematurely.')
            return

        logger.info(f"Rendering hunt {detection['folder']}")
        cmd = './pats_render --log ' + detection['folder'] + '/logging --render'
        lb.execute(cmd, logger_name='render', render_process_dir=render_process_dir)
        video_result_path = Path(render_process_dir, 'logging/replay/videoResult.mkv')
        if os.path.exists(video_result_path):
            Path(lb.renders_dir).mkdir(parents=True, exist_ok=True)
            shutil.copyfile(video_result_path, detection['target_path'])
            shutil.move(str(video_result_path), Path(detection['folder'], 'videoResult.mkv'))
        else:
            logger.error(f"hunt render not found after rendering {detection['folder']}")

    render_cnt = 0
    for detection in monitor_detections:
        if datetime.now() > abort_deadline:
            logger.warning('Time for rendering exceeded. Aborting rendering prematurely.')
            return

        render_cnt += 1
        logger.info(f"{str(render_cnt)} / {str(len(monitor_detections))}. Rendering {detection['video_src_path']}.")
        cmd = './pats_render --log ' + detection['folder'] + '/logging --monitor-render ' + detection['video_src_path'] + ' 2>&1 | /usr/bin/tee ' + detection['log_target_path']
        render_ok = False
        video_src_path = detection['video_src_path']
        video_target_path = detection['video_target_path']
        for i in range(1, 5):
            if lb.execute(cmd, render_process_dir=render_process_dir) == 0:
                render_ok = True
                break
            logger.warning('Render attempt ' + str(i) + f' failed for {video_src_path}')

        video_result_path = render_process_dir + '/logging/replay/videoResult.mkv'
        results_txt_path = Path(render_process_dir, 'logging', 'replay', 'results.txt')

        if not render_ok:
            logger.error(f'Render failed for {video_src_path}')
        elif not os.path.exists(video_result_path):
            logger.error(f'Render missing...? {video_src_path}')
        elif not os.path.exists(results_txt_path):
            logger.error(f'Results.txt missing...? {video_src_path}')
        else:
            with open(results_txt_path, "r") as results_txt:
                results_lines = results_txt.readlines()
                n_insects = -1
                for line in results_lines:
                    if line.find('n_insects') != -1:
                        n_insects = int(line.split(':')[1])
                        break
                if n_insects > 0:
                    shutil.move(video_result_path, video_target_path)
                else:
                    logger.error(f'Rendered empty video...? {video_src_path}')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script that renders video result in all pats log folders in a directory, bound by the minimum and maximum date\n\
        example: ./render_video.py -i "{path_directory_of_log_dirs}" -s "20200729_190000" -e "20200729_230000"')

    parser.add_argument('-i', help="Path to the folder with logs", required=False, default=lb.data_dir)
    parser.add_argument('-s', help="Directory date to start from", required=False)
    parser.add_argument('-e', help="Directory date to end on", required=False)
    args = parser.parse_args()

    logging.basicConfig()
    logger = logging.getLogger('render')
    logger.setLevel(logging.DEBUG)

    if not args.s and not args.e:
        render_last_day(args.i, abort_deadline=datetime.now() + timedelta(hours=3))
    elif not args.s:
        render(lb.str_to_datetime('20000101_000000'), args.e, args.i)
    elif not args.e:
        render(lb.str_to_datetime(args.s), lb.str_to_datetime('30000101_000000'), args.i)
    else:
        render(lb.str_to_datetime(args.s), lb.str_to_datetime(args.e), args.i)
