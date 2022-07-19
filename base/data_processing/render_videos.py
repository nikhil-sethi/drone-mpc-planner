#!/usr/bin/env python3

import os
import glob
import argparse
import shutil
import logging
import logging.handlers
from datetime import datetime, timedelta
from pathlib import Path
import lib_base as lb


def render_last_day(data_dir=lb.data_dir, abort_deadline=datetime.strptime('30000101_100000', '%Y%m%d_%H%M%S')):
    now = datetime.today()
    yesterday = now - timedelta(days=1)
    render_all(yesterday, now, data_dir, abort_deadline)


def create_render_lists(data_folder, start_datetime, end_datetime):
    logger = logging.getLogger('render')

    found_dirs = glob.glob(os.path.expanduser(data_folder) + "processed/202*_*")
    filtered_dirs = [d for d in found_dirs if lb.str_to_datetime(os.path.basename(os.path.normpath(d))) >= start_datetime and lb.str_to_datetime(os.path.basename(os.path.normpath(d))) <= end_datetime]  # filter the list of dirs to only contain dirs between certain dates

    detections = []
    flights = []
    for folder in filtered_dirs:
        logger.info("Preprocessing folder: " + folder)
        pats_xml_path = Path(folder, 'pats.xml')
        results_txt_path = Path(folder, 'results.txt')

        if os.path.exists(pats_xml_path) and os.path.exists(results_txt_path):

            files = os.listdir(folder)
            for file in files:
                if (file.startswith('insect') or file.startswith('anomoly')) and file.endswith(".mkv"):
                    video_target_path = folder + '/render_' + os.path.splitext(file)[0] + '.mkv'
                    log_target_path = folder + '/render_' + os.path.splitext(file)[0] + '.txt'
                    video_src_path = folder + '/' + file
                    file_to_be_rendered = {
                        "folder": folder,
                        "video_target_path": video_target_path,
                        "log_target_path": log_target_path,
                        "video_src_path": video_src_path
                    }
                    detections.append(file_to_be_rendered)
                if file.startswith('flight') and file.endswith(".mkv"):
                    video_target_path = folder + '/render_' + os.path.splitext(file)[0] + '.mkv'
                    log_target_path = folder + '/render_' + os.path.splitext(file)[0] + '.txt'
                    video_src_path = folder + '/' + file

                    if not os.path.isfile(video_target_path):
                        file_to_be_rendered = {
                            "folder": folder,
                            "video_target_path": video_target_path,
                            "log_target_path": log_target_path,
                            "video_src_path": video_src_path
                        }
                        flights.append(file_to_be_rendered)

    return detections, flights


def render(detection, render_process_dir, render_mode):
    logger = logging.getLogger('render')

    cmd = './executor_render --log ' + detection['folder'] + ' --' + render_mode + ' ' + detection['video_src_path'] + ' --render' + ' 2>&1'
    render_ok = False
    video_src_path = detection['video_src_path']
    video_target_path = detection['video_target_path']
    for i in range(1, 5):
        if lb.execute(cmd, render_process_dir=render_process_dir, raw_log_file=detection['log_target_path']) == 0:
            if os.stat((detection['video_src_path'])).st_size > 30000:  # another work around for #833
                render_ok = True
                break
        logger.warning('Render attempt ' + str(i) + f' failed for {video_src_path}')

    video_render_path = render_process_dir + '/logging/replay/videoRender.mkv'
    results_txt_path = Path(render_process_dir, 'logging', 'replay', 'results.txt')

    if not render_ok:
        logger.error('Render failed for ' + video_src_path)
    elif not os.path.exists(video_render_path):
        logger.error('Render missing...? ' + video_src_path)
    elif not os.path.exists(results_txt_path):
        logger.error('Results.txt missing...? ' + video_src_path)
    else:
        with open(results_txt_path, "r", encoding="utf-8") as results_txt:
            results_lines = results_txt.readlines()
            n_insects = -1
            n_monsters = -1
            n_takeoffs = -1
            for line in results_lines:
                if line.find('n_insects') != -1:
                    n_insects = int(line.split(':')[1])
                if line.find('n_monsters') != -1:
                    n_monsters = int(line.split(':')[1])
                if line.find('n_takeoffs') != -1:
                    n_monsters = int(line.split(':')[1])

            shutil.move(video_render_path, video_target_path)
            if n_insects == 0 and n_monsters == 0 and n_takeoffs == 0:
                logger.warning('Rendered empty video...? ' + video_src_path)


def render_all(start_datetime, end_datetime, data_folder, abort_deadline=datetime.strptime('30000101_100000', '%Y%m%d_%H%M%S')):
    logger = logging.getLogger('render')

    monitor_detections, flights = create_render_lists(data_folder, start_datetime, end_datetime)

    if not len(monitor_detections) + len(flights):
        return
    logger.info("Number of flight renders: " + str(len(flights)))
    logger.info("Number of monitor renders: " + str(len(monitor_detections)))

    original_process_dir = os.path.expanduser('~/code/pats/base/build/')
    render_process_dir = os.path.expanduser('~/code/pats/base/build-render/')
    if os.path.exists(render_process_dir):
        shutil.rmtree(render_process_dir)
    shutil.copytree(original_process_dir, render_process_dir, ignore=shutil.ignore_patterns('*logging*'))
    shutil.move(render_process_dir + '/executor', render_process_dir + '/executor_render')

    render_cnt = 0
    tot_render_cnt = len(monitor_detections) + len(flights)
    for detection in monitor_detections:
        if datetime.now() > abort_deadline:
            logger.warning('Time for rendering exceeded. Aborting rendering prematurely.')
            return
        render_cnt += 1
        logger.info(str(render_cnt) + ' / ' + str(tot_render_cnt) + '. Rendering ' + detection['video_src_path'] + '.')
        render(detection, render_process_dir, 'insect')
    for flight in flights:
        if datetime.now() > abort_deadline:
            logger.warning('Time for rendering exceeded. Aborting rendering prematurely.')
            return
        render_cnt += 1
        logger.info(str(render_cnt) + ' / ' + str(tot_render_cnt) + '. Rendering ' + flight['video_src_path'] + '.')
        render(flight, render_process_dir, 'flight')


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

    file_format = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
    fh = logging.handlers.RotatingFileHandler(filename=lb.log_dir + '/render.log', maxBytes=1024 * 1024 * 100, backupCount=1)
    fh.setFormatter(file_format)
    fh.level = logging.DEBUG
    logger.addHandler(fh)
    logger.setLevel(logging.DEBUG)
    logger.info(' reporting in!')

    if not args.s and not args.e:
        render_last_day(args.i, abort_deadline=datetime.now() + timedelta(hours=3))
    elif not args.s:
        render_all(lb.str_to_datetime('20000101_000000'), args.e, args.i)
    elif not args.e:
        render_all(lb.str_to_datetime(args.s), lb.str_to_datetime('30000101_000000'), args.i)
    else:
        render_all(lb.str_to_datetime(args.s), lb.str_to_datetime(args.e), args.i)
