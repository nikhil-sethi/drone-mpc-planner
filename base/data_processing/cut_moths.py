#!/usr/bin/env python3


import os
import glob
import argparse
import logging
from pathlib import Path
import lib_base as lb


def system_was_monitoring_in_folder(folder):
    pats_xml_path = Path(folder, 'logging', 'pats.xml')
    if not os.path.exists(pats_xml_path):
        print("Error: pats.xml not found in: " + folder)
        return False
    with open(pats_xml_path, "r") as pats_xml:
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


def cut_moths(folder, dry_run=False):
    logger = logging.getLogger('cut_moths')
    frames_fn = folder + '/logging/frames.csv'
    cut_log_fn = folder + '/logging/cut_moth.log'
    if not os.path.exists(frames_fn):
        print("Frames.csv not found")
        exit()
    with open(frames_fn, 'r') as flog:
        if system_was_monitoring_in_folder(folder):
            files = lb.natural_sort([fp for fp in glob.glob(os.path.join(folder, "logging", "log_itrk*.csv")) if "itrk0" not in fp])
            with open(cut_log_fn, 'w') as clog:
                video_in_file = folder + '/logging/videoRawLR.mkv'
                ffmpeg_cmd_init = 'ffmpeg -y -i ' + video_in_file
                ffmpeg_cmd = ffmpeg_cmd_init
                arg_cnt = 0
                if os.path.exists(video_in_file):
                    clog.write('Video file size: ' + str(os.path.getsize(video_in_file) / 1024 / 1024 / 1024) + 'GB')
                    for file in files:
                        logger.info('Processing ' + file)
                        clog.write(os.path.basename(file) + ': ')
                        with open(file, 'r') as ilog:
                            lines = ilog.read().splitlines()
                            fn = os.path.basename(file)
                            file_id = fn.split('.')[0][8:]
                            if len(lines) > 10:  # only cut a video if it is of substantial length...
                                heads = lines[0].split(';')
                                if len(lines[-1].split(';')) != len(heads):  # sometimes the last line was not completely finished.
                                    del lines[-1]
                                fp = lines[-1].split(';')[heads.index('fp')]
                                if fp == 'fp_not_a_fp':
                                    video_start_rs_id = int(lines[1].split(';')[0])
                                    while True:
                                        fline = flog.readline()
                                        video_start_time = -1
                                        video_start_video_id = -1
                                        if not fline or len(fline.split(';')) != 4:
                                            clog.write('Error: could not find frame in frames.csv\n')
                                            print('Error: could not find frame in frames.csv')
                                            break
                                        if int(fline.split(';')[2]) == video_start_rs_id:
                                            video_start_video_id = int(fline.split(';')[0])
                                            video_start_time = float(video_start_video_id) / 90
                                            break
                                        if int(fline.split(';')[2]) > video_start_rs_id:
                                            clog.write('Warning: frame id already passed. Skipping this one because lazy.\n')
                                            print('Warning: frame id already passed. Skipping this one because lazy.')
                                            break

                                    if video_start_time < 0:
                                        continue
                                    video_start_time = video_start_time - 1
                                    if (video_start_time < 0):
                                        video_start_time = 0
                                    video_end_rs_id = int(lines[-2].split(';')[0])
                                    video_end_video_id = video_end_rs_id - (video_start_rs_id - video_start_video_id)
                                    video_end_time = float(video_end_video_id) / 90 + 1
                                    video_duration = video_end_time - video_start_time
                                    log_folder = folder + '/logging/'
                                    video_out_file = log_folder + '/insect' + file_id + '.mkv'
                                    video_out_file = video_out_file.replace(os.path.expanduser('~'), '~')
                                    video_out_file = video_out_file.replace('//', '/')
                                    cmd = ' -c:v copy -an -ss ' + str(round(video_start_time, 1)) + ' -t ' + str(round(video_duration, 1)) + ' ' + video_out_file
                                    ffmpeg_cmd += cmd
                                    clog.write(cmd + '\n')
                                    arg_cnt += 1
                                    if arg_cnt > 1000:  # to prevent a too large argument list, wrap every 1000 video's
                                        execute_cut(ffmpeg_cmd, dry_run)
                                        arg_cnt = 0
                                        ffmpeg_cmd = ffmpeg_cmd_init

                                else:
                                    clog.write(fp + '\n')
                            else:
                                clog.write('Not enough lines!\n')

                    execute_cut(ffmpeg_cmd, dry_run)
                    if not dry_run:
                        os.remove(video_in_file)

                else:
                    print("VideoRawLR not found")


def execute_cut(ffmpeg_cmd, dry_run):
    logger = logging.getLogger('cut_moths')
    logger.info(ffmpeg_cmd)
    if not dry_run:
        lb.execute(ffmpeg_cmd, 1, 'cut_moths')


def cut_moths_all(dry_run=False):
    logger = logging.getLogger('cut_moths')
    found_dirs = glob.glob(lb.data_dir + '/202*_*')
    for folder in found_dirs:
        try:
            video_in_file = folder + '/logging/videoRawLR.mkv'
            if os.path.exists(video_in_file):
                logger.info('Processing ' + folder)
                cut_moths(folder, dry_run)
            else:
                logger.info('Skipping ' + folder + ' because no videoRawLR.mkv')
        except Exception as e:
            logger.error('Error in ' + folder + '; ' + str(e))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script that cuts the raw video into insects based on the log')
    parser.add_argument('-i', help="Path to the folder. If not provided all folders of the past day will be ran", required=False)
    parser.add_argument('--dry-run', help="Just run, don't cut or delete anything", dest='dry_run', action='store_true', required=False)
    args = parser.parse_args()

    logging.basicConfig()
    logger = logging.getLogger('cut_moths')
    logger.setLevel(logging.DEBUG)

    if not args.i:
        cut_moths_all(args.dry_run)
    else:
        cut_moths(args.i, args.dry_run)
