#!/usr/bin/env python3


import os
import glob
import argparse
import logging
import lib_base as lb


def cut_files(log_folder, files, logger, video_in_fn, frames_fn, cut_log_fn, dry_run=False):
    with open(frames_fn, 'r', encoding="utf-8") as frames_log:
        heads = frames_log.readline()
        with open(cut_log_fn, 'w', encoding="utf-8") as cut_log:
            ffmpeg_cmd_init = 'ffmpeg -y -i ' + video_in_fn
            ffmpeg_cmd = ffmpeg_cmd_init
            arg_cnt = 0
            if os.path.exists(video_in_fn):
                cut_log.write('Video file size: ' + str(os.path.getsize(video_in_fn) / 1024 / 1024 / 1024) + 'GB\n')
                key_frame_ids = []
                for file in files:
                    logger.info('Processing ' + file)
                    cut_log.write(os.path.basename(file) + ': ')
                    with open(file, 'r', encoding="utf-8") as tracking_log:
                        lines = tracking_log.read().splitlines()
                        fn = os.path.basename(file)
                        if len(lines) > 10:  # only cut a video if it is of substantial length...
                            heads = lines[0].split(';')
                            if len(lines[-1].split(';')) != len(heads):  # sometimes the last line was not completely finished.
                                del lines[-1]
                            flight_log = False
                            monitor_log = False
                            if 'imLx_drone' in heads:
                                flight_log = True
                                file_id = fn.split('.')[0][10:]
                            elif 'fp' in heads:
                                file_id = fn.split('.')[0][8:]
                                fp = lines[-1].split(';')[heads.index('fp')]
                                hunt_id = int(lines[-1].split(';')[heads.index('hunt_id')])
                                monitor_log = fp == ('fp_not_a_fp' or fp == 'fp_too_big' or fp == 'fp_too_far') and hunt_id < 0

                            if monitor_log:

                                video_start_offset = 1
                            elif flight_log:
                                video_start_offset = 5
                            if flight_log or monitor_log:
                                video_start_rs_id = int(lines[1].split(';')[heads.index('rs_id')])

                                while True:
                                    frame_line = frames_log.readline()
                                    video_start_time = -1
                                    video_start_video_id = -1
                                    splitted_frame_line = frame_line.split(';')
                                    if not frame_line or len(splitted_frame_line) != 4:
                                        cut_log.write('Error: could not find frame in frames.csv\n')
                                        print('Error: could not find frame in frames.csv')
                                        break

                                    if (int(splitted_frame_line[0])) % 30 == 0:  # assuming a keyframe every 30 frames
                                        key_frame_ids.append([int(splitted_frame_line[0]), int(splitted_frame_line[2])])

                                    if int(splitted_frame_line[2]) == video_start_rs_id:
                                        ii = (video_start_offset * int(90 / 30))
                                        if ii >= len(key_frame_ids):
                                            ii = len(key_frame_ids)
                                        keyframe_rs_id = key_frame_ids[-ii][1]
                                        video_start_video_id = key_frame_ids[-ii][0]  # instead of using exactly int(splitted_frame_line[0]), the way the video encoding works is that we can only cut from the last keyframe
                                        video_start_time = float(video_start_video_id - 1) / 90  # for some reason ffmpeg wants a time instead of a frame id. And it needs to be one frame before the actual key frame...
                                        break
                                    if int(splitted_frame_line[2]) > video_start_rs_id:
                                        cut_log.write('Warning: frame id already passed. Skipping this one because lazy.\n')
                                        print('Warning: frame id already passed. Skipping this one because lazy.')
                                        break

                                if video_start_time < 0:
                                    continue

                                video_duration = float(lines[-2].split(';')[heads.index('elapsed')]) - float(lines[1].split(';')[heads.index('elapsed')]) + video_start_offset
                                if monitor_log:
                                    video_out_file = log_folder + '/insect' + file_id + '.mkv'
                                else:
                                    video_out_file = log_folder + '/flight' + file_id + '.mkv'
                                    with open(log_folder + '/flight' + file_id + '.txt', 'w', encoding="utf-8") as flight_start_rs_id_file:
                                        flight_start_rs_id_file.write(str(keyframe_rs_id))
                                        flight_start_rs_id_file.write('\nThis file is auto generated from cut_moths.py. It contains the rs_id of a few keyframes (video encoding, every 30 frames by default) before the actual start of this flight/insect log.\n')
                                video_out_file = video_out_file.replace(os.path.expanduser('~'), '~')
                                video_out_file = video_out_file.replace('//', '/')
                                cmd = ' -c:v copy -an -ss ' + str(round(video_start_time, 2)) + ' -t ' + str(round(video_duration, 2)) + ' ' + video_out_file
                                ffmpeg_cmd += cmd
                                cut_log.write(cmd + '\n')
                                arg_cnt += 1
                                if arg_cnt > 1000:  # to prevent a too large argument list, wrap every 1000 video's
                                    execute_cut(ffmpeg_cmd, dry_run)
                                    arg_cnt = 0
                                    ffmpeg_cmd = ffmpeg_cmd_init

                            else:
                                cut_log.write(fp + '\n')
                        else:
                            cut_log.write('Not enough lines!\n')

                execute_cut(ffmpeg_cmd, dry_run)

            else:
                print("VideoRawLR not found")


def cut(folder, dry_run=False):
    logger = logging.getLogger('cut_moths')
    frames_fn = folder + '/frames.csv'
    if not os.path.exists(frames_fn):
        print("Frames.csv not found")
    else:
        video_in_fn = folder + '/videoRawLR.mkv'
        flights = lb.natural_sort([fp for fp in glob.glob(os.path.join(folder, "log_flight*.csv"))])
        if len(flights):
            cut_files(folder, flights, logger, video_in_fn, frames_fn, folder + '/cut_flight.log', dry_run)
        insects = lb.natural_sort([fp for fp in glob.glob(os.path.join(folder, "log_itrk*.csv")) if "itrk0" not in fp])
        if len(insects):
            cut_files(folder, insects, logger, video_in_fn, frames_fn, folder + '/cut_detections.log', dry_run)
        if not dry_run:
            os.remove(video_in_fn)


def execute_cut(ffmpeg_cmd, dry_run):
    logger = logging.getLogger('cut_moths')
    logger.info(ffmpeg_cmd)
    if not dry_run:
        lb.execute(ffmpeg_cmd, 1, 'cut_moths')


def cut_all(dry_run=False):
    logger = logging.getLogger('cut_moths')
    found_dirs = glob.glob(lb.data_dir + '/202*_*')
    for folder in found_dirs:
        try:
            video_in_file = folder + '/videoRawLR.mkv'
            if os.path.exists(video_in_file):
                logger.info('Processing ' + folder)
                cut(folder, dry_run)
            else:
                logger.info('Skipping ' + folder + ' because no videoRawLR.mkv')
        except Exception as e:  # pylint: disable=broad-except
            logger.error('Error in ' + folder + '; ' + str(e))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script that cuts the raw video into insects based on the log')
    parser.add_argument('-i', help="Path to the folder. If not provided all folders of the past day will be ran", required=False)
    parser.add_argument('--dry-run', help="Just run, don't cut or delete anything", dest='dry_run', action='store_true', required=False)
    parser.add_argument('-d', help="Run in build and debug folders", dest='debug', action='store_true', required=False)
    args = parser.parse_args()

    logging.basicConfig()
    logger = logging.getLogger('cut_moths')
    logger.setLevel(logging.DEBUG)

    if args.debug:
        if os.path.isdir('../build'):
            cut('../build', args.dry_run)
        if os.path.isdir('../build-vscode'):
            cut('../build-vscode', args.dry_run)
    elif not args.i:
        cut_all(args.dry_run)
    else:
        cut(args.i, args.dry_run)
