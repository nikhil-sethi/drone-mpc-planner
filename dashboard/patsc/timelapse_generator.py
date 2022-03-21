#!/usr/bin/env python3
import os
import re
from datetime import datetime
import glob
import argparse
import subprocess


def str_to_datetime(string):
    return datetime.strptime(string, "%Y%m%d_%H%M%S")


def execute(cmd):
    popen = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
    for _ in iter(popen.stdout.readline, ""):
        popen.poll()

    popen.stdout.close()


def natural_sort(line):
    def convert(text):
        return int(text) if text.isdigit() else text.lower()

    def alphanum_key(key):
        return [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(line, key=alphanum_key)


parser = argparse.ArgumentParser(description='Script that cuts the raw video into insects based on the log')
parser.add_argument('--start', help="Start date", required=True)
parser.add_argument('--end', help="End date", dest='end', required=True)
parser.add_argument('--prefix', help="rgb / stereoL/ stereoR", required=True)
args = parser.parse_args()

images_dir = os.path.expanduser('~/pats/images/')
images = glob.glob(images_dir + '/' + args.prefix + '*_13*.png')
images = natural_sort(images)
timelapse_dir = os.path.expanduser('~/pats/timelapse/')

start_date = str_to_datetime(args.start)
end_date = str_to_datetime(args.end)

if not os.path.exists(timelapse_dir):
    os.makedirs(timelapse_dir)

with open(timelapse_dir + '/timelapse_list.txt', 'w') as f:

    for im in images:
        stem = os.path.basename(im).split('.')[0]  # Path(im).stem requires a dependency to be installed
        d = str_to_datetime(stem[len(args.prefix) + 1:])
        if d > start_date and d < end_date:
            f.write("file '" + im + "'\n")

cmd = 'ffmpeg -n -f concat -safe 0 -r 5 -i ' + timelapse_dir + '/timelapse_list.txt -c:v libx264 -pix_fmt yuv420p ' + timelapse_dir + '/timelapse_' + args.prefix + '_' + args.start + '_' + args.end + '.mp4'
execute(cmd)
