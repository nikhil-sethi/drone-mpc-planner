#!/usr/bin/env python3
import csv
import numpy as np
import pandas as pd
import os
import pathlib
import sys
import random
from shutil import copyfile
import argparse

def cp_videos_for_checking(source_folder,files,n,dest_folder,no_video):

    pathlib.Path(dest_folder).mkdir(parents=True, exist_ok=True)

    if n > len(files):
        n = len(files)
    if n > 0:
        files = random.sample(files, n)
        for f_csv in files:
            copyfile(source_folder+f_csv, dest_folder+f_csv)
            if not no_video:
                f_mp4 = f_csv[0:len(f_csv)-4] + ".mp4"
                copyfile(source_folder+f_mp4, dest_folder+f_mp4)

def process_folder(source_folder):
    header_filename = source_folder + "log.csv"
    if not os.path.exists(header_filename):
        return list(), list(),list()
    with open(header_filename) as header_file:
        headers = csv.reader(header_file, delimiter=';')
        headers = list(headers)
        headers = headers[0]
        headers = [x.strip(' ') for x in headers]

    files = list_csv_files(source_folder)
    files_moth = list()
    files_no_moth = list()
    files_corrupted = list()
    i = 0
    n = sum(1 for i in list_csv_files(source_folder)) #python=KUT
    for f in files:
        i = i + 1
        printProgressBar(i,n)
        source_csv_file = source_folder + f
        if os.stat(source_csv_file).st_size < 1000:
            files_no_moth.append(f)
        else:
            with open(source_csv_file) as log_file:
                try:
                    log=pd.read_csv(log_file, names=headers, sep=';',header=None)
                    if log.size < 5:
                        files_no_moth.append(f)
                    else:
                        velx = log["svelX_insect"].astype(float)
                        if velx.abs().sum() < 0.1:
                            files_no_moth.append(f)
                        else:
                            posz = log["sposZ_insect"].astype(float)
                            if posz.mean() < -1:
                                files_no_moth.append(f)
                            else:
                                files_moth.append(f)
                except:
                    files_corrupted.append(f)

    return files_moth,files_no_moth,files_corrupted


from os import listdir
def list_csv_files(directory):
    return (f for f in listdir(directory) if f.endswith('.csv') and not f.startswith("log."))

def printProgressBar (iteration, total, prefix = '', suffix = '', decimals = 1, length = 100, fill = '█', printEnd = "\r"):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
        printEnd    - Optional  : end character (e.g. "\r", "\r\n") (Str)
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print('\r%s |%s| %s%% %s' % (prefix, bar, percent, suffix), end = printEnd)
    # Print New Line on Complete
    if iteration == total: 
        print()





parser = argparse.ArgumentParser(description='Process and check the logs.')
parser.add_argument('-s', '--source_folder',type=str,default="/home/houjebek/data/")
parser.add_argument('-d', '--dest_folder',type=str,default="/home/houjebek/data_check/")
parser.add_argument('-n', '--novideos', dest='novideo', action='store_true')

args = parser.parse_args()
#parser.print_help()

source_folder = args.source_folder
subfolders = [ name for name in os.listdir(source_folder) if os.path.isdir(os.path.join(source_folder, name)) ]

i = 0
all_files_corrupted = list()
for sf in subfolders:
    i = i + 1
    source_subfolder = source_folder + sf + "/"
    print(str(i) + "/" + str(len(subfolders)) + ": " + source_subfolder)
    if not os.path.exists(args.dest_folder + sf):            
        files_moth,files_no_moth,files_corrupted = process_folder(source_subfolder)
        
        files_corrupted = [sf + '/' + s for s in files_corrupted]
        print(files_corrupted)
        all_files_corrupted = all_files_corrupted + files_corrupted

        cp_videos_for_checking(source_subfolder,files_moth,3,args.dest_folder + sf + '/moth/',args.novideo)
        cp_videos_for_checking(source_subfolder,files_no_moth,3,args.dest_folder + sf + '/no_moth/',args.novideo)

if (len(all_files_corrupted) > 0):
    print("Warning: the followingcorrupted files where encountered.")
    print(all_files_corrupted)