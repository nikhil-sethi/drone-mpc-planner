#!/usr/bin/env python3
import csv
import numpy as np
import pandas as pd
import os
import pathlib
from pathlib import Path
import sys
import random
from shutil import copyfile
import argparse

def cp_videos_for_checking(source_folder,files,n,dest_folder,fn_prefix,no_video,reprocess):
    fn_prefix = fn_prefix.replace('__','_') # this may happen during reprocess, because then the folder id is already in the name and sf = ''
    pathlib.Path(dest_folder).mkdir(parents=True, exist_ok=True)

    if reprocess:
        n = len(files)

    if n > len(files):
        n = len(files)
    if n > 0:
        files = random.sample(files, n)

        for fn_csv in files:

            fn = fn_csv[0:len(fn_csv)-4]
            if reprocess:
                fn = fn.replace('no_moth_','')
                fn = fn.replace('moth_','')
            else:
                fn = fn.replace('log','')
            fn_new_csv = fn_prefix + fn + ".csv"

            copyfile(Path(source_folder,fn_csv), Path(dest_folder,fn_new_csv))
            if not reprocess:
                header = ""
                fn_header = "log.csv"
                with open (Path(source_folder,fn_header), "r") as myfile:
                    header=myfile.readlines()
                line_prepender(Path(dest_folder,fn_new_csv),header[0])

            if not no_video:
                if reprocess:
                    fn_mp4 = fn_csv[0:len(fn_csv)-4] + ".mp4"
                else:
                    fn_mp4 = fn_csv[0:len(fn_csv)-4] + ".mp4"
                    fn_mp4 = "insect" + fn_mp4[3:len(fn_mp4)]
                fn_new_mp4 = fn_prefix + fn + ".mp4"
                if Path(source_folder,fn_mp4).is_file():
                    copyfile(Path(source_folder,fn_mp4), Path(dest_folder,fn_new_mp4))

def process_folder(source_folder, reprocess, verbose):
    if not reprocess:
        header_filename = Path(source_folder,"log.csv")
        if not header_filename.is_file():
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
        source_csv_file = Path(source_folder,f)
        try:
            if check_log(source_csv_file,headers,verbose):
                files_moth.append(f)
            else:
                files_no_moth.append(f)
        except Exception:
            files_corrupted.append(f)

    return files_moth,files_no_moth,files_corrupted

def check_log(source_csv_file,headers,verbose):
    if os.stat(source_csv_file).st_size < 1000:
        if verbose:
            print("No moth because: file size too small")
        return False
    else:
        with open(source_csv_file) as log_file:
            if not headers:
                log=pd.read_csv(log_file, sep=';',header=0)
                log.rename(columns=lambda x: x.strip(), inplace=True)
            else:
                log=pd.read_csv(log_file, names=headers, sep=';',header=None)
            
        return pre_checks(log,verbose)

                   
def pre_checks(log,verbose):
    if log.size < 5:
        return False
    else:
        velx = log["svelX_insect"].astype(float)
        if velx.abs().sum() < 0.1:
            if verbose:
                print("No moth because: velocity zero")
            return False
        else:
            posz = log["sposZ_insect"].astype(float)
            if posz.mean() < -1:
                if verbose:
                    print("No moth because: posZ < -1")
                return False
            else:
                if verbose:
                    print("MOTH DETECTED!")
                return True

def line_prepender(filename, line):
    with open(filename, 'r+') as f:
        content = f.read()
        f.seek(0, 0)
        f.write(line.rstrip('\r\n') + '\n' + content)

from os import listdir
def list_csv_files(directory):
    return (f for f in listdir(directory) if f.endswith('.csv') and not f.startswith("log."))

def printProgressBar (iteration, total, prefix = '', suffix = '', decimals = 1, length = 75, fill = '█', printEnd = "\r"):
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
parser.add_argument('-s', '--source_folder',type=str,default="/home/pats/data/")
parser.add_argument('-d', '--dest_folder',type=str,default="/home/pats/data_check/")
parser.add_argument('-f', '--file',type=str,default="")
parser.add_argument('-n', '--novideos', dest='novideo', action='store_true')
parser.add_argument('-r', '--reprocess', dest='reprocess', action='store_true')

parser.add_argument('-v', '--verbose', action='store_true')

args = parser.parse_args()
# parser.print_help()

if args.file != "":
    check_log(os.path.expanduser(args.file.strip(" ")),list(),True)
else:
    source_folder = os.path.expanduser(args.source_folder.strip(" "))
    dest_folder = os.path.expanduser(args.dest_folder.strip(" "))
    subfolders = [ name for name in os.listdir(source_folder) if os.path.isdir(os.path.join(source_folder, name)) ]

    if args.reprocess:
        subfolders.append("")

    i = 0
    all_files_corrupted = list()
    for sf in subfolders:
        i = i + 1
        source_subfolder = Path(source_folder, sf)
        if not args.reprocess:
            source_subfolder = Path(source_subfolder,"logging")
        print(str(i) + "/" + str(len(subfolders)) + ": " + str(source_subfolder))
        if not (Path(dest_folder,sf).is_dir()) or args.reprocess:
            files_moth,files_no_moth,files_corrupted = process_folder(source_subfolder,args.reprocess,args.verbose)
            
            files_corrupted = [Path(sf,s) for s in files_corrupted]
            print(files_corrupted)
            all_files_corrupted = all_files_corrupted + files_corrupted

            cp_videos_for_checking(source_subfolder,files_moth,3,dest_folder,'moth_' + sf + '_',args.novideo,args.reprocess)
            cp_videos_for_checking(source_subfolder,files_no_moth,3,dest_folder,'no_moth_' + sf + '_',args.novideo,args.reprocess)

    if (len(all_files_corrupted) > 0):
        print("Warning: the following corrupted files where encountered.")
        print(all_files_corrupted)