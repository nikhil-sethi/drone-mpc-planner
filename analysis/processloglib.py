#!/usr/bin/env python3
import csv
import numpy as np
import pandas as pd
import os
import pathlib
from pathlib import Path
import random
from shutil import copyfile
import math
from scipy.signal import savgol_filter

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


def get_dataset(source_folder,n0,n1):
    
    files = list_csv_files(source_folder)
    i = 0
    n = sum(1 for i in list_csv_files(source_folder)) #python=KUT
    
    dataset = list()
    csv_data = list()
    for f in files:
        i = i + 1
        printProgressBar(i,n)
        source_csv_file = Path(source_folder,f)
        pre_check_ok = False
        try:
            pre_check_ok, log = read_log(source_csv_file,[],False)
        except Exception:
            pass

        if pre_check_ok:
                imLx,imLy,RS_ID,posX,posY,posZ = get_chunk(log,n0,n1)
                feat_vec = calc_features(not 'no_moth' in f,imLx,imLy,RS_ID,posX,posY,posZ)
                if not math.isnan(feat_vec[0]) and  not math.isnan(feat_vec[1]): 
                    dataset.append(feat_vec)
                    csv_data.append(np.transpose([RS_ID,imLx,imLy,posX,posY,posZ]))
                else:
                    print('\nObtained NaN in ' + f)
    dataset = np.asarray(dataset, dtype=np.float32)
    csv_col_names = ['RS_ID','imLx','imLy','posX','posY','posZ']
    return dataset,csv_data,csv_col_names

def calc_features(moth,imLx,imLy,RS_ID,posX,posY,posZ):
    c = 0
    if moth:
        c = 1
    return np.array([c,autocorr(imLx),autocorr(imLy),avg_speed(posX,posY,posZ),im_travel_distance(imLx,imLy)])

def autocorr(x, t=1):
    return np.corrcoef(np.array([x[:-t], x[t:]]))[1][0]

def avg_speed(x,y,z):
    dx = np.diff(x)
    dy = np.diff(y)
    dz = np.diff(z)
    tot = np.sum(np.sqrt(np.square(dx) + np.square(dy) + np.square(dz)))

    return tot

def im_travel_distance(x,y):
    x = savgol_filter(x, 51, 3)
    y = savgol_filter(y, 51, 3)
    d = math.sqrt((x[0] - x[-1])**2 + (y[0] - y[-1])**2)
    
    return d

def read_log(source_csv_file,headers,verbose):
    if os.stat(source_csv_file).st_size < 1000:
        if verbose:
            print("No moth because: file size too small")
        return False,[]
    else:
        with open(source_csv_file) as log_file:
            if not headers:
                log=pd.read_csv(log_file, sep=';',header=0)
                log.rename(columns=lambda x: x.strip(), inplace=True)
            else:
                log=pd.read_csv(log_file, names=headers, sep=';',header=None)
            
            if len(log.index) < 5:
                if verbose:
                    print("Not enough rows")
                return False,log
        log = log.dropna()
        return True,log

def check_log(source_csv_file,headers,verbose):
    res,log = read_log(source_csv_file,headers,verbose)
    if res:
        return check_for_moths(log,verbose),log
    else:
        return False,log

def check_for_moths(log,verbose):
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

def get_chunk(log,n0,n1):
    if len(log["imLx_insect"]) < n1:
        n1 = len(log["imLx_insect"])
    if n0 > n1:
        n0 = n1
    
    RS_ID = np.array(log["RS_ID"].astype(int))[n0:n1]
    imLx_insect = np.array(log["imLx_insect"].astype(float))[n0:n1]
    imLy_insect = np.array(log["imLy_insect"].astype(float))[n0:n1]
    
    posX_insect = np.array(log["posX_insect"].astype(float))[n0:n1]
    posY_insect = np.array(log["posY_insect"].astype(float))[n0:n1]
    posZ_insect = np.array(log["posZ_insect"].astype(float))[n0:n1]
     
    return imLx_insect,imLy_insect,RS_ID,posX_insect,posY_insect,posZ_insect


def append_zeros_to_log(entry,n):
    entry = np.asarray(entry, dtype=np.float32)
    return np.apply_along_axis( append_zeros_column, axis=0, arr=entry,n=n )

def append_zeros_column(col,n):
    if len(col) < n:
        result = np.pad(col, (n-len(col)-1, 1), 'constant', constant_values=(0))
        return result
    else: 
        return col
    


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
