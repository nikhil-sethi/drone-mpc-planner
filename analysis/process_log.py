#!/usr/bin/env python3
import os
from pathlib import Path
import argparse
import processloglib as lib

parser = argparse.ArgumentParser(description='Process and check the logs.')
parser.add_argument('-s', '--source_folder',type=str,default="/home/pats/data/")
parser.add_argument('-d', '--dest_folder',type=str,default="/home/pats/data_check/")
parser.add_argument('-f', '--file',type=str,default="")
parser.add_argument('-n', '--novideos', dest='novideo', action='store_true')
parser.add_argument('-r', '--reprocess', dest='reprocess', action='store_true')
parser.add_argument('-v', '--verbose', action='store_true')
args = parser.parse_args()

if args.file != "":
    res, log = lib.check_log(os.path.expanduser(args.file.strip(" ")),[],True)
else:
    source_folder = os.path.expanduser(args.source_folder.strip(" "))
    dest_folder = os.path.expanduser(args.dest_folder.strip(" "))
    subfolders = [ name for name in os.listdir(source_folder) if os.path.isdir(os.path.join(source_folder, name)) ]

    if args.reprocess:
        subfolders.append("") #the results from this script put all files in a single directory. so when reprocessing we don't have subfolders anymore

    i = 0
    all_files_corrupted = list()
    for sf in subfolders:
        i = i + 1
        source_subfolder = Path(source_folder, sf)
        if not args.reprocess:
            source_subfolder = Path(source_subfolder,"logging")
        print(str(i) + "/" + str(len(subfolders)) + ": " + str(source_subfolder))
        if not (Path(dest_folder,sf).is_dir()) or args.reprocess:
            files_moth,files_no_moth,files_corrupted = lib.process_folder(source_subfolder,args.reprocess,args.verbose)
            
            files_corrupted = [Path(sf,s) for s in files_corrupted]
            print(files_corrupted)
            all_files_corrupted = all_files_corrupted + files_corrupted

            lib.cp_videos_for_checking(source_subfolder,files_moth,3,dest_folder,'moth_' + sf + '_',args.novideo,args.reprocess)
            lib.cp_videos_for_checking(source_subfolder,files_no_moth,3,dest_folder,'no_moth_' + sf + '_',args.novideo,args.reprocess)

    if (len(all_files_corrupted) > 0):
        print("Warning: the following corrupted files where encountered.")
        print(all_files_corrupted)