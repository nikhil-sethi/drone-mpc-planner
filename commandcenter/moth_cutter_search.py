#!/usr/bin/python

import sys
print(sys.version)


# import pandas as pd
# import numpy as np
# import os, math, glob, re
# import datetime
# import subprocess
# import argparse


# def secs_to_time(seconds: float):
#     """converts seconds to a time forma supported by ffmpeg"""
#     mins = int(seconds // 60)
#     left = seconds % 60
#     secs = int(math.floor(left))
#     millies = str(round(left, 2)).split(".")[-1]
#     return f"00:{mins:02}:{secs:02}.{millies[:2]}"

# def cut_video_for_moth(absolute_video_path: str, output_path: str, delete_raw: bool=False):
#     main_path = os.path.dirname(absolute_video_path)
#     log_file_path = os.path.join(main_path, "log_itrk0.csv") # read csv with insect info

#     df = pd.read_csv(log_file_path, sep=";")
#     ds_insc = df['foundL_insect']
#     found_indices = np.where(ds_insc == 1)
#     buffer = 90 # 1 sec buffer before and after the insectis seen (90 fps)
#     last_record_index = ds_insc.shape[0]
#     lower_bound = max(0, found_indices[0][0]-buffer) 
#     upper_bound = min(last_record_index, found_indices[0][-1]+buffer)
#     new_csv = df[lower_bound:upper_bound]
#     post_fix = main_path.split("/")[-2]
#     new_csv.to_csv(os.path.join(output_path, f"insect_{post_fix}.csv"), sep=";", index=False) # creates a new csv that corresponds to the length of the cut video


#     # read the length of the video
#     s = ['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of', 'default=noprint_wrappers=1:nokey=1', absolute_video_path]
#     p = subprocess.Popen(s, stdout=subprocess.PIPE)
#     out, err = p.communicate()
#     vid_length = float(out) # str -> float

#     # get lower and upper bound of cut video from log
#     log_vid_diff = df.iloc[0]["time"]
#     video_lower_bound = new_csv.iloc[0]["time"] - log_vid_diff
#     video_upper_bound = new_csv.iloc[-1]["time"] - log_vid_diff

#     video_start_timestamp = lower_bound / last_record_index * vid_length
#     video_end_timestamp = upper_bound / last_record_index * vid_length
#     video_start_timestamp = secs_to_time(video_lower_bound)
#     video_end_timestamp = secs_to_time(video_upper_bound)

#     cmd = ["ffmpeg", "-i", absolute_video_path,
#     "-ss", str(video_start_timestamp),
#     "-to", str(video_end_timestamp),
#     os.path.join(output_path, f"cut_video_{post_fix}.mkv")]
#     subprocess.check_output(cmd)

#     if delete_raw:
#         for file_ in [log_file_path, absolute_video_path]:
#             if os.path.exists(file_):
#                 os.remove(file_)


# parser = argparse.ArgumentParser(description='Video slicer to end up with only the fragments that contain moths')

# parser.add_argument('-i', help="path to the folder with logs")
# parser.add_argument('-t', help="path to text_file with picked video names")
# parser.add_argument('-o', help="path to the folder as output for the cut videos")
# parser.add_argument('-d', help="delete raw data", default=False)

# args = parser.parse_args()
# delete_files = args.d

# log_folder = args.i
# found_videos = glob.glob(log_folder + "/*/logging/videoRawLR.mkv")

# with open(args.t, "r") as r:
#     data = "".join(r.readlines())
#     dirs = re.findall(r"\d+_\d+",data)
# filtered_vids = [i for i in found_videos if any(i for j in dirs if str(j) in i)]

# for found in filtered_vids:
#     cut_video_for_moth(found, args.o, args.d)

