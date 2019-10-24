#!/usr/bin/env python3

import os

path = '/home/pats/data/'
sift_dir = "data_logs"
sift_path = "/home/pats/" + sift_dir + "/"

csv_file = "logging/log*.csv "
ins_file = "logging/insect.log "

os.system("mkdir -p " + sift_path)

dirs = next(os.walk(path))[1]

for x in dirs:
    dir = path + x + "/"
    cmd1 = "mkdir -p " + sift_path + x
    cmd2 = "cp " + dir + csv_file + " " + sift_path + x
    cmd3 = "cp " + dir + ins_file + " " + sift_path + x
    print(cmd1)
    print(cmd2)
    print(cmd3)
    os.system(cmd1)
    os.system(cmd2)
    os.system(cmd3)

cmd4 = "cd /home/pats/ && tar -czvf " + sift_dir + ".tar.gz " + sift_dir
print(cmd4)
os.system(cmd4)

cmd5 = "rm -rf " + sift_path
print(cmd5)
os.system(cmd5)



print("Processed " + str(len(dirs)) + " dirs!")