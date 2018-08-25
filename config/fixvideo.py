#!/usr/bin/env python
import sys
import os
import subprocess

path = sys.argv[1]

for root, dirs, files in os.walk(path):
    #print(dirs)
    for tdir in dirs:        
        fdir = os.path.join(root, tdir)
        #print(fdir)
        for root2, dirs2, files2 in os.walk(fdir):
            if "videoRawLR.avi" in files2 and "videoRawLR_fixed.avi" not in files2:
                #call convert script here  ffmpeg -i tmp.avi -c:v libx264 -crf 18 -preset slow -c:a copy output.avi
                
                inf = os.path.join(fdir,'videoRawLR.avi')
                statinfo = os.stat(inf)

                outf = os.path.join(fdir,'videoRawLR_fixed.avi')
                print('**********************************************************************')
                print(inf)
                print('**********************************************************************')
                if (statinfo.st_size > 0):
                    subprocess.call('touch ' + outf, shell=True)
                    subprocess.call(['ffmpeg -i ' + inf + ' -y -c:v libx264 -crf 18 -preset slow -c:a copy ' + outf], shell=True)
                else:
                    print('Skipping (empty): '+ fdir)
            else:
                print('Skipping (already done): '+ fdir)
