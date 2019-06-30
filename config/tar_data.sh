#!/usr/bin/env bash
set -ex

mkdir ~/data_tmp/
cp ~/data/* ~/data_tmp/ -r
cd ~/data_tmp/

find -iname videoRawLR.avi -exec ffmpeg -i {} -c:v libx264 -level 3.0 -pix_fmt yuv420p -crf 21 -preset slow {}_reencoded.mp4 || true \; -exec rm {} \;

find -iname videoRawLR.avi -exec rm {} \;
find -iname terminal.log -exec rm {} \;
find -iname brightness.png -exec rm {} \;
find -iname disparity.png -exec rm {} \;
find -iname depth.png -exec rm {} \;
find -iname depth_filtered.png -exec rm {} \;
find -iname cam_calib.xml -exec rm {} \;

tar -czvf ../data.tar.gz *

