#!/usr/bin/env bash

#usage: ./render_folders.sh ~/data/ ~/data_renders/

mkdir -p $2

for dir in $1/*/
do
    dir=${dir%*/}      # remove the trailing "/"
    echo Rendering: ${dir}
    subdir=`basename "$dir"`
    [ -f ${2}/${subdir}.mp4 ] || {
        echo Rendering: ${dir}
        mv ${dir}/logging/pats.xml ${dir}/logging/pats.xml.bku
        #cp ~/Downloads/pats.xml ${dir}/logging/pats.xml #TMP!
        (cd ~/code/pats/pc/build/ && ./pats --log ${dir}/logging/ --render)
        cp ~/code/pats/pc/build/logging/replay/videoResult.mp4 ${2}/${subdir}.mp4
        mv ~/code/pats/pc/build/logging/replay/videoResult.mp4 $dir/
    }
done