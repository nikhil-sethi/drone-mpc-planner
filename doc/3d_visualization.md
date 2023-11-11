# How to use the 3D-Visualizer

## Install ros(1) and rviz

[http://wiki.ros.org/Installation/Ubuntu](ubuntu ros install)

## Configure pats project for 3D-Visualization

For showing hunt scenario in a replay:

    cd ~/code/pats/base/build
    cmake -DROSVIS=TRUE ..
    
For showing in solutions steps of the optimzer:

    cd ~/code/pats/base/build
    cmake -DOPTI_ROSVIS=TRUE ..

## Start ros and rviz
Before starting the replay:

    roscore &
    rviz -d ~/code/pats/config/pats.rviz

## Run executor
As normal:

    executor --log <logging-folder> --flight <flight-number>
