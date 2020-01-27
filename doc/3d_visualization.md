# How to use the 3d visualizer

## Install rviz2

### Setup locale

    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

### Add ros repository

    sudo apt update && sudo apt install curl gnupg2 lsb-release
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

### Add repository to sources list

    sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

### Install rviz2

    sudo apt update
    sudo apt install ros-dashing-rviz2

## Environment setup
For the default ubuntu terminal (_bash_):

    echo "source /opt/ros/dashing/setup.bash" >> ~/.bashrc
    source ~/.bashrc

For _zsh_:

    echo "source /opt/ros/dashing/setup.zsh" >> ~/.zshrc
    source ~/.zshrc

## Enable 3dviz in pats build
In the build folder (e.g. `pc/build-vscode`)

    cmake -DVIZ_3D=TRUE ..

### Make local copy of rviz config file:

    cp ~/code/pats/config/pats.rviz ~/

### Run rivz2

    rviz2 -d ~/pats.rviz
