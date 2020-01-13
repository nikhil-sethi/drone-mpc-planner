### How to use the 3d visualizer

## install rviz2

#setup locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

#add ros repository
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

#add repository to sources list
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

#install rviz2
sudo apt update
sudo apt install ros-dashing-rviz2

#environment setup
echo "source /opt/ros/dashing/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "source /opt/ros/dashing/setup.zsh" >> ~/.zshrc
source ~/.zshrc

## enable 3dviz in pats build
set 3D_VIZ to TRUE

##make local copy of rviz config file
cp ~/code/pats/config/pats.rviz ~/

## run rivz2
rviz2 -d ~/pats.rviz
