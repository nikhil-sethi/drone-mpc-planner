#!/usr/bin/env bash
set -e
DIRECTORY=~/code

if [ "$#" -ne 1 ]; then
	echo "Usage: $./install_sim.sh [user_version=0,dev_version=1]"
	exit 1
fi

if [[ $1 -eq 1 ]] ; then
	echo "Starting simulator install install script for developers!"
  ALIAS_NAME="startsimdev" # name of the alias to start the greenhouse env in the unreal editor
else
  ALIAS_NAME="startsim" # name of the alias to start the simulator
fi

echo "Installing dependencies..."
# install dependencies
sudo apt-get install gcc-8 g++-8

# make gcc8 default
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 700 --slave /usr/bin/g++ g++ /usr/bin/g++-7
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8

# check if directory exists
if [ ! -d $DIRECTORY ]; then
  mkdir -p $DIRECTORY
fi
cd $DIRECTORY

# DEV ONLY - install Unreal Engine
if [[ $1 -eq 1 ]] ; then
  echo "Installing Unreal Engine..."
  if [ ! -d $DIRECTORY/UnrealEngine ]; then
    # checking filespace
    FREE=`df -k --output=avail "$DIRECTORY" | tail -n1`
    if [[ $FREE -gt 104857600 ]]; then # 100G = 100*1024*1024k
        git clone -b 4.25 git@github.com:EpicGames/UnrealEngine.git || { echo "ERROR: Cloning failed. Are you a member of the Epic Games organization on Github?" && exit 1; }
        cd $DIRECTORY/UnrealEngine
        ./Setup.sh
        ./GenerateProjectFiles.sh
        make
        cd $DIRECTORY
    else
      echo "ERROR: Not enough space available (should be at least 100G)" && exit 1
    fi
  else
    echo "Unreal installation found!"
  fi

  # clone AirSim environment
  echo "Cloning AirSim environment..."
  if [ ! -d $DIRECTORY/AirSimEnvironment ]; then
    git clone git@github.com:pats-drones/AirSimEnvironment.git || { echo "ERROR: Cloning failed. Are you a member of the Pats organization on Github?" && exit 1; }

    if [ -d ~/Documents/AirSim ]; then
      rm -rf ~/Documents/AirSim
    fi
    mkdir -p ~/Documents/AirSim
    # link config file
    ln -s $DIRECTORY/AirSimEnvironment/settings.json ~/Documents/AirSim/settings.json
  else
    echo "AirSim environment found!"
  fi
else
  echo "Downloading PATS Simulator..."

  if [ -d $DIRECTORY/Simulator ]; then
      rm -rf $DIRECTORY/Simulator
  fi
  mkdir $DIRECTORY/Simulator && cd $DIRECTORY/Simulator

  echo "Because downloading private releases form GitHub sucks, please provide a Github token (https://github.com/settings/tokens/new) with repo and read:packages acces"
  read TOKEN

  # get the latest release id from the API
  URL=$(
      curl  -H 'Authorization: token '"$TOKEN"'' "https://api.github.com/repos/pats-drones/AirSimEnvironment/releases/latest" |
        grep '"url":'                 |
        sed -E 's/.*"([^"]+)".*/\1/'  |
        sed -n 3p
  )

  curl -LJ -o Simulator.zip -H 'Accept: application/octet-stream' -H 'Authorization: token '"$TOKEN"'' $URL
  unzip Simulator.zip
  rm Simulator.zip
  cd $DIRECTORY
fi

# install AirSim
echo "Installing AirSim..."
if [ ! -d $DIRECTORY/AirSim ]; then
  git clone -b simulator git@github.com:pats-drones/AirSim.git || { echo "ERROR: Cloning failed" && exit 1; }

  # DEV ONLY - link the greenhouse environment to AirSim so the build script of AirSim works
  if [[ $1 -eq 1 ]] ; then
    ln -s $DIRECTORY/AirSimEnvironment/Greenhouse $DIRECTORY/AirSim/Unreal/Environments/
  fi

  cd $DIRECTORY/AirSim
  ./setup.sh
  ./build.sh
  cd $DIRECTORY
else
  echo "AirSim installation found! Updating..."
  cd $DIRECTORY/AirSim
  git checkout simulator
  git pull
  cd $DIRECTORY
fi

# add WITH_AIRSIM flag to cmake
if [ -d $DIRECTORY/pats ]; then
  cd $DIRECTORY/pats/base
  mkdir -p build-vscode
  cd build-vscode
  cmake .. -DWITH_AIRSIM=TRUE
  cd $DIRECTORY
else
  echo "Pats installation not found! Please run 'cmake .. -DWITH_AIRSIM=TRUE' in the build-vscode folder"
fi

# add alias
if [ `alias | grep $ALIAS_NAME= | wc -l` == 0 ]; then
  if [[ $1 -eq 1 ]] ; then
    echo "alias $ALIAS_NAME='$DIRECTORY/UnrealEngine/Engine/Binaries/Linux/UE4Editor $DIRECTORY/AirSimEnvironment/Greenhouse/Greenhouse.uproject'" >> ~/.bash_aliases
    echo "export PATH=$DIRECTORY/UnrealEngine/Engine/Binaries/Linux:$PATH" >> ~/.bashrc
  else
    echo "alias $ALIAS_NAME='$DIRECTORY/Simulator/Greenhouse.sh -ResX=640 -ResY=480 -windowed'" >> ~/.bash_aliases
  fi
fi

echo
echo "****************************************************************"
echo
echo PATS simulator installed!
echo
echo To start the simulator restart the terminal and run: $ALIAS_NAME
echo
echo Please see *Installation* in the simulator docs for more information.
echo
echo "****************************************************************"
