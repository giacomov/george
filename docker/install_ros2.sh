#!/usr/bin/env bash

set -e

apt update && apt install -y curl gnupg2 lsb-release

ARCH=$(uname -i)
RELEASE=$(lsb_release -c -s)

apt update && apt install -y locales

locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

if [ $RELEASE == "bionic" ]
    then
        ROSDISTRO=eloquent

elif [ $RELEASE == "focal" ]
    then
        ROSDISTRO=galactic

elif [ $RELEASE == "jammy" ]
    then
        ROSDISTRO=humble

else
    echo "$RELEASE OS Release not supported. Exiting now"
    exit
fi

if [ $ARCH != "x86_64" ] && [ $ARCH != "aarch64" ]
    then
        echo "$ARCH architecture not supported. Exiting now"
        exit
fi

echo "Probed results: Ubuntu $RELEASE on $ARCH, building ROS2 $ROSDISTRO"

RELEASE_FILE=$ROSDISTRO$ARCH

# Add the ROS 2 apt repository
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "Installing ROS $ROSDISTRO on $ARCH architecture"
apt update

# Install colcon
apt install -y python3-colcon-common-extensions python3-colcon-core python3-rosdep

if [ $ARCH == "x86_64" ]
    then
        apt install -y ros-$ROSDISTRO-desktop

elif [ $ARCH == "aarch64" ]
    then
        apt install -y ros-$ROSDISTRO-ros-base
fi

# Install python3 libraries
apt install -y libpython3-dev python3-pip
pip3 install -U argcomplete pytest-rerunfailures

# Remove conflicting em package
pip3 uninstall em || echo "em not installed"

rosdep init
rosdep update

echo ""
echo "ROS $ROSDISTRO installation complete!"
echo ""

echo "source /opt/ros/$ROSDISTRO/setup.bash" >> $HOME/.bashrc
