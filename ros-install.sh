#!/bin/bash
# ros-install.sh

# System update, upgrade
sudo apt-get update; sudo apt-get upgrade -y

# Adds ros repo
# via http://www.ros.org/wiki/groovy/Installation/Ubuntu#groovy.2BAC8-Installation.2BAC8-Sources.Setup_your_sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update; sudo apt-get upgrade -y

# Adds ros package dependencies
sudo apt-get install python-rosdep python-wstool python-catkin-pkg build-essential -y

# Adds other dependencies
sudo apt-get install libapr1-dev libaprutil1-dev libbz2-dev python-dev libgtest-dev python-paramiko libboost-all-dev liblog4cxx10-dev pkg-config python-empy swig python-nose lsb-release python-wxgtk2.8 python-gtk2 python-matplotlib libwxgtk2.8-dev python-imaging libqt4-dev graphviz qt4-qmake python-numpy libtiff4-dev libpoco-dev assimp-utils libtinyxml-dev python-pydot python-qwt5-qt4 libxml2-dev libsdl-image1.2-dev bison++ automake autoconf -y

sudo apt-get install ranger emacs -y

# Catkin setup
sudo mkdir -p /opt/ros/groovy/catkin_ws
cd /opt/ros/groovy/catkin_ws

# Downloads packages from groovy desktop
sudo wstool init src -j4 http://packages.ros.org/web/rosinstall/generate/raw/groovy/desktop

# adds swig-wx to the catkin workspace
cd src
sudo wstool set swig-wx https://github.com/ros/swig-wx.git --git -y ; sudo wstool update swig-wx
cd ..

# Install Dependencies

# Patching rosdep for linaro
cd /usr/share/pyshared/rospkg/
sudo patch -p0 < ~/patches/rospkg/rospkg-linaro.patch
# TODO download patchset from somewhere

# perform dependency install ignoring errors
cd /opt/ros/groovy/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro groovy -yr
# rosdep check --from-paths src --ignore-src --rosdistro groovy