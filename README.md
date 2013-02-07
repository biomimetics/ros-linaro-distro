ros-linaro-build
================

Instructions on building ROS on an odroid-u2 running Linaro.
You can either install via the provided install script (ros-install.sh), or follow instructions in this README.


Installation Setup
---------------------

### System update/upgrade:

    $ sudo apt-get update; sudo apt-get upgrade -y


### ROS dependencies:

Add ROS repo:

    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
    $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    $ sudo apt-get update; sudo apt-get upgrade -y

Install ROS base packages:

    $ sudo apt-get install python-rosdep python-wstool python-catkin-pkg build-essential -y

Install system dependencies:

    $ sudo apt-get install libapr1-dev libaprutil1-dev libbz2-dev python-dev libgtest-dev python-paramiko libboost-all-dev liblog4cxx10-dev pkg-config python-empy swig python-nose lsb-release python-wxgtk2.8 python-gtk2 python-matplotlib libwxgtk2.8-dev python-imaging libqt4-dev graphviz qt4-qmake python-numpy libtiff4-dev libpoco-dev assimp-utils libtinyxml-dev python-pydot python-qwt5-qt4 libxml2-dev libsdl-image1.2-dev bison++ automake autoconf -y

Other useful system tools:

    $ sudo apt-get install ranger emacs -y

### Catkin Setup

Make workspace:

    $ sudo mkdir -p /opt/ros/groovy/catkin_ws
    $ cd /opt/ros/groovy/catkin_ws

Initialize workspace with ROS desktop distribution:

    $ sudo wstool init src -j4 http://packages.ros.org/web/rosinstall/generate/raw/groovy/desktop

Add swig-wx to workspace:

    $ cd src
    $ sudo wstool set swig-wx https://github.com/ros/swig-wx.git --git -y ; sudo wstool update swig-wx
    $ cd ..

### Install ROS Package dependencies

Patch rosdep for linaro

First, place this repo(ros-linaro-build) in your $HOME.

    $ git clone https://github.com/andrewjchen/ros-linaro-build.git

Patch rosdep for Linaro support:

    $ cd /usr/share/pyshared/rospkg/
    $ sudo patch -p0 < ~/patches/rospkg/rospkg-linaro.patch

Install ROS system dependencies via rosdep:

    $ https://github.com/andrewjchen/ros-linaro-build.git
    $ rosdep install --from-paths src --ignore-src --rosdistro groovy -yr

### Yaml-cpp



