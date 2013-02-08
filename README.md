ros-linaro-build
================

Instructions on building ROS on an odroid-u2 running Linaro.
You can either install via the provided install script (ros-install.sh), or follow instructions in this README.

Current Images
--------------

[2013 Feb 05](https://berkeley.box.com/s/32q7f01xh0oayk3h4d72)

Current Status
--------------

May have bugs, and is not 100% complete yet.

[Changelog] (Changelog.md)
[Resources] (Resources.md)

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

Building yaml-cpp via repo

    $ echo "deb http://ppa.launchpad.net/stephane.magnenat/precise/ubuntu precise main
deb-src http://ppa.launchpad.net/stephane.magnenat/precise/ubuntu precise main" | sudo tee -a /etc/apt/sources.list
    $ sudo apt-get update; sudo apt-get upgrade -y
    $ sudo apt-get build-dep yaml-cpp
    $ mkdir -p ~/fix/yaml-cpp
    $ cd ~/fix/yaml-cpp
    $ sudo apt-get source yaml-cpp
    $ cd yaml-cpp-0.2.6
    $ sudo dpkg-buildpackage -us -uc -nc
    $ cd ..
    $ sudo dpkg -i *.deb

Build yaml-cpp dummy package

    $ mkdir -p ~/fix/yaml-cpp/
    $ cp ~/ros-linaro-build/yaml-cpp/yaml-cpp ~/fix/yaml-cpp/yaml-cpp
    $ cd ~/fix/yaml-cpp
    $ sudo apt-get install equivs -y
    $ equivs-build yaml-cpp
    $ sudo dpkg -i yaml-cpp_1.0_all.deb



### tbb-dev

Download and build patched tbb-dev:

    $ sudo mkdir -p /opt/intel/libtbb-dev
    $ cd /opt/intel/libtbb-dev
    $ sudo wget http://threadingbuildingblocks.org/sites/default/files/software_releases/source/tbb40_20120613oss_src.tgz
    $ sudo tar xzvf tbb40_20120613oss_src.tgz
    $ cd tbb40_20120613oss
    $ sudo patch -p1 < ~/ros-linaro-build/tbb-dev/tbb40_20120613oss-0001-Endianness.patch
    $ sudo patch -p1 < ~/ros-linaro-build/tbb-dev/tbb40_20120613oss-0002-ARM-support.patch
    $ sudo patch -p1 < ~/ros-linaro-build/tbb-dev/tbb40_20120613oss-0003-Add-machine_fetchadd-48-intrinsics.patch
    $ sudo make -j4
    $ echo "source /opt/intel/libtbb-dev/tbb40_20120613oss/build/linux_armv7_gcc_cc4.6_libc2.15_kernel3.0.60_release/tbbvars.sh" >> ~/.bashrc


Build tbb-dev dummy package:

    $ mkdir -p ~/fix/tbb-dev/
    $ cp ~/ros-linaro-build/tbb-dev/libtbb-dev ~/fix/tbb-dev/libtbb-dev
    $ cd ~/fix/tbb-dev
    $ sudo apt-get install equivs -y
    $ equivs-build libtbb-dev
    $ sudo dpkg -i libtbb-dev_1.0_all.deb

### Collada-dom

    $ cd /opt
    $ wget http://sourceforge.net/projects/collada-dom/files/latest/download
    $ tar xf download
    $ cd collada-dom-2.4.0
    $ cmake .
    $ sudo make install -j4

### flann
    $ cd /opt
    $ wget http://people.cs.ubc.ca/~mariusm/uploads/FLANN/flann-1.8.4-src.zip
    $ unzip flann*
    $ mkdir build
    $ cd build
    $ cmake ..
    $ sudo make install -j4

### pcl

### Remove unsupported packages

    $ cd /opt/ros/groovy/catkin_ws/src
    $ sudo rm -rf */
    $ sudo patch -p0 <  ~/ros-linaro-build/ros-desktop-install/rosinstall.patch
    $ sudo rosws update

### rosgraph/ifaddrs.py Bug

    $ cd /opt/ros/groovy/catkin_ws/src/rosgraph/src/rosgraph
    $ sudo patch -p0 < ~/ros-linaro-build/rosgraph/rosgraph-ifaddrs.py.patch


Build ROS-desktop
-----------------

### Build Catkin packages

    $ cd /opt/ros/groovy/catkin_ws
    $ sudo ./src/catkin/bin/catkin_make_isolated -j1 --install

Build rosbuild packages(TODO)

    $ source /opt/ros/groovy/catkin_ws/install_isolated/setup.bash
    $ mkdir -p ~/rosbuild_ws
    $ rosws init ~/rosbuild_ws /opt/ros/groovy/install_isolated
    $ rosws merge http://packages.ros.org/web/rosinstall/generate/dry/raw/groovy/desktop
    $ rosws update -j4
    $ source ~/rosbuild_ws/setup.bash
    $ rosmake -a


ROS user-level workspace
------------------------

### Catkin workspace

    $ mkdir -p ~/ros_catkin_ws/src
    $ source ~/rosbuild_ws/setup.bash
    $ cd ~/ros_catkin_ws/src
    $ wstool init

### Rosbuild workspace

    $ mkdir -p ~/ros_ws
    $ source ~/ros_catkin_ws/install/setup.bash
    $ rosws init ~/ros_ws ~/ros_catkin_ws/install


Optional: rosjava
-----------------

### Downloading the package


### Downloading Gradle


### Downloading jdk-1.8.0


### rosjava patch