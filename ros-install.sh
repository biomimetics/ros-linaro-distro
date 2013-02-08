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

sudo apt-get install ranger htop  emacs -y

# Catkin setup
sudo mkdir -p /opt/ros/groovy/catkin_ws
cd /opt/ros/groovy/catkin_ws

# Downloads packages from groovy desktop
sudo wstool init src -j4 http://packages.ros.org/web/rosinstall/generate/raw/groovy/desktop

# adds swig-wx to the catkin workspace
cd src
sudo wstool set swig-wx https://github.com/ros/swig-wx.git --git -y ; sudo wstool update swig-wx
ccd ..

# Install Dependencies

# Patching rosdep for linaro
cd /usr/share/pyshared/rospkg/
sudo patch -p0 < ~/ros-linaro-build/rospkg/rospkg-linaro.patch
# TODO download patchset from somewhere

# perform dependency install ignoring errors
cd /opt/ros/groovy/catkin_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro groovy -yr
# rosdep check --from-paths src --ignore-src --rosdistro groovy

# Building Yaml-cpp via repo
echo "deb http://ppa.launchpad.net/stephane.magnenat/precise/ubuntu precise main
deb-src http://ppa.launchpad.net/stephane.magnenat/precise/ubuntu precise main" | sudo tee -a /etc/apt/sources.list
sudo apt-get update; sudo apt-get upgrade -y
sudo apt-get build-dep yaml-cpp
mkdir -p ~/fix/yaml-cpp
cd ~/fix/yaml-cpp
sudo apt-get source yaml-cpp
cd yaml-cpp-0.2.6
sudo dpkg-buildpackage -us -uc -nc
cd ..
sudo dpkg -i *.deb

#dummy yaml-cpp
mkdir -p ~/fix/yaml-cpp/
cp ~/ros-linaro-build/yaml-cpp/yaml-cpp ~/fix/yaml-cpp/yaml-cpp
cd ~/fix/yaml-cpp
sudo apt-get install equivs -y
equivs-build yaml-cpp
sudo dpkg -i yaml-cpp_1.0_all.deb


# patched tbb-dev
sudo mkdir -p /opt/intel/libtbb-dev
cd /opt/intel/libtbb-dev
sudo wget http://threadingbuildingblocks.org/sites/default/files/software_releases/source/tbb40_20120613oss_src.tgz
sudo tar xzvf tbb40_20120613oss_src.tgz
cd tbb40_20120613oss
sudo patch -p1 < ~/ros-linaro-build/tbb-dev/tbb40_20120613oss-0001-Endianness.patch
sudo patch -p1 < ~/ros-linaro-build/tbb-dev/tbb40_20120613oss-0002-ARM-support.patch
sudo patch -p1 < ~/ros-linaro-build/tbb-dev/tbb40_20120613oss-0003-Add-machine_fetchadd-48-intrinsics.patch
sudo make -j4
echo "source /opt/intel/libtbb-dev/tbb40_20120613oss/build/linux_armv7_gcc_cc4.6_libc2.15_kernel3.0.60_release/tbbvars.sh" >> ~/.bashrc


# removing unsupported packages
cd /opt/ros/groovy/catkin_ws/src
sudo rm -rf */
sudo patch -p0 <  ~/ros-linaro-build/ros-desktop-install/rosinstall.patch
sudo rosws update

# rosgraph/ifaddrs.py
cd /opt/ros/groovy/catkin_ws/src/rosgraph/src/rosgraph
sudo patch -p0 < ~/ros-linaro-build/rosgraph/rosgraph-ifaddrs.py.patch


# bulid catkin packages
cd /opt/ros/groovy/catkin_ws
sudo ./src/catkin/bin/catkin_make_isolated -j4 --install

# Rosbuild packages
# source /opt/ros/groovy/catkin_ws/install_isolated/setup.bash
# mkdir -p ~/rosbuild_ws
# rosws init ~/rosbuild_ws /opt/ros/groovy/catkin_ws
# cd ~/rosbuild_ws
# rosws merge http://packages.ros.org/web/rosinstall/generate/dry/raw/groovy/desktop
# rosws update -j4
# source ~/rosbuild_ws/setup.bash
# rosmake -a

# user overlay catkin
mkdir -p ~/catkin_overlay/src
source /opt/ros/groovy/catkin_ws/install_isolated/setup.bash
cd ~/catkin_overlay/src
wstool init . /opt/ros/groovy/catkin_ws/install_isolated

# user overlay rosbuild
mkdir ~/rosbuild_overlay
cd ~/rosbuild_overlay
rosws init . ~/catkin_overlay/devel


# rosjava
source ~/rosbuild_overlay/setup.bash
cd ~/rosbuild_overlay/
rosws merge http://rosjava.googlecode.com/hg/.rosinstall
rosws update rosjava_core
rospack profile

# download gradle
sudo mkdir -p /opt/gradle/
cd /opt/gradle/
sudo wget http://services.gradle.org/distributions/gradle-1.4-bin.zip
sudo unzip gradle-1.4-bin.zip

# download jdk1.8.0
sudo mkdir -p /opt/oracle/
cd /opt/oracle/
sudo wget http://www.java.net/download/JavaFXarm/jdk-8-ea-b36e-linux-arm-hflt-29_nov_2012.tar.gz
sudo tar xzvf jdk-8-ea-b36e-linux-arm-hflt-29_nov_2012.tar.gz
echo "export JAVA_HOME=/opt/oracle/jdk1.8.0
 PATH=.:$JAVA_HOME/bin:$JAVA_HOME/jre/bin:$PATH" >> ~/.bashrc


# building rosjava_core with gradle
source ~/rosbuild_overlay/setup.bash
roscd rosjava_core
/opt/gradle/gradle-1.4/bin/gradle install
rosmake rosjava_core
#todo rosdep install rosjava_core

# building april
source ~/rosbuild_overlay/setup.bash
cd ~/rosbuild_overlay
rosws merge http://utexas-ros-pkg.googlecode.com/svn/trunk/rosinstall/april.rosinstall
rosws update april
rospack profile
source ~/rosbuild_overlay/setup.bash
rosdep install april -yr

# patch april-tags-node
source ~/rosbuild_overlay/setup.bash
roscd april_tags_node
patch -p0 < ~/ros-linaro-build/april_tags_node/april_tags_node.patch
rosmake april

# download usb_cam
source ~/rosbuild_overlay/setup.bash
cd ~/rosbuild_overlay/
rosws set usb_cam --svn http://svn.code.sf.net/p/bosch-ros-pkg/code/trunk/stacks/bosch_drivers -y
source setup.bash
rosdep install usb_cam -yr