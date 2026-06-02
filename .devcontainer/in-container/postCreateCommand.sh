#!/usr/bin/env bash

set -ex

echo "source /opt/ros/$ROS_DISTRO/setup.bash" >>~/.bashrc

# (Dumb) Workaround; Add the submodules to the list of safe directories for git
for d in /cws/src/FANUC-CORPORATION/fanuc_driver/.git/modules/fanuc_libs/dependencies/sockpp /cws/src/FANUC-CORPORATION/fanuc_driver/.git/modules/fanuc_libs/dependencies/readerwriterqueue /cws/src/FANUC-CORPORATION/fanuc_driver/.git/modules/fanuc_libs/dependencies/yaml-cpp /cws/src/FANUC-CORPORATION/fanuc_driver/.git/modules/fanuc_libs/dependencies/reflect-cpp;
do
  git config --global --add safe.directory $d;
done

# Temporarilly clone Moveit2 in order to enable Pilz POLYLINE that is not available in .deb yet.
cd /cws
apt update && apt install -y python3-pip
pip3 install vcs2l --break-system-packages
vcs import --input /tmp/in-container/dep.repos --recursive /cws/src

rosdep update && rosdep install --ignore-src --from-paths src -y

. /opt/ros/jazzy/setup.sh
colcon build --symlink-install --packages-up-to fanuc_moveit_config --cmake-args -DBUILD_EXAMPLES=OFF
. ./install/setup.sh"

echo "Done devcontainer's postCreateCommand!"
