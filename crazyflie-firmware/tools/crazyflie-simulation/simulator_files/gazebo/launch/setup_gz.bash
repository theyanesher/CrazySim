#!/bin/bash

# Setup environment to make Crazysim visible to Gazebo.
# Derived from PX4 setup_gazebo.sh file

if [ "$#" != 2 ]; then
    echo -e "usage: source setup_gz.bash src_dir build_dir\n"
    return 1
fi

SRC_DIR=$1
BUILD_DIR=$2

# setup gz environment and update package path
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:${BUILD_DIR}/build_crazysim_gz
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:${SRC_DIR}/tools/crazyflie-simulation/simulator_files/gazebo/models:${SRC_DIR}/tools/crazyflie-simulation/simulator_files/gazebo/worlds
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${BUILD_DIR}/build_crazysim_gz