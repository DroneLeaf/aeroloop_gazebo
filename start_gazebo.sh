#!/bin/bash
# For debugging - Updated for Gazebo Harmonic
export SDF_PATH=${PWD}/models:${SDF_PATH}
export GZ_SIM_RESOURCE_PATH=${PWD}/worlds:${GZ_SIM_RESOURCE_PATH}
export GZ_SIM_SYSTEM_PLUGIN_PATH=${PWD}/plugins/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
gz sim -r -v 4 -s --headless-rendering ${PWD}/worlds/$1
