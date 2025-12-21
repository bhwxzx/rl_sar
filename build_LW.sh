#!/bin/bash

# 对于jetson 需要 export IS_JETSON=true

source ./install/setup.bash
./build.sh serial
./build.sh fdilink_ahrs
./build.sh rl_sar