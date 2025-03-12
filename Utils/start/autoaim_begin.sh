#!/bin/bash

cd /home/spr/Desktop/rm_vision_sentry/

source install/setup.bash 

ros2 launch rm_vision_bringup vision_bringup.launch.py

cd /home/spr/Desktop/SPR_utils/start/

./autoaim_begin1.sh

