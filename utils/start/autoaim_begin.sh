#!/bin/bash

cd /home/spr/SPR-Vision-2026-main/Main_ws

source install/setup.bash 

ros2 launch rm_bringup bringup.launch.py

cd /home/spr/SPR-Vision-2026-main/utils/start/

./autoaim_begin1.sh
