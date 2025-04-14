#!/bin/bash

cd /home/spr/SPR-Vision-2025/Main_ws

source install/setup.bash 

ros2 launch rm_bringup bringup.launch_mvtest.py

cd /home/spr/SPR-Vision-2025/Utils/start/

./autoaim_begin1.sh