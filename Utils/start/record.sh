#!/bin/bash

cd /home/spr/Desktop/rm_vision_sentry/

source install/setup.bash 

sleep 30

ros2 bag record /debug_img /armor_detector_marker /armor_solver_marker
