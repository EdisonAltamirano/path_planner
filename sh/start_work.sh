#!/bin/bash
time=$(date)
echo $time
echo $time >>  ~/catkin_ws/src/path_planning_system/documentation/work_times.txt
git pull origin master