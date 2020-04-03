#!/bin/bash
time=$(date)
echo $time
echo $time >>  ../documentation/work_times.txt
git pull origin master