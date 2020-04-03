#!/bin/bash
time=$(date)
echo $time
echo $time >>  work_times.txt
git pull origin master