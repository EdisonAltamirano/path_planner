#!/bin/bash
time=$(date)
echo "

                                                      
    _/    _/  _/_/_/_/  _/        _/          _/_/    
   _/    _/  _/        _/        _/        _/    _/   
  _/_/_/_/  _/_/_/    _/        _/        _/    _/    
 _/    _/  _/        _/        _/        _/    _/     
_/    _/  _/_/_/_/  _/_/_/_/  _/_/_/_/    _/_/        
                                                      
                                                      

"
git pull origin master
echo $time
echo $time >>  ~/catkin_ws/src/path_planning_system/documentation/work_times.txt
