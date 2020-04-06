#!/bin/bash
echo "

                                     
    _/_/_/    _/      _/  _/_/_/_/   
   _/    _/    _/  _/    _/          
  _/_/_/        _/      _/_/_/       
 _/    _/      _/      _/            
_/_/_/        _/      _/_/_/_/       
                                     
                                     

"
time=$(date)
echo "termination time: ${time}"
echo "end ${time}" >> ~/catkin_ws/src/path_planning_system/documentation/work_times.txt
git add .
read -p "github comment: " comment
git commit -m "${comment}"
git push origin master