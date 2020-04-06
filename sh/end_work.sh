#!/bin/bash
echo "

__________                __________               
\______   \___.__. ____   \______   \___.__. ____  
 |    |  _<   |  |/ __ \   |    |  _<   |  |/ __ \ 
 |    |   \\___  \  ___/   |    |   \\___  \  ___/ 
 |______  // ____|\___  >  |______  // ____|\___  >
        \/ \/         \/          \/ \/         \/ 

"
time=$(date)
echo "termination time: ${time}"
echo "end ${time}" >> ~/catkin_ws/src/path_planning_system/documentation/work_times.txt
git add .
read -p "github comment: " comment
git commit -m "${comment}"
git push origin master