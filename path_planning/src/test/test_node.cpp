#include "ros/ros.h"
#include "time.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_node");
    
    ros::NodeHandle test_handle;

    ROS_INFO("Walltime: %lf, Rostime: %lf", ros::WallTime::now().toSec(), ros::Time::now().toSec());

    ROS_INFO("Walltime: %lf, Rostime: %lf", ros::WallTime::now().toSec(), ros::Time::now().toSec());

    ros::Rate sleepy(0.5);
    while (ros::ok())
    {
            ROS_INFO("Walltime: %lf, Rostime: %lf", ros::WallTime::now().toSec(), ros::Time::now().toSec());
            sleepy.sleep();

    }
    

    

    ros::spin();
    
    return 0;
}