#include "ros/ros.h"
#include "path_planning/time_service.h"
#include "path_planning/path_service.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "time_test");
    ros::NodeHandle n;

    ros::ServiceClient pclient = n.serviceClient<path_planning::path_service>("path_service");
    ros::ServiceClient client = n.serviceClient<path_planning::time_service>("time_service");

    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path_of_time_test", 1000);

    path_planning::time_service tsrv;
    path_planning::path_service psrv;

    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> sharedPose;
    geometry_msgs::PoseWithCovarianceStamped pose_curr;
    sharedPose = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/tb3_1/amcl_pose",n);
    if(sharedPose != NULL){
        pose_curr = *sharedPose;
    }

    psrv.request.start.pose = pose_curr.pose.pose;
    psrv.request.start.header.frame_id = "map";

    psrv.request.goal.header.frame_id = "map";
    srand(time(NULL));
    psrv.request.goal.pose.position.x = rand() % 6;
    psrv.request.goal.pose.position.y = rand() % 6;
    ROS_INFO("[x] %f, [y] %f", psrv.request.goal.pose.position.x, psrv.request.goal.pose.position.y);
    psrv.request.goal.pose.orientation.w = 1.0;

    if(pclient.call(psrv))
    {
        ROS_INFO("Path size: %lu\nPath published on the '/path_of_time_test' topic.\n", psrv.response.path.poses.size());

    } else {
        ROS_INFO("Path server failed.");
        return 1;
    }
    
    //server call
    tsrv.request.path = psrv.response.path;
    tsrv.request.startTime = ros::Time::now().toSec();
    tsrv.request.average_velocity = 0.5;

    if(client.call(tsrv))
    {
        ROS_INFO("End time(calculated): %f", tsrv.response.path_timestamped.poses[1].header.stamp.toSec());

    } else {
        ROS_INFO("Time server failed.");
        return 1;
    }
    

    while(ros::ok()){
        ros::spinOnce();
        path_pub.publish(psrv.response.path);
    }
    
    return 0;    

    
}