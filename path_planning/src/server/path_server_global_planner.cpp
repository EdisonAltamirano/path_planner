
#include "ros/ros.h"
#include "path_planning/path_service.h"
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <global_planner/planner_core.h>

bool createPath(path_planning::path_service::Request &req, path_planning::path_service::Response &res)
{
    std::vector<geometry_msgs::PoseStamped> plan;
    //costmap
    tf::TransformListener tf(ros::Duration(10));
    costmap_2d::Costmap2DROS costmap ("costmap", tf);
    costmap.start();

    //global planner - global_planner
    global_planner::GlobalPlanner global_planner;
    global_planner.initialize("my_global_planner", &costmap); 
    global_planner.makePlan(req.start, req.goal, plan);

    //plan is a pose array
    res.path.poses = plan;
    res.path.header.frame_id = "map";
    
    ROS_INFO("plan size: %d", (int) res.path.poses.size());

    return true;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("path_service", createPath);
    ROS_INFO("Ready to simulate a path.");

    ros::spin();

    return 0;
}