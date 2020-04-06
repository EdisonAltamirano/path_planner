//Test node for the path server
//calls the planning service of the move_base and simultaneously of the path_server
//if the path_server plan is similar to the move_base plan it is likely that the robot takes this exact path if the plan is executed
//Corresponding files are path_test.launch and path_server_global_planner.cpp/path_server_navfn.cpp

#include "ros/ros.h"
#include "path_planning_system/path_service.h"
#include "nav_msgs/GetPlan.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_test");
    ros::NodeHandle n;

    ros::ServiceClient client_server = n.serviceClient<path_planning_system::path_service>("path_service");
    ros::ServiceClient client_move_base = n.serviceClient<nav_msgs::GetPlan>("/tb3_1/move_base/make_plan");

    //publisher for rviz
    ros::Publisher path_pub_server = n.advertise<nav_msgs::Path>("path_test_server", 1000);
    ros::Publisher path_pub_move_base = n.advertise<nav_msgs::Path>("path_test_move_base", 1000);

    path_planning_system::path_service service_path_server;
    nav_msgs::GetPlan service_move_base;

    //input check
    //goal and start position
    if(argc != 5 || isdigit(atoi(argv[1])) || isdigit(atoi(argv[2])) || isdigit(atoi(argv[3])) || isdigit(atoi(argv[4]))){
        ROS_INFO("Input Parameters are wrong!\nstart position:\nx\ny\ngoal position:\nx\ny");
        return 1;
    }

    //server call
    service_path_server.request.start.header.frame_id = "map";
    service_path_server.request.start.pose.position.x = atoi(argv[1]);
    service_path_server.request.start.pose.position.y = atoi(argv[2]);
    service_path_server.request.start.pose.orientation.w = 1.0;

    service_path_server.request.goal.header.frame_id = "map";
    service_path_server.request.goal.pose.position.x = atoi(argv[3]);
    service_path_server.request.goal.pose.position.y = atoi(argv[4]);
    service_path_server.request.goal.pose.orientation.w = 1.0;

    service_move_base.request.start = service_path_server.request.start;
    service_move_base.request.start.header.frame_id = "/tb3_1/map";
    service_move_base.request.goal = service_path_server.request.goal;
    service_move_base.request.goal.header.frame_id = "/tb3_1/map";

    if(client_server.call(service_path_server))
    {
        ROS_INFO("Path size: %lu\nPath published on the '/path_test_server' topic.\n", service_path_server.response.path.poses.size());

    } else {
        ROS_INFO("Path server failed.");
        return 1;
    }
    if(client_move_base.call(service_move_base))
    {
        service_move_base.response.plan.header.frame_id = "map";
        ROS_INFO("Path size: %lu\nPath published on the '/path_test_move_base' topic.\n", service_move_base.response.plan.poses.size());

    } else {
        ROS_INFO("Move base failed.");
        return 1;
    }


    while(ros::ok()){
        path_pub_server.publish(service_path_server.response.path);
        path_pub_move_base.publish(service_move_base.response.plan);
    }
    return 0;    

    
}