//Test node for the path server
//calls the planning service of the move_base and simultaneously of the path_server
//if the path_server plan is similar to the move_base plan it is likely that the robot takes this exact path if the plan is executed
//Corresponding files are path_test.launch and path_server_global_planner.cpp/path_server_navfn.cpp

#include "ros/ros.h"
#include "path_planning/path_service.h"
#include "nav_msgs/GetPlan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h" 
#include <geometry_msgs/PoseWithCovarianceStamped.h>
double newx=0.0, newy=0.0, newz =0.0;
double resetx=0.0, resety=0.0, restz =0.0;
double oldx=0.0, oldy=0.0, oldz =0.0;
const float SAMPLE_TIME_S       = 1;
void newgoalcallback(const geometry_msgs::Vector3::ConstPtr& msg){
    newx = msg->x;
    newy = msg->y;
    newz =  msg->z;
}
void positioncallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    oldx = msg->pose.pose.position.x ;
    oldy = msg->pose.pose.position.y; 
    oldz = msg->pose.pose.position.z;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_test");
    ros::NodeHandle n;
    ros::Rate           cycle_rate(int(1 / SAMPLE_TIME_S));
    ros::ServiceClient client_server = n.serviceClient<path_planning::path_service>("path_service");
    ros::ServiceClient client_move_base = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    ros::Subscriber newgoal = n.subscribe("newcoordiantes",1000,newgoalcallback);
    ros::Subscriber newposition = n.subscribe("/amcl_pose",1000,positioncallback);
    //publisher for rviz
    ros::Publisher path_pub_server = n.advertise<nav_msgs::Path>("path_test_server", 1000);
    ros::Publisher path_pub_move_base = n.advertise<nav_msgs::Path>("path_test_move_base", 1000);

    path_planning::path_service service_path_server;
    nav_msgs::GetPlan service_move_base;

    //input check
    //goal and start position
    // if(argc != 5 || isdigit(atoi(argv[1])) || isdigit(atoi(argv[2])) || isdigit(atoi(argv[3])) || isdigit(atoi(argv[4]))){
    //     ROS_INFO("Input Parameters are wrong!\nstart position:\nx\ny\ngoal position:\nx\ny");
    //     return 1;
    // }





    while(ros::ok()){
        ros::spinOnce();
        if ((oldx!=0.0 && oldy !=0 && newx!=0.0 && newy!=0.0) && (newx!= resetx && newy!=resety)){
            //ROS_INFO("Input Parameters are wrong!\nstart position:\nx\ny\ngoal position:\nx\ny");
            // return 1;
            //server call
            ROS_INFO("New Coordinates");
            ROS_INFO("%f",newx);
            ROS_INFO("%f",newy);
            ROS_INFO("Old Coordinates");
            ROS_INFO("%f",oldx);
            ROS_INFO("%f",oldy);
            service_path_server.request.start.header.frame_id = "map";
            service_path_server.request.start.pose.position.x =oldx;
            service_path_server.request.start.pose.position.y = oldy;
            service_path_server.request.start.pose.orientation.w = 1.0;

            service_path_server.request.goal.header.frame_id = "map";
            service_path_server.request.goal.pose.position.x = newx;
            service_path_server.request.goal.pose.position.y = newy;
            service_path_server.request.goal.pose.orientation.w = 1.0;

            service_move_base.request.start = service_path_server.request.start;
            service_move_base.request.start.header.frame_id = "/map";
            service_move_base.request.goal = service_path_server.request.goal;
            service_move_base.request.goal.header.frame_id = "/map";
            resetx=newx;
            resety=newy;
        
            if(client_server.call(service_path_server))
            {
                ROS_INFO("Path size: %lu\nPath published on the '/path_test_server' topic.\n", service_path_server.response.path.poses.size());

            } else {
                ROS_INFO("Path server failed.");
                //return 1;
            }
            /*if(client_move_base.call(service_move_base))
            {
                service_move_base.response.plan.header.frame_id = "map";
                //ROS_INFO("Path size: %lu\nPath published on the '/path_test_move_base' topic.\n", service_move_base.response.plan.poses.size());

            } else {
                //ROS_INFO("Move base failed.");
                //return 1;
            }*/
            if(service_path_server.response.path.poses.size()!=0){
                path_pub_server.publish(service_path_server.response.path);
            }

            //path_pub_move_base.publish(service_move_base.response.plan);

        }

        cycle_rate.sleep();
    }
    return 0;    

    
}