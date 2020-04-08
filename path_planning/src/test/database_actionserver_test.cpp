//Database Actionserver
//Handles the organization and storage of the path schedule for a dynamic number of robots
//Middleware connected with the time_ and path_server
#include "ros/ros.h"
#include "path_planning/time_service.h"
#include "path_planning/path_service.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "nav_msgs/Path.h"
#include <stdlib.h>



class path_database
{
    public:

        path_database(int number_robots)
        {
            number_robots_ = number_robots;
            for (int i = 0; i < number_robots; i++)
            {
                std::vector<nav_msgs::Path> robot;
                path_database_.push_back(robot);
            }
            
        }

        /*CRUD
        create
        read
        update
        delete*/

        bool new_path(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal)
        {
            return true;
        }

        bool random_test_path()
        {
            srand(time(NULL));
            geometry_msgs::PoseStamped start, goal;
            start.header.frame_id = "map";
            start.pose.position.x = rand() % 6;
            start.pose.position.y = rand() % 6;
            start.header.stamp = ros::Time::now();
            goal.header.frame_id = "map";
            goal.pose.position.x = rand() % 6;
            goal.pose.position.y = rand() % 6;
            nav_msgs::Path path = createPath(start, goal);
            if(path.poses.size() <= 0)
                return false;
            path_database_[rand() % number_robots_].push_back(path);
            return true;
      
        }

    private:

        int number_robots_;
        std::vector<std::vector<nav_msgs::Path> > path_database_;
        ros::NodeHandle n;
        ros::ServiceClient path_client_ = n.serviceClient<path_planning::path_service>("path_service");
        ros::ServiceClient time_client_ = n.serviceClient<path_planning::time_service>("time_service");

        nav_msgs::Path createPath(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal)
        {
            path_planning::path_service p_srv;

            p_srv.request.start = start;
            p_srv.request.start.header.frame_id = "map";
            p_srv.request.goal = goal;
            p_srv.request.goal.header.frame_id = "map";

            if(path_client_.call(p_srv))
            {
                ROS_INFO("Path server succeeded. [path_size] %u", p_srv.response.path.poses.size());
                return p_srv.response.path;
            } else {
                ROS_INFO("Path server failed.");
                nav_msgs::Path fail;
                return fail;
            }
        }



};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "chief_executing_test");
    ros::NodeHandle n;

    path_database pdb(3);
    if(pdb.random_test_path())
    {
        ROS_INFO("Sucessssss!");
    }
    else
    {
        ROS_INFO("Fail!");
    }

    ros::spin();

    return 0;
}

