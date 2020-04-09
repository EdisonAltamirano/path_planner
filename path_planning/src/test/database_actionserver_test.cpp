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

        /*
        CRUD
        create
        read
        update
        delete
        */

        bool dummySchedule(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal)
        {
            return true;
        }

        //path with time and id
        bool createPathIDTimeStartGoal(int robot_id, float start_time, geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal)
        {
            path_database_[robot_id].push_back(createPath(start, goal));
            if(path_database_[robot_id].back().poses.size()==0)
            {
                ROS_ERROR("Path is empty.(createPathIDTimeStartGoal)");
                return false;
            }
            return true;
        }

        //random path generator on a random resource with a random start and goal position
        bool randomStartTestPath()
        {
            geometry_msgs::PoseStamped start, goal;
            srand(time(NULL));
            start = random_pose_stamped();
            goal = random_pose_stamped();
            nav_msgs::Path path = createPath(start, goal);

            for (int i = 0; i < 10; i++)
            {
                if(path.poses.size() > 0)
                {
                    path_database_[rand() % number_robots_].push_back(path);
                    return true;
                }
                start = random_pose_stamped();
                goal = random_pose_stamped();   
                path = createPath(start, goal);    
            }
            return false;
        }

        bool randomPosteriorPath()
        {
            geometry_msgs::PoseStamped start, goal;
            srand(time(NULL));
            int robot_id = rand() % number_robots_;
            start = path_database_[robot_id].back().poses.back();
            start.header.stamp = ros::Time::now();
            goal = random_pose_stamped();
            nav_msgs::Path path = createPath(start, goal);

            for (int i = 0; i < 10; i++)
            {
                if(path.poses.size() > 0)
                {
                    path_database_[robot_id].push_back(path);
                    return true;
                }
                goal = random_pose_stamped();   
                path = createPath(start, goal);    
            }
            return false;

        }

        //Display the scheduled paths
        bool read_schedule()
        {
            for (int i = 0; i < number_robots_; i++)
            {
                for (int j = 0; j < path_database_[i].size(); j++)
                {
                    ROS_INFO("Robot %u Path %u Start [x] %.2lf [y] %.2lf Goal [x] %.2lf [y] %.2lf Size %u",
                     i, j, path_database_[i][j].poses.front().pose.position.x, path_database_[i][j].poses.front().pose.position.y, path_database_[i][j].poses.back().pose.position.x, path_database_[i][j].poses.back().pose.position.y, path_database_[i][j].poses.size());
                }   
            }
            return true;
        }

    private:

        int number_robots_;
        int area_bound_ = 6;
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

        //random pose generation for automated test process
        geometry_msgs::PoseStamped random_pose_stamped()
        {
            geometry_msgs::PoseStamped random_pose;
            srand(time(NULL));
            random_pose.header.stamp = ros::Time::now();
            random_pose.header.frame_id = "map";
            random_pose.pose.position.x = rand() % area_bound_;
            random_pose.pose.position.y = rand() % area_bound_;
            return random_pose;
        }

        //find the start pose for paths depending on the schedule of the resource
        geometry_msgs::PoseStamped start_pose_detection(int resource_id, float start_time)
        {
            float chrono_distance;
            for (int i = 0; i < path_database_[resource_id].size(); i++)
            {
                
            }
            
            
        }



};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "database_actionserver_test");
    ros::NodeHandle n;

    path_database pdb(3);
    geometry_msgs::PoseStamped start, goal;
    start.pose.position.x = 0;
    start.pose.position.y = 0;
    goal.pose.position.x = 1;
    goal.pose.position.y = 1;
    pdb.createPathIDTimeStartGoal( 0, ros::Time::now().toSec(), start, goal);
    start.pose.position.x = 0;
    start.pose.position.y = 1;
    goal.pose.position.x = 2;
    goal.pose.position.y = 1;
    pdb.createPathIDTimeStartGoal( 1, ros::Time::now().toSec(), start, goal);
    start.pose.position.x = 0;
    start.pose.position.y = 2;
    goal.pose.position.x = 1;
    goal.pose.position.y = 2;
    pdb.createPathIDTimeStartGoal( 2, ros::Time::now().toSec(), start, goal);

    for (int i = 0; i < 5; i++)
    {
        if(pdb.randomPosteriorPath())
        {
            pdb.read_schedule();
            ROS_INFO("Sucess!");
        }
        else
        {
            ROS_INFO("Fail!");
        }
    }

    //ros::spin();

    return 0;
}

