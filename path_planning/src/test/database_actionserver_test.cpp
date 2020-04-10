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

        //CRUD operation - create: Creates a path and distributes it with a priority system
        bool createPath(geometry_msgs::PoseStamped goal, float start_time)
        {
            //TODO priority system
            //TODO start pose 
            //TODO newPath(start, goal, start_time)
            return true;
        }

        //CRUD operation - read: searches a path with the number in the robot queue
        void searchPath(int robot_id, int path_id)
        {
                    ROS_INFO("Robot %u Path %u Start [x] %.2lf [y] %.2lf Goal [x] %.2lf [y] %.2lf Size %4u [start_time] %4.2lf [end_time] %4.2lf",
                     robot_id, path_id, path_database_[robot_id][path_id].poses.front().pose.position.x, path_database_[robot_id][path_id].poses.front().pose.position.y, path_database_[robot_id][path_id].poses.back().pose.position.x, path_database_[robot_id][path_id].poses.back().pose.position.y, path_database_[robot_id][path_id].poses.size(), path_database_[robot_id][path_id].poses.front().header.stamp.toSec(), path_database_[robot_id][path_id].poses.back().header.stamp.toSec());
        }

        //CRUD operation - read: Writes the path queue for one robot
        void searchPath(int robot_id)
        {
            for (int i = 0; i < path_database_[robot_id].size(); i++)
            {
                searchPath(robot_id, i);
            }
            
        }

        //CRUD operation - read: Display the whole schedule for all robots
        void searchPath()
        {
            for (int i = 0; i < number_robots_; i++)
            {
                for (int j = 0; j < path_database_[i].size(); j++)
                {
                    searchPath(i, j);
                }   
            }
        }

        //CRUD operation - update: Update the posterior paths. Provisionally just updating the next path.
        bool updatePath(int robot_id, int path_id)
        {
            nav_msgs::Path path;
            geometry_msgs::PoseStamped start;
            if(path_id == 0)
            {
                start.pose.position.x = robot_id;
                start.pose.position.y = 0;
            }
            else
            {
                start = path_database_[robot_id][path_id - 1].poses.back();
            }
            path = newPath(start, path_database_[robot_id][path_id].poses.back(), path_database_[robot_id][path_id].poses.front().header.stamp.toSec());
            if(path.poses.empty())
                return false;
            path_database_[robot_id][path_id] = path;
            return true;
        }

        //CRUD operation - delete: Delete the path and update the remaining queue.
        bool deletePath(int robot_id, int path_id)
        {
            path_database_[robot_id].erase(path_database_[robot_id].begin() + path_id);
            if(path_database_[robot_id].size() > path_id)
            {
                ROS_INFO("updating the path.");
                updatePath(robot_id, path_id);
            }
        }

        //path with time and id
        bool planPathIDTimeStartGoal(int robot_id, float start_time, geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal)
        {
            path_database_[robot_id].push_back(planPath(start, goal));
            if(path_database_[robot_id].back().poses.size()==0)
            {
                ROS_ERROR("Path is empty.(planPathIDTimeStartGoal)");
                return false;
            }
            return true;
        }

        //random path generator on a random resource with a random start and goal position
        bool randomTestPath()
        {
            geometry_msgs::PoseStamped start, goal;
            srand(time(NULL));
            int robot_id = rand() % number_robots_;
            //if the robot has no planned paths use the start position, otherwise use the last position of the last planned path
            if(path_database_[robot_id].empty())
            {
                start.pose.position.x = 0;
                start.pose.position.y = robot_id; 
            }
            else
            {
                start = path_database_[robot_id].back().poses.back();
            }
            goal = random_pose_stamped();
            //time is rostime if it is the first planned path, otherwise it is the time of the last pose of the previous path + 60s
            float time;
            if(path_database_[robot_id].empty())
            {
                time = ros::Time::now().toSec();
            }
            else
            {
                time = path_database_[robot_id].back().poses.back().header.stamp.toSec() + 60;
            }
            nav_msgs::Path path = newPath(start, goal, time);

            //loop if path server fails
            for (int i = 0; i < 10; i++)
            {
                if(path.poses.size() > 0)
                {
                    path_database_[robot_id].push_back(path);
                    return true;
                }
                goal = random_pose_stamped();   
                path = newPath(start, goal, ros::Time::now().toSec());    
            }
            return false;
        }


    private:

        int number_robots_;
        int area_bound_ = 6;
        float average_velocity_ = 0.1;
        std::vector<std::vector<nav_msgs::Path> > path_database_;
        ros::NodeHandle n;
        ros::ServiceClient path_client_ = n.serviceClient<path_planning::path_service>("path_service");
        ros::ServiceClient time_client_ = n.serviceClient<path_planning::time_service>("time_service");

        nav_msgs::Path newPath(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal, float start_time)
        {
            nav_msgs::Path path;
            path = planPath(start, goal);
            if(path.poses.empty())return path;
            path = timestampPath(path, start_time);
            return path;
        }

        nav_msgs::Path planPath(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal)
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

        nav_msgs::Path timestampPath(nav_msgs::Path path, float start_time)
        {
            path_planning::time_service t_srv;

            t_srv.request.startTime = start_time;
            t_srv.request.average_velocity = average_velocity_;
            t_srv.request.path = path;

            if(time_client_.call(t_srv))
            {
                ROS_INFO("Time server succeeded. [start_time] %f [end_time] %f", t_srv.response.path_timestamped.poses.front().header.stamp.toSec(), t_srv.response.path_timestamped.poses.back().header.stamp.toSec());
                return t_srv.response.path_timestamped;
            }
            else
            {
                ROS_ERROR("Time server failed.");
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
    for (int i = 0; i < 5; i++)
    {
        if(pdb.randomTestPath())
        {
            pdb.searchPath();
            ROS_INFO("Sucess!");
        }
        else
        {
            ROS_INFO("Fail!");
        }
    }

    pdb.deletePath(1, 0);
    pdb.searchPath();

    //ros::spin();

    return 0;
}

