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
            int position, robot_id = checkPriority(goal, start_time);
            geometry_msgs::PoseStamped start;
            nav_msgs::Path path;
            ROS_INFO("which resource should take the task %d", robot_id);
            start = searchStartPose(robot_id, start_time);
            ROS_INFO("start: [x] %.2lf [y] %.2lf", start.pose.position.x, start.pose.position.y);
            position = checkPosition(robot_id, start_time);
            ROS_INFO("position %d", position);
            path = newPath(start, goal, start_time);
            if(path.poses.empty())
                return false;
            path_database_[robot_id].insert(path_database_[robot_id].begin() + position, path);
            if(path_database_[robot_id].size() > position)
            {
                 return updatePath(robot_id, position);
            }
            return true;
        }

        bool createPathTest()
        {
            srand(time(NULL));
            float time = rand() % 2000;
            geometry_msgs::PoseStamped goal = random_pose_stamped();
            return createPath(goal, time);
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
                start.pose.position.x = 0;
                start.pose.position.y = robot_id;
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
            if(path_database_[robot_id].size() >= path_id)
            {
                ROS_INFO("updating the path.");
                updatePath(robot_id, path_id);
            }
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
            //loop while time is not initialized properly
            while(ros::Time::now().toSec()==0)
            {}
            if(path_database_[robot_id].empty())
            {
                time = ros::Time::now().toSec();
            }
            else
            {
                time = path_database_[robot_id].back().poses.back().header.stamp.toSec() + 120;
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
                path = newPath(start, goal, time);    
            }
            return false;
        }

        //random path generator on a defined resource with a random start and goal position
        bool randomTestPath(int robot_id)
        {
            geometry_msgs::PoseStamped start, goal;
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
            //loop while time is not initialized properly
            while(ros::Time::now().toSec()==0)
            {}
            if(path_database_[robot_id].empty())
            {
                time = ros::Time::now().toSec();
            }
            else
            {
                time = path_database_[robot_id].back().poses.back().header.stamp.toSec() + 120;
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
                path = newPath(start, goal, time);    
            }
            return false;
        }

        bool checkAvailability(int robot_id, float time)
        {
            for (int i = 0; i < path_database_[robot_id].size(); i++)
            {
                //time is between last pose of i-1 and first pose of i -> available
                if(time < path_database_[robot_id][i].poses.front().header.stamp.toSec())
                    return true;
                //time is between the start and end of the focused path -> not available
                if(time > path_database_[robot_id][i].poses.front().header.stamp.toSec() && time < path_database_[robot_id][i].poses.back().header.stamp.toSec())
                    return false;
            }
            return true;
        }

        //Checks where a new path has to be inserted to secure the chronological order
        //returns the correct position for the new path at the transferred time
        int checkPosition(int robot_id, float start_time)
        {
            for (int i = 0; i < path_database_[robot_id].size(); i++)
            {
                if(start_time < path_database_[robot_id][i].poses.front().header.stamp.toSec())
                    return i;
            }
            return path_database_[robot_id].size();
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
        geometry_msgs::PoseStamped searchStartPose(int robot_id, float start_time)
        {
            geometry_msgs::PoseStamped start;
            int position = checkPosition(robot_id, start_time);
            if(path_database_[robot_id].empty() || position==0)
            {
                start.pose.position.x = 0;
                start.pose.position.y = robot_id; 
            } else {
                start = path_database_[robot_id][position - 1].poses.back();
            }
            return start;
        }

        //priority distribution - criteria are availability and number
        int checkPriority(geometry_msgs::PoseStamped goal, float start_time)
        {
            for (int i = 0; i < number_robots_; i++)
            {
                if(checkAvailability(i, start_time))
                    return i;
            }
            return -1;
            
        }







};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "database_actionserver_test");
    ros::NodeHandle n;

    path_database pdb(3);
    /*
    for (int i = 0; i < 10; i++)
    {
        if(pdb.randomTestPath(1))
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
    pdb.deletePath(1, 3);
    pdb.searchPath();

    
    for (int i = 0; i < 20; i++)
    {
        ROS_INFO("available at %d? %d", pdb.checkAvailability(1, i*100), i*100);
    }

    ROS_INFO("position at 500 %d",pdb.checkPosition(1, 500));
    ROS_INFO("position at 1000 %d",pdb.checkPosition(1, 1000));
    */

    for (int i = 0; i < 10; i++)
    {
        pdb.createPathTest();
    }
    
    pdb.searchPath();

    //ros::spin();

    return 0;
}

