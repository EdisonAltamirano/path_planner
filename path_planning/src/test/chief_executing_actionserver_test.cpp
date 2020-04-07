#include "nav_msgs/Path.h"
#include "filename.h"


class path_database
{
    public:

        path_database(int #robots)
        {
            _#robots = #robots;
            for (int i = 0; i < #robots; i++)
            {
                std::vector<nav_msgs::Path> robot;
                _path_database.push_back(robot);
            }
            
        }

        /*CRUD
        create
        read
        update
        delete*/

        bool create_path_entry(nav_msgs::Path path, int robot_id)
        {

            _path_database[robot_id].push_back(path);
            return true;
            
        }
    
    private:

        int _#robots;
        std::vector<std::vector<nav_msgs::Path> > _path_database;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "chief_executing_test");
    ros::NodeHandle n;

    path_database(3);
    path_database.create_path_entry();

    ros::spin();

    return 0;
}

