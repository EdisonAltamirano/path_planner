#include "nav_msgs/Path.h"
#include "filename.h"


class path_database
{
    public:

        path_database(int #robots)
        {
            _#robots = #robots;
            
            
            
            
        }
    
    private:

        int _#robots;
        std::vector<std::vector<nav_msgs::Path>> _path_database;

}