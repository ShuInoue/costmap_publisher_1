#ifndef _COSTMAP_PUBLISHER_
#define _COSTMAP_PUBLISHER_

#include<ros/ros.h>
#include<nav_msgs/OccupancyGrid.h>
#include<map_msgs/OccupancyGridUpdate.h>
#include<ros/callback_queue.h>
#include<vector>

class costmap_publisher
{
    private:
        ros::NodeHandle publisher;
        ros::NodeHandle subscriber;
        ros::CallbackQueue map_queue;
        ros::Rate rate = 50;

        //map_msgs::OccupancyGridUpdate map_data;
        nav_msgs::OccupancyGrid map_data;
        map_msgs::OccupancyGridUpdate map_update;
        void cost_set(void);
    
    public:
        ros::Publisher cost_pub;
        ros::Subscriber map_sub;
        costmap_publisher();
        ~costmap_publisher();
        void mainloop(void);
        //void map_info_setter(const map_msgs::OccupancyGridUpdate::ConstPtr &map_msg);
        void map_info_setter(const nav_msgs::OccupancyGrid::ConstPtr &map_msg);
        void map_data_publisher(void);

};

#endif