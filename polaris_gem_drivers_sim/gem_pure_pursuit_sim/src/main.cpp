#include "PurePursuit.h"
#include <ros/ros.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "pure_pursuit_sim_node");
    PurePursuit p;
    ros::Rate r(10);

    while(ros::ok())
    {
        ros::spin();
        r.sleep();
    }
    return 0;
}