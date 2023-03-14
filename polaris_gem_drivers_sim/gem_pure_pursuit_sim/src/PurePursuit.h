#ifndef GROUND_VEHICLE_CONTROL_PUREPURSUIT_H
#define GROUND_VEHICLE_CONTROL_PUREPURSUIT_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <fstream>
#include "nav_msgs/Odometry.h"
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_datatypes.h>
#include "sensor_msgs/NavSatFix.h"
#include <iostream>

typedef std::pair<float, float> Point2D;
typedef std::vector<float> f_vector;

class PurePursuit{
public:
    PurePursuit();
    ~PurePursuit();

    void readWaypoints();

    /**
     * Calculate ecludiean distance of two 2D points.
     * @param point1
     * @param point2
     * @return
     */
    float dist(Point2D point1, Point2D point2);

    float findAngle(Point2D point1, Point2D point2);
    
    void gpsPoseCallback(const sensor_msgs::NavSatFix &msg) ;
    void odmYawCallback(const nav_msgs::Odometry &msg);


private:

    short goal;
    ackermann_msgs::AckermannDrive ackermannMsgs;
   
    // Initialization for parameters
    const float LOOK_AHEAD          = 6.f;
    const float WHEEL_BASE          = 1.75f;
    // const std::string WPS_FILE_PATH = "/home/mustofa/TII_ws/src/POLARIS_GEM_e2/polaris_gem_drivers_sim/gem_pure_pursuit_sim/waypoints/wps.csv";
    const std::string ACKERMANN_CMD_TOPIC   = "/gem/ackermann_cmd";
    static const float ORIGIN_LAT;
    static const float ORIGIN_LNG;

    f_vector pathPointsXCoor;
    f_vector pathPointsYCoor;
    f_vector pathYaws;
    f_vector distance;
    f_vector curr;

    // Subscribers
    ros::Subscriber gpsSubscriber;
    ros::Subscriber odmSubscriber;
    
    // Publishers
    ros::Publisher ackermanPub;
    ros::Publisher crossErrors;
    void publish();
};


#endif //GROUND_VEHICLE_CONTROL_PUREPURSUIT_H