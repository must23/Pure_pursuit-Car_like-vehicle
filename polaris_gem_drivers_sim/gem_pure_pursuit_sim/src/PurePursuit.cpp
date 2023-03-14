#include "PurePursuit.h"

float mdeglat(float lat)
{
    float latrad = lat*2.0*M_PI/360.0;
    float dy = 111132.09 - 566.05 * cos(2.0*latrad) + 1.20 * cos(4.0*latrad) - 0.002 * cos(6.0*latrad);
    return dy;
}

float mdeglon(float lat)
{
    float latrad = lat*2.0*M_PI/360.0;
    float dx = 111415.13 * cos(latrad) - 94.55 * cos(3.0*latrad)+ 0.12 * cos(5.0*latrad);

    return dx;
}

template <typename T>
float round_to(T value, int decimal)
{
    return round(value * pow(10, decimal))/float(pow(10, decimal));
}

float clip(float value, float min, float max)
{
    return (value < min)? min:((value > max)? max: value);
}

float dot(Point2D v1, Point2D v2)
{
    return v1.first * v2.first + v1.second * v2.second;
}

float cross(Point2D v1, Point2D v2)
{
    return v1.first * v2.second - v1.second * v2.first;
}

PurePursuit::PurePursuit() {

    readWaypoints();
    curr = f_vector (3, 0);

    ackermannMsgs = ackermann_msgs::AckermannDrive ();
    ackermannMsgs.steering_angle_velocity   = 0.;
    ackermannMsgs.acceleration              = 0.;
    ackermannMsgs.jerk                      = 0.;
    ackermannMsgs.speed                     = 0.;
    ackermannMsgs.steering_angle            = 0.;
    ros::NodeHandle handler;
    ackermanPub     = handler.advertise<ackermann_msgs::AckermannDrive>("/gem/ackermann_cmd", 1);
    crossErrors     = handler.advertise<std_msgs::Float32>("/cross_error", 1);
    gpsSubscriber   = handler.subscribe("/gem/gps/fix", 10, &PurePursuit::gpsPoseCallback, this);
    odmSubscriber   = handler.subscribe("/gem/base_footprint/odom", 10, &PurePursuit::odmYawCallback, this);

}

void PurePursuit::gpsPoseCallback(const sensor_msgs::NavSatFix& msg)
{
    /// Get pose from GPS
    float lat = msg.latitude;
    float lng = msg.longitude;
    curr[0] = (lng - ORIGIN_LNG) * mdeglon(ORIGIN_LAT);
    curr[1] = (lat - ORIGIN_LAT) * mdeglat(ORIGIN_LAT);

}

void PurePursuit::odmYawCallback(const nav_msgs::Odometry& msg) {
    tf::Quaternion quaternion(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
                );
    tf::Matrix3x3 eular(quaternion);
        double _, yaw;
        eular.getRPY(_,_,yaw);

    curr[2] = round_to(yaw,4);


    publish();
}

PurePursuit::~PurePursuit()= default;

void PurePursuit::readWaypoints()
{
    std::cout<<"============== Begin ============="<<std::endl;

    std::cout<<"Please input the directory for the given path csv file, e.g: /home/mustofa/TII_ws/src/POLARIS_GEM_e2/polaris_gem_drivers_sim/gem_pure_pursuit_sim/waypoints/wps.csv"<<std::endl;
    std::string filepath;
    std::getline(std::cin, filepath);
    
    std::ifstream file(filepath);
    
    std::string pointXCoor;
    std::string pointYCoor;
    std::string yaw;
    std::string IGNORE;

    int count = 0;
    if (file.is_open()){
        while(std::getline(file, pointXCoor, ','))
    {
        std::getline(file, pointYCoor, ',');
        std::getline(file, yaw, ',');
        std::getline(file, IGNORE, ',');
        std::getline(file, IGNORE);

        pathPointsXCoor.push_back(std::stof(pointXCoor));
        pathPointsYCoor.push_back(std::stof(pointYCoor));
        pathYaws.push_back(std::stof(yaw));
    }
    file.close();
    }
    
    else{
        std::cout << "Unable to open file" << std::endl;
    }    
    distance = std::vector<float>(pathPointsXCoor.size(), 0);

}

float PurePursuit::dist(Point2D point1, Point2D point2)
{
    return round_to(std::sqrt(std::pow(point1.first - point2.first, 2) + std::pow(point1.second -point2.second, 2)), 3);
}

float PurePursuit::findAngle(Point2D point1, Point2D point2)
{
    float cosAngle  = dot(point1, point2);
    float sinAngle  = std::norm(cross(point1, point2));

    return atan2(sinAngle, cosAngle);
}

void PurePursuit::publish()
{
    for(int i = 0; i < pathPointsXCoor.size(); i++)
    {
        distance[i] = dist(Point2D(pathPointsXCoor[i], pathPointsYCoor[i]), Point2D(curr[0], curr[1]));
        if(distance[i] < LOOK_AHEAD + 0.3 && distance[i] > LOOK_AHEAD - 0.3)
        {
            Point2D v1 {pathPointsXCoor[i] - curr[0], pathPointsYCoor[i] - curr[1]};
            Point2D v2 {cos(curr[2]), sin(curr[2])};

            float angle = findAngle(v1, v2);
            if (abs(angle) < M_PI/2.)
            {
                goal = i;
                break;
            }
        }
    }

    float alpha     = pathYaws[goal] - curr[2];
    float ct_error  = round_to(sin(alpha) * distance[goal], 3);
    std_msgs::Float32 errorMsg;
    errorMsg.data = std::abs(ct_error);

    std::cout<< "Crosstrack Error: " << ct_error<<std::endl;

    crossErrors.publish(errorMsg);

    ackermannMsgs.speed             = 2.8;
    ackermannMsgs.steering_angle    = round_to(clip(2 * atan((2* 0.285 * WHEEL_BASE * sin(alpha))/ distance[goal]), -0.61, 0.61), 3);
    ackermanPub.publish(ackermannMsgs);
}

const float PurePursuit::ORIGIN_LAT          = 40.093025;
const float PurePursuit::ORIGIN_LNG          = -88.235755;