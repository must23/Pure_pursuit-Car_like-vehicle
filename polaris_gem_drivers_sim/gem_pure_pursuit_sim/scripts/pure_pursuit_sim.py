#!/usr/bin/env python3

#================================================================
# File name: pure_pursuit_sim.py                                                                  
# Description: pure pursuit controller for GEM vehicle in Gazebo                                                              
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 07/10/2021                                                                
# Date last modified: 07/15/2021                                                          
# Version: 0.1                                                                    
# Usage: rosrun gem_pure_pursuit_sim pure_pursuit_sim.py                                                                    
# Python version: 3.8                                                             
#================================================================

# Python Headers
import os 
import csv
import math
import numpy as np
from numpy import linalg as la

# ROS Headers
import rospy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Gazebo Headers
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState

#########################################################################
#######To get NavSatFix and Odom#########################################
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import alvinxy.alvinxy as axy # Import AlvinXY transformation module
###########################################################################

class PurePursuit(object):
    
    def __init__(self):

        self.rate       = rospy.Rate(20)

        self.look_ahead = 6    # meters
        self.wheelbase  = 1.75 # meters
        self.offset     = 0.46 # meters

        self.gps_sub     = rospy.Subscriber("/gem/gps/fix", NavSatFix, self.gps_callback)
        self.lat        = 0.0
        self.lon        = 0.0

        self.olat       = 40.093025
        self.olon       = -88.235755

  

        self.odom_sub    = rospy.Subscriber("/gem/base_footprint/odom", Odometry, self.odom_callback)
     
        self.gazebo_yaw = 0.0
    



        self.goal       = 0

        self.read_waypoints() # read waypoints

        self.ackermann_msg = AckermannDrive()
        self.ackermann_msg.steering_angle_velocity = 0.0
        self.ackermann_msg.acceleration            = 0.0
        self.ackermann_msg.jerk                    = 0.0
        self.ackermann_msg.speed                   = 0.0 
        self.ackermann_msg.steering_angle          = 0.0

        self.ackermann_pub = rospy.Publisher('/gem/ackermann_cmd', AckermannDrive, queue_size=1)


    # import waypoints.csv into a list (path_points)
    def read_waypoints(self):

        dirname  = os.path.dirname(__file__)
        filename = os.path.join(dirname, '../waypoints/wps.csv')

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        # turn path_points into a list of floats to eliminate the need for casts
        self.path_points_x   = [float(point[0]) for point in path_points]
        self.path_points_y   = [float(point[1]) for point in path_points]
        self.path_points_yaw = [float(point[2]) for point in path_points]
        self.dist_arr        = np.zeros(len(self.path_points_x))
       
    # computes the Euclidean distance between two 2D points
    def dist(self, p1, p2):
        return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)

    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        # [-pi, pi]
        return np.arctan2(sinang, cosang)
 

#####################################################################################################
############## ADD NEW ###########################################################################
    
    def gps_callback(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude

    def wps_to_local_xy(self, lon_wp, lat_wp):
        # convert GNSS waypoints into local fixed frame reprented in x and y
        lon_wp_x, lat_wp_y = axy.ll2xy(lat_wp, lon_wp, self.olat, self.olon)
        return lon_wp_x, lat_wp_y   


    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.gazebo_yaw = yaw
        self.x_dot = msg.twist.twist.linear.x
        self.y_dot = msg.twist.twist.linear.y

    def get_pos_gps(self):
        
        local_x_curr, local_y_curr = self.wps_to_local_xy(self.lon, self.lat)
       
        # heading to yaw (degrees to radians)
        # heading is calculated from two GNSS antennas
        
        # reference point is located at the center of rear axle
       # curr_x = local_x_curr - self.offset * np.cos(curr_yaw)
        # curr_y = local_y_curr - self.offset * np.sin(curr_yaw)
        return round(local_x_curr, 4), round(local_y_curr, 4)
 ########################################################################################################         

    # def get_gem_pose(self):

    #     rospy.wait_for_service('/gazebo/get_model_state')

    #     try:
    #         service_response = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    #         model_state = service_response(model_name='gem')
    #     except rospy.ServiceException as exc:
    #         rospy.loginfo("Service did not process request: " + str(exc))

    #     x = model_state.pose.position.x
    #     y = model_state.pose.position.y

    #     orientation_q      = model_state.pose.orientation
    #     orientation_list   = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    #     (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    #     return round(x,4), round(y,4), round(yaw,4)


    def start_pp(self):
        
        while not rospy.is_shutdown():

            # get current position and orientation in the world frame
            curr_yaw = self.gazebo_yaw

            curr_x, curr_y = self.get_pos_gps()
            #print("from GPS:", curr_x, curr_y, curr_yaw)

            # gazebo_x, gazebo_y, gazebo_yaw = self.get_gem_pose()
            # print("from gazebo:", gazebo_x, gazebo_y, gazebo_yaw)

            self.path_points_x = np.array(self.path_points_x)
            self.path_points_y = np.array(self.path_points_y)
         
            
            # finding the distance of each way point from the current position
            for i in range(len(self.path_points_x)):
                self.dist_arr[i] = self.dist((self.path_points_x[i], self.path_points_y[i]), (curr_x, curr_y))

            # finding those points which are less than the look ahead distance (will be behind and ahead of the vehicle)
            goal_arr = np.where( (self.dist_arr < self.look_ahead + 0.3) & (self.dist_arr > self.look_ahead - 0.3) )[0]

            # finding the goal point which is the last in the set of points less than the lookahead distance
            for idx in goal_arr:
                v1 = [self.path_points_x[idx]-curr_x , self.path_points_y[idx]-curr_y]
                v2 = [np.cos(curr_yaw), np.sin(curr_yaw)]
                temp_angle = self.find_angle(v1,v2)
                if abs(temp_angle) < np.pi/2:
                    self.goal = idx
                    break

            # finding the distance between the goal point and the vehicle
            # true look-ahead distance between a waypoint and current position
            L = self.dist_arr[self.goal]





            # transforming the goal point into the vehicle coordinate frame 
            gvcx = self.path_points_x[self.goal] - curr_x
            gvcy = self.path_points_y[self.goal] - curr_y
            goal_x_veh_coord = gvcx*np.cos(curr_yaw) + gvcy*np.sin(curr_yaw)
            goal_y_veh_coord = gvcy*np.cos(curr_yaw) - gvcx*np.sin(curr_yaw)


            

            # find the curvature and the angle 
            alpha   = self.path_points_yaw[self.goal] - (curr_yaw)
            k       = 0.285
            angle_i = math.atan((2 * k * self.wheelbase * math.sin(alpha)) / L) 
            angle   = angle_i*2
            angle   = round(np.clip(angle, -0.61, 0.61), 3)

            ct_error = round(np.sin(alpha) * L, 3)

           # print("Crosstrack Error: " + str(ct_error))

            # implement constant pure pursuit controller
            self.ackermann_msg.speed          = 2.8
            self.ackermann_msg.steering_angle = angle
            self.ackermann_pub.publish(self.ackermann_msg)

            self.rate.sleep()

def pure_pursuit():

    rospy.init_node('pure_pursuit_sim_node', anonymous=True)
    pp = PurePursuit()

    try:
        pp.start_pp()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    pure_pursuit()

