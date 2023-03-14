#!/usr/bin/env python3

#the output of trajectories for visual will be at "/path" topic
import rospy
import numpy as np
import os 
import csv
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker,MarkerArray
path_pub = rospy.Publisher('line_marker', Marker, queue_size=4)

def read_waypoints():

    dirname  = os.path.dirname(__file__)
    filename = os.path.join(dirname, '../waypoints/wps.csv')

    with open(filename) as f:
        path_points = [tuple(line) for line in csv.reader(f)]

    #print(path_points)
    show_marker (path_points)

def show_marker(observation):
  
   # rospy.loginfo('Publishing example line')
    #marked=MarkerArray()
  
    marker = Marker()
    marker.header.frame_id = "base_footprint"
    marker.id=1
    # marker.ns = "linear_constraints"
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD

    # marker scale
    marker.scale.x = 0.03
    marker.scale.y = 0.03
    marker.scale.z = 0.03

    # marker color
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    # marker orientaiton
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    #print(type(float(observation[0][0])))

    for i in range (len(observation)):
        
        line_list = Point()
        line_list.x = float(observation[i][0])
        line_list.y = float(observation[i][1])
        marker.points.append(line_list)
        
    path_pub.publish(marker)


if __name__ == '__main__':
    rospy.init_node('path', anonymous=True)

    while not rospy.is_shutdown():
        read_waypoints()

    #rospy.spin()