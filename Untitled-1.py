#!/usr/bin/env python2.7
#!pip install commands
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
import utm

from shapely.geometry import Point,Polygon
from shapely.wkt import loads

from std_msgs.msg import String, Float64
from mavros_main.msg import Escapee
from gps_common.msg import GPSFix
from geometry_msgs.msg import Twist
import time

class EscapeeNode:
    def __init__(self):

        # Create a polygon using the provided coordinates
        polygon_wkt = "POLYGON ((475962.7519573149 4934711.119215475, 475971.05994338985 4934711.577173998, 475981.6379940134 4934709.605181078, 475981.5094134559 4934698.475402928, 475962.48225412774 4934698.3459709175, 475955.4595843022 4934701.871042486, 475955.6938117249 4934709.35699199, 475962.7519573149 4934711.119215475))"
        self.polygon = loads(polygon_wkt)
        self.coords = list(self.polygon.exterior.coords)
        self.num_sides = len(self.coords) - 1

        # Calculate the centroid of the polygon using shapely
        self.centroid = self.polygon.centroid

        rospy.init_node('test_escapee', anonymous=True)
        self.name = rospy.get_param("~robot_name")
        rospy.Subscriber('/info', Escapee, self.info_callback)
       
        sub_name = "gps_fix_out"
        rospy.Subscriber(sub_name, GPSFix, self.update_gps_values)

        sub_name2 = "/" + self.name + "/mavros/global_position/compass_hdg"
        rospy.Subscriber(sub_name2, Float64, self.giveHeading)

        # Create a publisher for twist commands
        pub_name = "/" + self.name + "/cmd_vel"
        self.pub = rospy.Publisher(pub_name, Twist, queue_size=10)

        rate = rospy.Rate(1.0)  # Publish rate



    def info_callback(self, info_msg):
        # Print the mac,name,behavior_mode and the lat and lon from the info_msg
        rospy.loginfo("mac: %s", info_msg.mac.data)
        rospy.loginfo("name: %s", info_msg.name.data)
        rospy.loginfo("behavior_mode: %s", info_msg.behavior_mode.data)
        rospy.loginfo("lat: %f", info_msg.gps_loc.latitude)
        rospy.loginfo("lon: %f", info_msg.gps_loc.longitude)

    def update_gps_values(self,gps_msg):
        lat = gps_msg.latitude
        lon = gps_msg.longitude
        utm_coords = utm.from_latlon(lat,lon)  
        self.xx = utm_coords[0]
        self.yy = utm_coords[1]
        #rospy.loginfo("*********************************** %f, %f",lat,lon)
        rospy.loginfo("*********************************** %f, %f", self.xx , self.yy)


    def giveHeading(self,track):
        heading = track.data
        rospy.loginfo("------------------------------------- %f",heading)
        aa = 475962.7
        bb = 4934711.1
        self.haude(aa,bb,heading)

    def haude(self,x,y,head):
       
        point = Point(x,y)

        if not self.polygon.contains(point):
            rospy.loginfo("Point is outside the polygon")
            return

        distances = []
        for i in range(self.num_sides):
            x1,y1 = self.coords[i]
            x2,y2 = self.coords[i+1]
            A = y2 - y1
            B = x1 - x2
            C = (x2*y1) - (x1*y2)
            distance = abs((A*x) + (B*y) + C) / np.sqrt((A**2) + (B**2))
            distances.append(distance)
       
        min_distance = min(distances)
        threshold = 1
       
        if min_distance < threshold:
            rospy.loginfo("Distance from point to side :::::::::::: %f",min_distance)
           
            twist = Twist()
           
            angle = math.atan2(self.centroid.y - y, self.centroid.x - x)
           
            angular_velocity = angle - head
           
            linear_velocity = 0.5
           

            twist.linear.x = linear_velocity
            twist.angular.z = angular_velocity
            twist.linear.x = 0.1
            twist.angular.z = angular_velocity

            self.pub.publish(twist)
           





if __name__ == '__main__':
    try:
        node = EscapeeNode()
        # node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()