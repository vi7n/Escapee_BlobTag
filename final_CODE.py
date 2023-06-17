#!/usr/bin/env python2.7
#!pip install commands
import rospy
import math
import numpy as np
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
      self.behavior_mode = "noinfo"
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
      if info_msg.name.data == self.name:
         if info_msg.behavior_mode.data == "tagger":
            self.behavior_mode = "tagger"
            rospy.loginfo("MY behavior_mode: %s", self.behavior_mode)
         elif info_msg.behavior_mode.data == "escapee":
            self.behavior_mode = "escapee"
            rospy.loginfo("MY behavior_mode: %s", self.behavior_mode)
         else:
            rospy.loginfo("BLANK")
         #rospy.loginfo("MY behavior_mode: %s", self.behavior_mode)
         #rospy.loginfo("mac: %s", info_msg.mac.data)
         #rospy.loginfo("name: %s", info_msg.name.data)
         rospy.loginfo("behavior_mode:%s", info_msg.behavior_mode.data)
         #rospy.loginfo("lat: %f", info_msg.gps_loc.latitude)
         #rospy.loginfo("lon: %f", info_msg.gps_loc.longitude)
      if info_msg.name.data != self.name:
         utm_coords_other = utm.from_latlon(info_msg.gps_loc.latitude,info_msg.gps_loc.longitude)
         self.xx_other = utm_coords_other[0]
         self.yy_other = utm_coords_other[1]

   def update_gps_values(self,gps_msg):
      lat = gps_msg.latitude
      lon = gps_msg.longitude
      utm_coords = utm.from_latlon(lat,lon)  
      self.xx = utm_coords[0]
      self.yy = utm_coords[1]
      #rospy.loginfo("*********************************** %f, %f",lat,lon)
      #rospy.loginfo("*********************************** %f, %f", self.xx , self.yy)


   def giveHeading(self,track):
      heading = track.data

      #HEADING CORRECTION IN DEGREES, different for different robots
      zero_error = -11 #corrention needed(can be positive or negative)
      heading = heading + zero_error
      rospy.loginfo("------------------------------------- %f",heading)

      #TEST OTM POINT ON LINE: WORKS!!!!!
      #self.xx = 475962.7
      #self.yy = 4934711.1
      #self.haude(heading)


      #CALL THE MAIN FUNCTION AND START THE MAIN LOOP:
      if self.behavior_mode == "escapee":
         self.haude(heading)
      elif self.behavior_mode == "tagger":
         self.ipidipode(heading)

   #ESCAPEE BEHAVIOR
   def haude(self,head):
      x = self.xx
      y = self.yy      
      point = Point(x,y)

      if not self.polygon.contains(point):
         #rospy.loginfo("Point is outside the polygon")
         robot_outside = 1
      else:
         robot_outside = 0

      #geoboundary
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
      twist = Twist()
      threshold = 1
      linear_velocity = 0.75
      angular_velocity = 0
      ang = 0

      if min_distance < threshold or robot_outside == 1:
         #rospy.loginfo("Distance from point to side ::::::::::::::::: %f",min_distance)

         angle = math.atan2(self.centroid.y - y, self.centroid.x - x)

         ang = angle - head
         #slow at the boundary
         linear_velocity = 0.25
      distance_to_other_robot = math.sqrt((x - self.xx_other)**2 + (y - self.yy_other)**2)

      if distance_to_other_robot <= 0.75:
         other_angle = math.atan2(self.yy_other - y, self.xx_other - x)
         dif_angle = head - other_angle
         if abs(dif_angle) <= 40:
            if dif_angle > 0:
               ang = ang + (0.9*(90-dif_angle))
            if dif_angle < 0:
               ang = ang - (0.9*(90-dif_angle))

      #angle normalizing code
      if ang > 0:
         ang = ang % 360
         if ang > 180:
            ang = ang % -360
      if ang < 0:
         ang = ang % -360
         if ang < -180:
            ang = ang % 360          


      twist.linear.x = linear_velocity



      angular_velocity = math.radians(angle)

      twist.angular.z = angular_velocity
      #rospy.loginfo("YOOOOOOOOOOOOOOOOOOOOOOO ::::::::::::::::: %f and in deg: %f of rad: %f",linear_velocity, ang, angular_velocity)

      #TEST VALUES: WORKS!
      #twist.linear.x = 0.25
      #twist.angular.z = 0

      self.pub.publish(twist)

   #TAGGER BEHAVIOR          
   def ipidipode(self,head):
      x = self.xx
      y = self.yy      
      point = Point(x,y)

      if not self.polygon.contains(point):
         #rospy.loginfo("Point is outside the polygon")
         robot_outside = 1
      else:
         robot_outside = 0

      #GEOBOUNDARY
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
      twist = Twist()
      threshold = 0.1
      linear_velocity = 0.75

      if min_distance < threshold or robot_outside == 1:

         #rospy.loginfo("Distance from point to side ::::::::::::::::: %f",min_distance)
         linear_velocity = 0.2

         twist = Twist()

         angle = math.atan2(self.centroid.y - y, self.centroid.x - x)

         ang = angle - head

      distance_to_other_robot = math.sqrt((self.xx - self.xx_other)**2 + (self.yy - self.yy_other)**2)

         #pursuer code
      if distance_to_other_robot <= 2:
         other_angle = math.atan2(self.yy_other - self.yy, self.xx_other - self.xx)
         dif_angle = head - other_angle
         if abs(dif_angle) <= 45:
            ang = - dif_angle

      if distance_to_other_robot <= 0.5:
         other_angle = math.atan2(self.yy_other - self.yy, self.xx_other - self.xx)
         dif_angle = head - other_angle
         if abs(dif_angle) <= 40:
            if dif_angle > 0:
               ang = ang + (0.6*(90-dif_angle))
            if dif_angle < 0:
               ang = ang - (0.6*(90-dif_angle))
               #repulsive_force_y = (self.yy - self.yy_other) / distance_to_other_robot**2
               #ang += math.atan2(repulsive_force_y,repulsive_force_x) - head




      #normalizing code
      if ang > 0:
         ang = ang % 360
         if ang > 180:
            ang = ang % -360
      if ang < 0:
         ang = ang % -360
         if ang < -180:
            ang = ang % 360

      twist.linear.x = linear_velocity

      angular_velocity = math.radians(angle)
      twist.angular.z = angular_velocity

      #rospy.loginfo("YOOOOOOOOOOOOOOOOOOOOOOO ::::::::::::::::: %f and in deg: %f of rad: %f",linear_velocity, ang, angular_velocity)

      #TEST VALUES: WORKS!
      #twist.linear.x = 0.25
      #twist.angular.z = 0

      self.pub.publish(twist)




if __name__ == '__main__':
   try:
      node = EscapeeNode()
      # node.run()
   except rospy.ROSInterruptException:
      pass
   rospy.spin()