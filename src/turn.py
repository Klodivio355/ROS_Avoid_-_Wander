#! /usr/bin/env python
# coding=utf-8

import rospy
import math
from math import sqrt
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion
import numpy as np
import math as m

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

class Turn:
    def calc_range(self,x,y):
        return (x * x) + (y * y) 

    def stop_motion(self):
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.y = 0.0
        pub.publish(vel)

    def inactivity_track(self, data)

    def move_forward(self,distance):
        vel = Twist()
        vel.linear.x = 1
        while not rospy.is_shutdown():
           t0 = rospy.Time.now().to_sec()
           current_distance = 0
           #Loop to move the turtle in an specified distance
           while(current_distance < distance):
               #Publish the velocity
               pub.publish(vel)
               #Takes actual time to velocity calculus
               t1=rospy.Time.now().to_sec()
               #Calculates distancePoseStamped
               current_distance= vel.linear.x*(t1-t0)
           #After the loop, stops the robot
           vel.linear.x = 0
           #Force the robot to stop
           pub.publish(vel)

    def callback1(self, data):
        if data.data == True:
            self.callback2(True)
        else:
            self.callback1()

    def callback2(self,data, valid):
        if valid == True
            vel = Twist()
            t0 = rospy.Time.now().to_sec()
            current_angle = 0
            rad = abs(m.atan2(data.y, data.x))
            vel.angular.z = 1.0
            while not rospy.is_shutdown:
                while(current_angle < rad):
                    pub.publish(vel)
                    t1 = rospy.Time.now().to_sec()
                    current_angle = vel.angular.z*(t1-t0)
            vel.angular.z = 0
            pub.publish(vel)
            self.move_forward(self.calc_range(data.x,data.y))
        else
            self.callback2(False)

    def callback3(self, data):
        if data.data == True:
            self.stop_motion()
            self.callback2(True)
        else:
            self.callback3()
        
    def __init__(self):
        rospy.init_node('subscriber_node', anonymous=True)
        self.sub = rospy.Subscriber("collide_bool", Bool, self.callback1)
        self.sube = rospy.Subscriber("trigo_pro", Vector3, self.callback2)
        self.suber = rospy.Subscriber("distance_bool", Bool, self.callback3)

        rospy.loginfo("subscriber node is active...")

    def main_loop(self):
        rospy.spin()
        
if __name__ == '__main__':
    feelforce_instance = Turn()
    feelforce_instance.main_loop()