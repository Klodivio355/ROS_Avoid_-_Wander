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

pub = rospy.Publisher('/distance_bool', Bool, queue_size=1)

class Runaway:

    def calc_range(self,data):
        return (data.x * data.x) + (data.y * data.y) 

    def callback(self, data):
        if self.calc_range(data) <= 0.9:
            print 'U can go! Too close'
            pub.publish(True)
        else:
            print 'Dont move, The object is quite far from you'
            pub.publish(False)
        
        
    def __init__(self):
        rospy.init_node('subscriber_node', anonymous=True)
        self.sub = rospy.Subscriber("trigo_pro", Vector3, self.callback)
        rospy.loginfo("subscriber node is active...")

    def main_loop(self):
        rospy.spin()
        

if __name__ == '__main__':
    feelforce_instance = Runaway()
    feelforce_instance.main_loop()