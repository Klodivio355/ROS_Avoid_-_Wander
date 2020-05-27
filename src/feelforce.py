#! /usr/bin/env python
# coding=utf-8

import rospy
import math
from math import sqrt
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion
import numpy as np

pub = rospy.Publisher('/trigo_pro', Vector3, queue_size=1)
ans = Vector3()

class Feelforce:
    def cosinus(self, data):
        return np.cos(np.radians(data)+np.pi/2)
    
    def sinuse(self, data):
        return np.sin(np.radians(data)+np.pi/2)

    def cal_opposite_pyth(self,cos_distance,hyp):
        if cos_distance >= 0:
            return sqrt(hyp*hyp - cos_distance*cos_distance)
        else:
            return -sqrt(hyp*hyp - cos_distance*cos_distance)

    def proper_mean(self,data):
        if np.mean(data) >= 1:
            return 1
        else:
            return np.mean(data)
    
    def callback(self, data):
        #rospy.sleep(1)
        vec_array_x = []
        vec_array_y = []
        for index in range(0,360,1):
            if data.ranges[index] != float("inf"):
                vec_array_x.append(self.cosinus(index))
                vec_array_y.append(self.sinuse(index))
        print 'cos :', -np.mean(vec_array_x), ' sin :', -np.mean(vec_array_y)
        ans.x = -np.mean(vec_array_x)
        ans.y = -np.mean(vec_array_y)
        pub.publish(ans)
        
    def __init__(self):
        rospy.init_node('subscriber_node', anonymous=True)
        self.sub = rospy.Subscriber("scan", LaserScan, self.callback)
        rospy.loginfo("subscriber node is active...")

    def main_loop(self):
        rospy.spin()
        

if __name__ == '__main__':
    feelforce_instance = Feelforce()
    feelforce_instance.main_loop()