#! /usr/bin/env python
# coding=utf-8

import rospy
import math
from math import sqrt
from geometry_msgs.msg import Twist
from srv_examples.srv import SetBool, SetBoolResponse
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
import numpy as np

pube = rospy.Publisher('/collide_bool', Bool, queue_size=1)
#pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

class Collide:
    #def stop_motion(self):
     #   vel = Twist()
     #   vel.linear.x = 0.0
     #   pub.publish(vel)

    def callback(self, data):
        ans = Bool()
        i = 0 
        pube.publish(ans.data)
        ans.data = False
        for index in range(0,65,1):
            if data.ranges[index] != float("inf") and data.ranges[index] < 1 :
                print 'stop, object detected dead ahead left'
                ans.data = True 
                pube.publish(ans.data)
                break
            else: 
                for indexe in range(294,359,1):
                    if data.ranges[indexe] != float("inf") and data.ranges[indexe] < 1:     
                        print 'stop, object detected dead ahead right'
                        ans.data = True
                        pube.publish(ans.data)
                        break
                    else:
                        continue
                        
    def __init__(self):
        rospy.init_node('subscriber_node', anonymous=True)
        self.sub = rospy.Subscriber("scan", LaserScan, self.callback)
        rospy.loginfo("subscriber node is active...")

    def main_loop(self):
        rospy.spin()
        

if __name__ == '__main__':
    feelforce_instance = Collide()
    feelforce_instance.main_loop()