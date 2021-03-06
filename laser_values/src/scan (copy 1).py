#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    # values at 0 degree
    print (msg.ranges[0])
    # values at 90 degree
    print (msg.ranges[180])
    # values at 180 degree
    print (msg.ranges[360])
    # values at 270 degree
    print (msg.ranges[540])
    

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
