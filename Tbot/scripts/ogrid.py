#!/usr/bin/env python
"""
@gothWare
"""
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(msg):
    print("zero deg reading : {}\n".format(msg.ranges[0]))
    print("90 deg reading : {}\n".format(msg.ranges[90]))
    print("270 deg reading : {}\n".format(msg.ranges[270]))
    
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
    mov = Twist()
    for i in msg.ranges[0:120]:
        if i<0.6:
            mov.linear.x = 0
            mov.angular.z = 0.4
        else:
            mov.angular.z = 0
            mov.linear.x = 0.1
    for i in msg.ranges[240:360]:
        if i<0.6:
            mov.linear.x = 0
            mov.angular.z = -0.4
        else:
            mov.angular.z = 0
            mov.linear.x = 0.1

    pub.publish(mov)
    

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
