#!/usr/bin/env python
import rospy
from math import pow, atan2, sqrt, sin, cos


from pid import PID
from Snakegate import Snakegate

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
from matplotlib import style
import time as t
import cv2
from cv_bridge import CvBridge
import numpy as np


style.use("ggplot")


class Quadcopter:

    def __init__(self):
        rospy.init_node('quad_controller', anonymous=True)

        self.motor_pub = rospy.Publisher('/Kwad/joint_motor_controller/command', Float64MultiArray, queue_size=4)

        rospy.Subscriber('/gazebo/model_states',ModelStates,self.update_pose)
        rospy.Subscriber("/Kwad/Kwad/camera1/image_raw",Image,self.image_cap)
        
        self.bridge = CvBridge()
        self.pose = Pose()
        self.rate = rospy.Rate(10)
        self.Kp = 30
        self.quart = [0,0,0,0]
        self.rpy = [0,0,0]
        self.cx = 0
        self.cy = 0
        self.frame_cent_x = 0
        self.frame_cent_y = 0
        self.detection_Flag = False
    
    def image_cap(self,im):
        frame = self.bridge.imgmsg_to_cv2(im)
        self.frame_cent_x = frame.shape[0]//2
        self.frame_cent_y = frame.shape[1]//2
        self.cx , self.cy = Snakegate(frame,self.frame_cent_x,self.frame_cent_y)
        

    def update_pose(self, data):
        ind = data.name.index('Kwad')
        ##self.pose = data
        orientationObj = data.pose[ind].orientation
        positionObj = data.pose[ind].position
        self.pose.position.x = round(positionObj.x, 4)
        self.pose.position.y = round(positionObj.y, 4)
        self.pose.position.z = round(positionObj.z, 4)

        self.pose.orientation.x = round(orientationObj.x, 4)
        self.quart[0] = round(orientationObj.x, 4)
        self.pose.orientation.y = round(orientationObj.y, 4)
        self.quart[1] = round(orientationObj.y, 4)
        self.pose.orientation.z = round(orientationObj.z, 4)
        self.quart[2] = round(orientationObj.z, 4)
        self.pose.orientation.w = round(orientationObj.w, 4)
        self.quart[3] = round(orientationObj.w, 4)

    def VelocityPID(self, x_vel, y_vel, z_vel, x_setpt, y_setpt, z_setpt,f,thrust):
        Kp_v = 0.1
        er_x_vel = Kp_v*(x_setpt - x_vel)
        er_y_vel = Kp_v*(y_setpt - y_vel)
        er_z_vel = Kp_v*(z_setpt - z_vel)
        print([er_x_vel, er_y_vel])
        (r,p,y)  = (euler_from_quaternion(self.quart))

        f, eR, eP, eY = PID(r,p,y,f,thrust,roll_setpt=er_y_vel, pitch_setpt=er_x_vel, yaw_setpt=90)

        return f, eR, eP, eY

    def moveup(self,setpt):
        roll_array = []
        pitch_array = []
        yaw_array = []
        yarr = []
        vel_msg = Float64MultiArray()


        thrust = 0
        count = 0
        prev_x = self.pose.position.x
        prev_y = self.pose.position.y
        prev_z = self.pose.position.z

        prev_t  = 0
        t1 = t.time()
        #dT = 0.1
        for i in range(500):
            t2 = t.time()
            #dT = timeframe for velocity calc
            dT = t2-prev_t
            
            #ignore this section, its to fuck around with PID and tune Kp,Kd,Ki vals
            if (t2-t1)<20:
                roll_setpt = 0
                pitch_setpt = 0
                yaw_setpt = 0
                if (t2-t1)>5:
                    roll_setpt = 0
                    pitch_setpt = 0
                    yaw_setpt = 0
            else:
                roll_setpt = 0
                pitch_setpt = 0
                yaw_setpt = 0
            #stop ignoring from here
            #checking if detection is even there or nah, if its there, the new center coord != frame center coord.
            if abs(self.frame_cent_y-self.cy)>5:
                self.detection_Flag = True

            #an error from a set height to hover at while Window is being detected
            err = (setpt -  self.pose.position.z)

            #this is the main error vector we will use for control
            err_scalar = self.euclidean_distance([self.cx, self.cy], [self.frame_cent_x, self.frame_cent_y])
            err_angular = self.steering_angle([self.cx, self.cy], [self.frame_cent_x, self.frame_cent_y])

            print("error vector : {}".format([err_scalar,err_angular]))

            #basic shit, self explanatory
            dX = self.pose.position.x - prev_x
            dY = self.pose.position.y - prev_y
            dZ = self.pose.position.z - prev_z

            vel_array = [dX/dT ,dY/dT ,dZ/dT]

            prev_x = self.pose.position.x
            prev_y = self.pose.position.y
            prev_z = self.pose.position.z

            print("Velocities are {}".format(vel_array))
            print("PITCH SETPOINT IS {}".format(roll_setpt))

            #here we HOVER if window not detected, else use a standard V.cos(theta), V.sin(theta) relation to set vertical velocity V
            #V==0 iff img center==window center ie. Theta==0 ie. v.sin(theta)==0 ...thrust becomes 49, hovering setpoint, else self tune according to sine relation.
            if self.detection_Flag==True:
                thrust = 49 - sin(err_angular)
                if thrust>55:
                    thrust = 53
            else:
                if err>0:
                    thrust = 49 + 3*err
                    if thrust>55:
                        thrust = 53
                else:
                    thrust = 49 + 3*err
                    if abs(thrust)>55:
                        thrust = 45

            # converts Quarternion orientation system(ROS Default) to Euler orientations(roll,  pitch ,yaw), ez for processing
            (roll, pitch, yaw) = (euler_from_quaternion(self.quart))
            
            #this is actually OPTIONAL, FIRST line is used for controlling only R,P,Y space(slower response time, better for hover), SECOND transforms R,P,Y into Velocity domain control scheme(faster response)
            #vel_msg2, erR, erP, erY = PID(roll,pitch,yaw,vel_msg,thrust,roll_setpt,pitch_setpt,yaw_setpt)
            vel_msg2, erR, erP, erY = self.VelocityPID(vel_array[0], vel_array[1], vel_array[2] , pitch_setpt,roll_setpt,0, vel_msg, thrust)

            #if yall know ROS networks, this sends the data over to drone (simulation), HMU if yall wanna run the sim for yourself(Install ROS : Melodic)
            self.motor_pub.publish(vel_msg2)
            print("THRUST IS {}".format(thrust))
            print(self.pose)

            prev_t = t2
            yarr.append(count)
            count+=1
            #sum lines to plot response of PID
            roll_array.append(roll)
            pitch_array.append(pitch)
            yaw_array.append(yaw)
            self.rate.sleep()

        while thrust>0:
            (roll, pitch, yaw) = (euler_from_quaternion(self.quart))
            vel_msg2, erR, erP, erY = PID(roll,pitch,yaw,vel_msg,thrust,roll_setpt,pitch_setpt,yaw_setpt)

            self.motor_pub.publish(vel_msg2)
            print("landing")
            thrust-=0.5
            self.rate.sleep()
        
        vel_msg.data = [0,0,0,0]
        self.motor_pub.publish(vel_msg)

        plt.plot(yarr,pitch_array,label="velocity")
        plt.plot(yarr,roll_array,label='roll')
        #plt.plot(yarr,yaw_array,label='setpoint')
        plt.legend()
        plt.show()
        self.rate.sleep()


    def euclidean_distance(self, goal_pose, curr_pose):
        return sqrt( pow((goal_pose[0] - curr_pose[0]), 2) + pow((goal_pose[1] - curr_pose[1]), 2))

    # def linear_vel(self, goal_pose, constant=1.5):
    #     return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose, curr_pose):
        return atan2(goal_pose[1] - curr_pose[1], goal_pose[0] - curr_pose[0])

    # def angular_vel(self, goal_pose, constant=6):
    #     return constant * (self.steering_angle(goal_pose) - self.pose.theta)


if __name__ == '__main__':
    try:
        drone = Quadcopter()
        drone.moveup(0.1)


    except rospy.ROSInterruptException:
        pass
