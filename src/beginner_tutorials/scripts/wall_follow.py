#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import time

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


pi = math.pi

#PID CONTROL PARAMS
kp = 14#TODO
kd = .09#TODO
ki = 0#TODO
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
WALL_FOLLOW_THETA = pi/3 # 0 < THETA <= 70; 60, pi/6 from top = 0

ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

prev_velocity = 0.0
prev_ts = time.time()

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)#TODO: Subscribe to LIDAR
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10) #TODO: Publish to drive

    # getting the Range value from theta
    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # angle in radians
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement
        return data.ranges[self.get_index_from_theta(data, angle)]

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        #TODO: Use kp, ki & kd to implement a PID controller for 
        angle = (kp * error) + (kd * (prev_error - error)) + (ki * integral)


        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = self.get_velocity_from_steering_angle(angle)
        self.drive_pub.publish(drive_msg)

        # update for future calc
        prev_velocity = drive_msg.drive.speed 
        prev_error = error
        integral += error


    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm 
        #TODO:implement
        theta0 = (pi/2) - WALL_FOLLOW_THETA# wall_follow_theta converted to ros orientation
        a = self.getRange(data, theta0)
        b = self.getRange(data, pi/2)

        alpha = math.atan((a * math.cos(WALL_FOLLOW_THETA) - b)/(a * math.sin(WALL_FOLLOW_THETA)))
        Dt = b * math.cos(alpha)

        # calculate Dt+1
        ts = time.time()
        L = prev_velocity * (ts - prev_ts)
        Dt1 = L * math.sin(alpha)

        D = Dt + Dt1
        return DESIRED_DISTANCE_LEFT - D # e(t)

    def followRight(self, data, rightDist):
        """
        """
        return 0.0

    def lidar_callback(self, data):
        """ 
        """
        thetaAtLeft = pi/2
        leftDist = self.getRange(data, thetaAtLeft)

        error = self.followLeft(data, leftDist) #TODO: replace with error returned by followLeft
        #send error to pid_control
        self.pid_control(error, VELOCITY)


    def get_index_from_theta(self, data, theta):
        """ 
        theta in radians from front of car
        """
        return int((theta - data.angle_min)/data.angle_increment)



    def get_velocity_from_steering_angle(self, theta):
        """
        thresholds obtained from lab 3
        theta: radians
        return: m/s
        """
        if theta >= 0 and theta <= radians(10):
            return 1.5
        elif theta > radians(10) and theta <= radians(20): 
            return 1.0
        else: 
            return 0.5

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
