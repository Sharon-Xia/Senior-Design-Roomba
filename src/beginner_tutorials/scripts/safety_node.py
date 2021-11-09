#!/usr/bin/env python
import rospy

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from math import *
# TODO: import ROS msg types and libraries


class Safety(object):
    """
    The class that handles emergency braking.
    """
    safety_threshold = 10 # ttc threshold

    scan_sub_rate = 200 # once every 50 msgs
    scan_sub_tick = 0

    odom_sub_rate = 200
    odom_sub_tick = 0

    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        # TODO: create ROS subscribers and publishers.
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback) # current speed

        self.brake_pub = rospy.Publisher('/brake', AckermannDriveStamped, queue_size=10)
        self.brakeb_pub = rospy.Publisher('/brake_bool', Bool, queue_size=10)


    # note: doesn't update after crash in simulator
    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.odom_sub_tick += 1 

        if self.odom_sub_tick >= self.odom_sub_rate:
            rospy.loginfo("twist.linear.x, y, z: %f , %f, %f", odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z)
            rospy.loginfo("twist.angular.x, y, z: %f , %f, %f", odom_msg.twist.twist.angular.x, odom_msg.twist.twist.angular.y, odom_msg.twist.twist.angular.z)
            rospy.loginfo("pose.position.x, y, z: %f , %f, %f", odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z)
            rospy.loginfo("pose.orientation.x, y, z: %f , %f, %f", odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z)
            
            self.speed = sqrt(odom_msg.twist.twist.linear.x ** 2 + (odom_msg.twist.twist.linear.y ** 2))
            rospy.loginfo("current speed: %f", self.speed)

            self.odom_sub_tick = 0


    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        self.scan_sub_tick += 1

        if self.scan_sub_tick >= self.scan_sub_rate:
            TTC = 99999999
            ttc_theta = 0
            ttc_rd = 0
            ttc_range = []

            for i, d in enumerate(scan_msg.ranges): # index, distance
                theta = scan_msg.angle_min + (i * scan_msg.angle_increment)
                rd = self.speed * cos(theta) # rderivative, or vcostheta
                #rospy.loginfo("theta: %f, rd: %f", theta, rd)
                if (rd > 0):
                    ttc = d / (rd)
                    if (ttc < TTC):
                        TTC = ttc # ttc's
                        ttc_theta = theta
                        ttd_rd = rd

                    ttc_range.append(ttc)

            rospy.loginfo("TTC: %f", TTC)
            rospy.loginfo("TTC_theta: %f", ttc_theta)
            rospy.loginfo("TTC_rd: %f", ttc_rd)
            # rospy.loginfo(ttc_range)            

            # TODO: publish brake message and publish controller bool
            if (TTC < self.safety_threshold): # brake threshold exceeded
                rospy.loginfo("SAFETY THRESHOLD EXCEEDED, SETTING VELOCITY TO 0")
                control_msg = AckermannDriveStamped()
                control_msg.drive.speed = 0
                self.brake_pub.publish(control_msg)
                self.brakeb_pub.publish(True)
            else:
                #self.brakeb_pub.publish(False)
                one = 1

            self.scan_sub_tick = 0






def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()