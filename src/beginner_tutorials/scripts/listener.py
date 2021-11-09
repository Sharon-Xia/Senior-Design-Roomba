#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from beginner_tutorials.msg import scan_range
#from dynamic_reconfigure.parameter_generator import *


pub_closest = rospy.Publisher('/scan_range', scan_range, queue_size=10)
scan = scan_range()


def callback(data):
    ranges = [r for r in data.ranges if r < data.range_max and r > data.range_min]
    rospy.loginfo(rospy.get_caller_id() + ' closest_point :  %f', min(ranges))
    rospy.loginfo(rospy.get_caller_id() + ' farthest_point :  %f', max(ranges))
    rospy.loginfo(rospy.get_caller_id() + ' num_points: %f', len(data.ranges))
    rospy.loginfo(rospy.get_caller_id() + ' min_angle: %f', data.angle_min)
    rospy.loginfo(rospy.get_caller_id() + ' max_angle: %f', data.angle_max)
    rospy.loginfo(rospy.get_caller_id() + ' angle_increment: %f', data.angle_increment)

    scan.closest_point = min(ranges)
    scan.farthest_point = max(ranges)
    pub_closest.publish(scan)



def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/scan', LaserScan, callback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
