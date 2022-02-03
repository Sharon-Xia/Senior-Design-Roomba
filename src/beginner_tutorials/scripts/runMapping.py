#!/usr/bin/env python


import rospy
from tf.transformations import euler_from_quaternion
import sys

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from roombot import *
from location import *
from map import *
from tickCounter import *

import math
import atexit # call function on exit

# front of vehicle: radians = 0

# left of vehicle: radians > 0
# right of vehicle: radians < 0
SIDE_SENSOR_ANGLE = math.pi/2
SCAN_ANGLES = [0, SIDE_SENSOR_ANGLE, -SIDE_SENSOR_ANGLE]

# distances above sensitivity are ignored from map
SCAN_SENSITIVITY = 10

# Scanning for Roombot mapping
# Inputs scanning data for map creation
class MapScan:

	# for ROS
	def __init__(self):
		tickers = [("scan", 40), ("odom", 40)]
		self.ticker = TickCounter(tickers)

		#Topics & Subs, Pubs
		self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)#TODO: Subscribe to LIDAR
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback) # current speed


		# Mapping stuff
		self.roomBot = Roombot("test-run")



	# scan_msg: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html 
	
	def scan_callback(self, scan_msg):
		if not self.ticker.tick("scan"):
			return

		# update Vehicle's position
		self.roomBot.updatePath()

		# TODO
		for theta in SCAN_ANGLES:
			i = self.get_index_from_theta(scan_msg, theta)
			distance = scan_msg.ranges[i]

			#rospy.loginfo("distance: " + str(distance))
			if distance < SCAN_SENSITIVITY:
				self.roomBot.updateWallCoordinate(theta, distance)


	# odom_msg: http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
	def odom_callback(self, odom_msg):
		if not self.ticker.tick("odom"):
			return

		# TODO

		rospy.loginfo("")

		# calculated orientations
		# update distance 
		prevTs = self.roomBot.timeOfLastUpdate
		ts = self.roomBot.updateTs()

		speed = math.sqrt(odom_msg.twist.twist.linear.x ** 2 + (odom_msg.twist.twist.linear.y ** 2))
		if odom_msg.twist.twist.linear.y < 0: 
			speed *= -1

		# in seconds
		timePassed = (ts - prevTs).total_seconds()


		distance = timePassed * speed
		self.roomBot.updateDistanceTraveled(distance)

		# update angle 
		deltaTheta = (odom_msg.twist.twist.angular.z) * timePassed
		self.roomBot.updateAngle(deltaTheta)

		rospy.loginfo("linear.x: %f, linear.y: %f, angular.z: %f", \
			odom_msg.twist.twist.linear.x, \
			odom_msg.twist.twist.linear.y, \
			odom_msg.twist.twist.angular.z)

		#rospy.loginfo("speed: %f, time passed: %f, distance traveled: %f, deltaTheta: %f", speed, timePassed, distance, deltaTheta)


		# TODO : TEST CALCULATED ORIENTATIONS
		# update real location in RoomBot
		# calculate accuracy
		x = odom_msg.pose.pose.position.x
		y = odom_msg.pose.pose.position.y
		realLoc = Location(-y, x, datetime.now()) # TODO
		ddiff = self.roomBot.updateRealLocation(realLoc)

		orientation = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
		(roll, pitch, yaw) = euler_from_quaternion(orientation)

		realAngle = self.yaw_to_theta(yaw)
		adiff = self.roomBot.updateRealAngle(realAngle)

		self.roomBot.updateRecord()

		return


	def get_index_from_theta(self, data, theta):
		""" 
		theta in radians from front of car
		"""
		return int((theta - data.angle_min)/data.angle_increment)


	def yaw_to_theta(self, yaw):
		theta = math.pi/2 + yaw
		return theta if theta < math.pi else theta - (2 * math.pi)


def main(args):
	rospy.init_node("Mapping_node", anonymous=True)
	wf = MapScan()
	atexit.register(wf.roomBot.end)

	try:
		rospy.sleep(0.1)
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("generating image")
		wf.roomBot.map.generatePNG()



if __name__=='__main__':
	main(sys.argv)


# also try -- except KeyboardInterrupt


#   roslaunch f1tenth_simulator simulator.launch