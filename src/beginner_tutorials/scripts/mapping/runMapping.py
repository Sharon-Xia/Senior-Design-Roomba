import rospy
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
SIDE_SENSOR_ANGLE = math.pi/3
SCAN_ANGLES = [0, SIDE_SENSOR_ANGLE, -SIDE_SENSOR_ANGLE]

# Scanning for Roombot mapping
# Inputs scanning data for map creation
class MapScan:

	# for ROS
	def __init__(self):
		tickers = [("scan", 20), ("odom", 20)]
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

			self.roomBot.updateWallCoordinate(theta, distance)


	# odom_msg: http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
	def odom_callback(self, odom_msg):
		if not self.ticker.tick("odom"):
			return

		# TODO
		# update real location in RoomBot
		# calculate accuracy

		realLoc = Location(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, datetime.now()) # TODO
		ddiff = self.roomBot.realLocation(realLoc)

		tanTheta = odom_msg.pose.pose.orientation.y / odom_msg.pose.pose.orientation.x
		realAngle = math.atan(tanTheta)
		adiff = self.roomBot.realAngle(realAngle)

		return


	def get_index_from_theta(self, data, theta):
		""" 
        theta in radians from front of car
        """
		return int((theta - data.angle_min)/data.angle_increment)


def main(args):
	rospy.init_node("Mapping_node", anonymous=True)
	wf = MapScan()
	atexit.register(wf.roomBot.map.generatePNG())
	rospy.sleep(0.1)
	rospy.spin()



if __name__=='__main__':
	main(sys.argv)


# also try -- except KeyboardInterrupt
