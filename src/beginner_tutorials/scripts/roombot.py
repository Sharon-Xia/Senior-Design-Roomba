#!/usr/bin/env python
import rospy
import math


from map import *
from location import *
from datetime import datetime


# stores information on the vehicle
# including location, sensors..etc.
# also calculates map points with sensors if map present

# constants
pathThreshold = .3 # amount of noise to ignore for path traveled in ?? units

class Roombot:

	def __init__(self, nameOfRun):
		self.map = Map(nameOfRun) 

		self.distanceTraveled = 0.0
		self.currentAngle = 0.0

		# keep track of delta(location)
		self.deltaDistance = 0.0
		self.deltaAngle = 0.0
		self.timeOfLastUpdate = datetime.now()


		# ros only - check accuracy
		self.realLocation = Location(0, 0, datetime.now()) # TODO
		self.realAngle = 0

	# records
	def updatePath(self):
		currentLoc = self.updatedLocation()
		self.currentAngle += self.deltaAngle
		
		self.resetDelta()

		# append if difference great enough
		if self.map.getLastLocation().distanceFrom(currentLoc) > pathThreshold:
			self.map.addPathTraveled(currentLoc)

		return currentLoc # compare with actual location in ROS

	# calculates current location & updates trackers
	# returns current location
	def updatedLocation(self):
		prevLoc = self.map.getLastLocation()
		angle = self.currentAngle # or self.currentAngle + (self.deltaAngle/2)

		x = prevLoc.x() + (math.cos(angle) * self.deltaDistance)
		y = prevLoc.y() + (math.sin(angle) * self.deltaDistance)

		ts = datetime.now()
		self.timeOfLastUpdate = ts

		currentLoc = Location(x, y, ts)

		return currentLoc


	def resetDelta(self):
		self.deltaAngle = 0.0
		self.deltaDistance = 0.0


	"""
	The following functions update vehicle distance and angle for
	its position to be calculated later on with the updatedLocation() function
	"""

	# amount of vehicle turn
	def updateAngle(self, theta):
		self.deltaAngle += theta

	# amount of vehicle travel
	def updateDistanceTraveled(self, distance):
		self.deltaDistance += distance



	"""
	ROS version of getting sensor data
	"""

	# calculates wall coordinate given scan information
	# theta: radians of sensor in relation to vehicle orientation
	# distance: how far the wall/obstacle is from the sensor
	# output: wall coordinate in (x, y) form - recorded in map
	def updateWallCoordinate(self, theta, distance):
		
		# ensure vehicle position is updated before calling this function
		currentLocation = self.map.getLastLocation()
		
		x = distance * math.cos(theta + self.currentAngle) + currentLocation.x()
		y = distance * math.sin(theta + self.currentAngle) + currentLocation.y()

		wallCoordinate = Location(x, y, self.timeOfLastUpdate)
		self.map.addMapCoordinate(wallCoordinate)


	# updates real location
	# compares calculated location for accuracy
	def updateRealLocation(self, realLoc):
		self.realLocation = realLoc

		# TODO
		currentLoc = self.map.getLastLocation()

		xdiff = currentLoc.compareX(realLoc)
		ydiff = currentLoc.compareY(realLoc)
		ddiff = currentLoc.distanceFrom(realLoc)

		rospy.loginfo("currentLoc: " + str(currentLoc) + "   realLoc: " + str(realLoc))
		rospy.loginfo("x-diff: {xdiff}  y-diff: {ydiff}  distance-diff: {ddiff}")

		return ddiff

	def updateRealAngle(self, realAngle):
		self.realAngle = realAngle

		adiff = self.currentAngle - self.realAngle

		# TODO
		rospy.loginfo("angle-diff: {adiff}\n")
		return adiff