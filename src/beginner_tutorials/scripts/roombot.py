#!/usr/bin/env python
import rospy
import math


from map import *
from location import *
from datetime import datetime

import matplotlib.pyplot as plt


# stores information on the vehicle
# including location, sensors..etc.
# also calculates map points with sensors if map present

# constants
pathThreshold = 0.0 # amount of noise to ignore for path traveled in ?? units

# idk why but the calculated angles are diff from the real 
# simulator -- coefficient adjusts sensitivity of deltaTheta
ANGLE_COEFFICIENT = 1.00

class Roombot:

	def __init__(self, nameOfRun):
		self.map = Map(nameOfRun) 

		self.distanceTraveled = 0.0
		self.currentAngle = math.pi/2 # instantiate with odometer

		# isInstantiated to be same as simulator
		self.angleInstantiated = False 
		self.locationInstantiated = False


		# keep track of delta(location)
		self.deltaDistance = 0.0
		self.deltaTheta = 0.0
		self.timeOfLastUpdate = datetime.now()


		# ros only - check accuracy
		self.realLocation = Location(0, 0, datetime.now()) # TODO

		# simulator starts at math.pi/2 (upwards orientation)
		# later maybe instantiate with simulator's true angle
		self.realAngle = math.pi/2

		self.thetaToDistanceRecord = []
		self.lastDeltaTheta = 0
		self.lastDeltaDistance = 0
		self.realDeltaTheta = 0
		self.realDeltaDistance = 0

	# records
	def updatePath(self):
		currentLoc = self.updatedLocation()

		# append if difference great enough
		if self.map.getLastLocation().distanceFrom(currentLoc) > pathThreshold:
			self.map.addPathTraveled(currentLoc)
			self.resetDelta()			

		return currentLoc # compare with actual location in ROS

	# calculates current location & updates trackers
	# returns current location
	def updatedLocation(self):
		prevLoc = self.map.getLastLocation()
		#transformedDeltaTheta = self.deltaTheta 

		# dtheta callibration -- after running tests
		# self.deltaTheta = min(self.deltaTheta, 0.15)
		# self.deltaTheta = max(self.deltaTheta, -0.15)

		transformedDeltaTheta = self.deltaTheta
		transformedDeltaTheta *= self.calculateAngleCoefficient()

		angle = self.currentAngle + transformedDeltaTheta

		rospy.loginfo("deltaDistance: %f   delta-x: %f  delta-y: %f theta-used: %f", self.deltaDistance, \
		 (math.cos(angle) * self.deltaDistance), \
		 (math.sin(angle) * self.deltaDistance) , \
		 self.deltaTheta)

		x = prevLoc.x() + (math.cos(angle) * self.deltaDistance)
		y = prevLoc.y() + (math.sin(angle) * self.deltaDistance)

		self.currentAngle += transformedDeltaTheta
		currentLoc = Location(x, y, self.timeOfLastUpdate)

		return currentLoc

	# as ddelta decreases, dtheta becomes less accurate
	def calculateAngleCoefficient(self):
		versatileRange = [0, 0.2, 0.5, 0.8, 0.11, 0.15]
		coefficients = [1, 1.4, 1.5, 1.3, 1.2]

		for i, c in enumerate(coefficients):
			if self.deltaDistance > versatileRange[i] and self.deltaDistance < versatileRange[i+1]:
				return c

		return ANGLE_COEFFICIENT

	def resetDelta(self):
		self.lastDeltaTheta = self.deltaTheta
		self.lastDeltaDistance = self.deltaDistance

		self.deltaTheta = 0.0
		self.deltaDistance = 0.0


	"""
	The following functions update vehicle distance and angle for
	its position to be calculated later on with the updatedLocation() function
	"""

	# amount of vehicle turn
	def updateAngle(self, theta):
		self.deltaTheta += theta

	# amount of vehicle travel
	def updateDistanceTraveled(self, distance):
		self.deltaDistance += distance

	# call when distance/angle updated
	def updateTs(self):
		self.timeOfLastUpdate = datetime.now()
		return self.timeOfLastUpdate

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
		actualDistanceTraveled = realLoc.distanceFrom(self.realLocation)
		rospy.loginfo("real delta distance: %f", actualDistanceTraveled)
		self.realDeltaDistance = actualDistanceTraveled

		self.realLocation = realLoc
		self.map.addRealPathTraveled(realLoc, actualDistanceTraveled)

		if not self.locationInstantiated:
			self.map.instantiateStartingLocation(realLoc)
			self.locationInstantiated = True

		# TODO
		currentLoc = self.map.getLastLocation()

		rospy.loginfo("currentLoc: " + str(currentLoc) + "   realLoc: " + str(realLoc))


	def updateRealAngle(self, realAngle):
		actualDeltaTheta = realAngle - self.realAngle
		rospy.loginfo("real delta theta: %f", actualDeltaTheta)
		self.realDeltaTheta = actualDeltaTheta

		self.realAngle = realAngle

		if not self.angleInstantiated: # for ROS Simulation
			self.currentAngle = realAngle
			self.angleInstantiated = True

		adiff = self.currentAngle - self.realAngle

		# TODO
		rospy.loginfo("currentAngle: %f   realAngle: %f", self.currentAngle, self.realAngle)
		#rospy.loginfo("angle-diff: " + str(adiff))
		return adiff


	def updateRecord(self):
		self.thetaToDistanceRecord.append((self.lastDeltaTheta, \
		self.lastDeltaDistance, \
		self.realDeltaTheta, \
		self.realDeltaDistance))


	def end(self):
		dtheta = [x[0] for x in self.thetaToDistanceRecord]
		ddist = [x[1] for x in self.thetaToDistanceRecord]
		rdtheta = [x[2] for x in self.thetaToDistanceRecord]
		rddist = [x[3] for x in self.thetaToDistanceRecord]

		distdiff = [x[1] - x[3] for x in self.thetaToDistanceRecord]
		thetadiff = [x[2] - x[0] for x in self.thetaToDistanceRecord]

		# plt.plot(ddist, dtheta, label="delta theta vs. dist")
		# plt.plot(rddist, rdtheta, label="real delta theta vs. real delta dist")

		# plt.plot(distdiff, rdtheta, 'ro', label="dist-diff vs. real delta theta")
		plt.plot(rddist, thetadiff, 'ro', label="real delta dist vs. thetadiff")
		plt.plot(ddist, thetadiff, 'bo', label="calc delta dist vs. thetadiff")

		# plt.plot(rdtheta, dtheta, 'ro', label="real dtheta vs. calc dtheta")

		plt.xlabel("rddist")
		plt.ylabel("thetadiff")

		plt.legend()
		#plt.show()

		self.map.generatePNG()