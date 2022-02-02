#!/usr/bin/env python
import rospy

# https://www.codegrepper.com/code-examples/python/draw+pixel+by+pixel+python
from PIL import Image

from location import *
from datetime import datetime


# records map info
# creates .png file
class Map:

	class Outline:
		# keeps track of outer edges of map
		PIXEL_ADJUST = 100

		def __init__(self):
			self.minX = 0
			self.maxX = 0
			self.minY = 0
			self.maxY = 0

		def updateOutline(self, loc):
			if loc.x() < self.minX:
				self.minX = loc.x()
			elif loc.x() > self.maxX:
				self.maxX = loc.x()

			if loc.y() < self.minY:
				self.minY = loc.y()
			elif loc.y() > self.maxY:
				self.maxY = loc.y()

		def width(self):
			return self.maxX - self.minX

		def height(self):
			return self.maxY - self.minY

		# after recentering map, adjust locations
		# x > 0, y > 0
		def adjustLoc(self, loc):
			x = (loc.x() - self.minX) * self.PIXEL_ADJUST
			y = (loc.y() - self.minY) * self.PIXEL_ADJUST
			return Location(int(x), int(y), loc.timestamp())

		def adjustWidth(self):
			return int(self.width() * self.PIXEL_ADJUST) + 5

		def adjustHeight(self):
			return int(self.height() * self.PIXEL_ADJUST) + 5


	def __init__(self, name):
		self.nameOfRun = name

		self.runStartTime = datetime.now()
		self.map = []
		self.pathTraveled = [Location(0, 0, self.runStartTime)] # stores path coordinates
		# design system to account for repeat values later

		self.outline = self.Outline()

	def getLastLocation(self):
		return self.pathTraveled[-1]

	def addPathTraveled(self, path):
		self.pathTraveled = path

	def addMapCoordinate(self, loc):
		self.map.append(loc)
		self.outline.updateOutline(loc)

	def generatePNG(self):
		
		# null image
		if self.outline.width() == 0 or self.outline.height() == 0:
			return

		img = Image.new('RGB', (self.outline.adjustWidth(), self.outline.adjustHeight()))

		for loc in self.map:
			adjustedLoc = self.outline.adjustLoc(loc)
			img.putpixel((adjustedLoc.x(), adjustedLoc.y()), (0, 0, 0))

		img.save(self.nameOfRun + ".png")
		img.show()

		return img

