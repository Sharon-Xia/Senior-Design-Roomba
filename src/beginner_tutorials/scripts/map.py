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
		PIXEL_ADJUST = 25

		def __init__(self):
			self.minX = 0
			self.maxX = 0
			self.minY = 0
			self.maxY = 0

		def instantiateLocationOffset(self, loc):
			if self.minX == 0:
				self.minX = loc.x()
			else:
				self.minX = min(loc.x(), self.minX)

			if self.maxX == 0:
				self.maxX = loc.x()
			else:
				self.maxX = max(loc.x(), self.maxX)

			if self.minY == 0:
				self.minY = loc.y()
			else:
				self.minY = min(loc.y(), self.minY)

			if self.maxY == 0:
				self.maxY = loc.y()
			else:
				self.maxY = max(loc.y(), self.maxY)


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
			y = (self.maxY - loc.y()) * self.PIXEL_ADJUST
			return Location(int(x), int(y), loc.timestamp())

		def adjustWidth(self):
			return int(self.width() * self.PIXEL_ADJUST) + 5

		def adjustHeight(self):
			return int(self.height() * self.PIXEL_ADJUST) + 5


	def __init__(self, name):
		self.nameOfRun = name

		self.runStartTime = datetime.now()
		self.map = []

		# replaced in ros simulator
		self.pathTraveled = [Location(0, 0, self.runStartTime)] # stores path coordinates
		# design system to account for repeat values later

		self.realPathTraveled = []

		self.outline = self.Outline()


	def instantiateStartingLocation(self, loc):
		self.pathTraveled = [loc]
		self.outline.instantiateLocationOffset(loc)


	def getLastLocation(self):
		return self.pathTraveled[-1]

	def addPathTraveled(self, loc):
		self.pathTraveled.append(loc)
		self.outline.updateOutline(loc)

	def addRealPathTraveled(self, loc, deltaDist):
		self.realPathTraveled.append((loc, deltaDist))
		self.outline.updateOutline(loc)

	def addMapCoordinate(self, loc):
		self.map.append(loc)
		self.outline.updateOutline(loc)

	def generatePNG(self):
		print("generating image")
		
		# null image
		if self.outline.width() == 0 or self.outline.height() == 0:
			return

		imageDimensions = (self.outline.adjustWidth(), self.outline.adjustHeight())
		print("image-dimensions: " + str(imageDimensions))
		img = Image.new('RGB', imageDimensions, "white")


		def drawPixel(loc, rgb):
			adjustedLoc = self.outline.adjustLoc(loc)

			try:
				img.putpixel((adjustedLoc.x(), adjustedLoc.y()), rgb)
			except IndexError as e:
				print(adjustedLoc)
				print(e)

		for loc in self.map:
			drawPixel(loc, (0, 0, 0))


		for loc in self.pathTraveled:
			drawPixel(loc, (255, 0, 0))

		for path in self.realPathTraveled:
			shadeOfGreen = int((path[1]/1.5) * 255)
			drawPixel(path[0], (0, shadeOfGreen, 0))


		img.save(self.nameOfRun + ".jpg")
		img.show()

		return img

