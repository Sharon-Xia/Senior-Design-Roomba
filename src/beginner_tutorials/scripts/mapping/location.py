import math
# stores location coordinates


class Location:

	def __init__(self, x, y, ts):
		self.x = x
		self.y = y
		self.ts = ts

	def x(self):
		return self.x

	def y(self):
		return self.y

	def timestamp(self):
		return self.ts

	def __str__(self):
		time_string = self.ts.strftime("%H:%M:%S")
		return f"x: {self.x}, y: {self.y}, time: {time_string}"


	# comparing with another loc
	# self.x > loc.x -> self.compareX(loc) > 0
	def compareX(self, loc):
		return self.x() - loc.x()

	def compareY(self, loc):
		return self.y() - loc.y()


	def distanceFrom(self, loc):
		sqr = (self.compareX(loc) ** 2) + (self.compareY(loc) ** 2)
		return math.sqrt(sqr)