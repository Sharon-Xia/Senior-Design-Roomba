import math
# stores location coordinates


class Location:

	def __init__(self, x, y, ts):
		self.X = x
		self.Y = y
		self.Ts = ts

	def x(self):
		return self.X

	def y(self):
		return self.Y

	def timestamp(self):
		return self.Ts

	def __str__(self):
		time_string = self.Ts.strftime("%H:%M:%S")
		return "x: {self.X}, y: {self.Y}, time: {time_string}"


	# comparing with another loc
	# self.x > loc.x -> self.compareX(loc) > 0
	def compareX(self, loc):
		return self.x() - loc.x()

	def compareY(self, loc):
		return self.y() - loc.y()


	def distanceFrom(self, loc):
		sqr = (self.compareX(loc) ** 2) + (self.compareY(loc) ** 2)
		return math.sqrt(sqr)