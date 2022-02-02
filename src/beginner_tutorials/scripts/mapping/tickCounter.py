

# tool to restrict how often function calls go through
# tracks number of ticks until a function is called
class TickCounter:
	
	# tickers: list of tick types to count & the # of ticks for each type
	# 			[(string: tickName, int: # of ticks), ...]
	def __init__(self, tickers):
		
		self.tickLimits = {} # number of ticks it takes to set off each function
		self.tickCounts = {} # counts ticks

		for t in tickers:
			self.tickLimits[t[0]] = t[1]
			self.tickCounts[t[0]] = 0

	
	# name: string = name of tick
	# returns: true if function is called, false if not called
	def tick(self, name):
		if not self.tickCounts.keys().contains(name):
			raise IndexError(f"Reference Tick Error: no such tick [{name}]")
		
		self.tickCounts[name] += 1

		if self.tickCounts[name] == self.tickLimits[name]:
			self.tickCounts[name] = 0
			return True 

		return False

	
	# adds new ticker to track
	# ticker: (string: tickName, int: # of ticks)
	def addTicker(self, ticker):
		self.tickLimits[ticker[0]] = ticker[1]
		self.tickCounts[ticker[0]] = 0