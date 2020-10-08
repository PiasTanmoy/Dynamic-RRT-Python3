import math

# vector[0] = x; vector[1] = y; vector[2] = theta
class Vector(object):

	def __init__(self, coords):
		self.coords = coords

	def __str__(self):
		return str(self.coords)

	# gets the coordinate at the indicated index, called as vector[i]
	def __getitem__(self, index):
		return self.coords[index]

	# sets the coordinate at the indicated index, called as vector[i] = val
	def __setitem__(self, index, val):
		self.coords[index] = val
		return self

	# gives the number of coordinates in the vector, called as len(vector)
	def __len__(self):
		return len(self.coords)

	# gives the Euclidean length of vector, called as vector.len()
	def len(self):
		total = 0
		for coord in self.coords:
			total += coord**2.0

		return math.sqrt(total)

	# returns this vector multiplied by a scalar
	def scalar(self, scalar):
		new_vector = []

		for coord in self.coords:
			new_vector.append(coord * scalar)

		return Vector(tuple(new_vector))

	# adds this vector to other
	def add(self, other):
		new_vector = []

		for i in range(0, len(self)):
			new_vector.append(self[i] + other[i])

		return Vector(tuple(new_vector))

	# subtracts other from self
	def subtract(self, other):
		new_vector = []

		for i in range(0, len(self)):
			new_vector.append(self[i] - other[i])

		return Vector(tuple(new_vector))

class Shape(object):
	# Takes an tuple of vectors defining the corners of the shape relative to the center, 
	# clockwise from UL; where the center starts; and a velocity that the shape is moving at
	def __init__(self, points, t0, velocity):

		points3 = [] # points, but vectors of length 3

		# append a zero for rotation if needed
		for point in points:
			if len(point) is 2:
				points3.append(Vector((point[0], point[1], 0)))
			else:
				points3.append(point)

		self.points = tuple(points3)
		self.velocity = velocity
		self.t0 = t0 # position at t = 0

	def __str__(self):
		str_list = []
		for point in self.points:
			str_list.append(str(point))
		return ''.join(str_list)
	
	# returns the location of the shape at the given time t
	# as r(t) = integral(vdt, 0, t) = vt + initial position
	def location(self, t):
		return Vector((
			self.velocity[0] * t + self.t0[0], 
			self.velocity[1] * t + self.t0[1],
			self.velocity[2] * t + self.t0[2]
			))

	# returns the shape's points, relative to the canvas NOT the center at time = t
	# this is the very important function that returns coordinates for drawing the obstacle!
	def absolute_pos(self, t):
		abs_points = []

		rotated = self.rotate(self.velocity[2]*t + self.t0[2])

		curr_loc = rotated.location(t)

		for point in rotated.points:
			# adding combines the components of the center with those of each point
			abs_points.append(curr_loc.add(point))

		return Shape(abs_points, rotated.t0, rotated.velocity)

	# returns the shape, rotated by a (measured in radians)
	def rotate(self, a):
		rot_matrix = Matrix((
			Vector((math.cos(a), math.sin(a), 0)),
			Vector((-math.sin(a), math.cos(a), 0)),
			Vector((0, 0, 1))
			))

		new_points = []

		for point in self.points:
			new_points.append(rot_matrix.mult(point))

		# new_t0 = (self.t0[0], self.t0[1], self.t0[2] + a)
		return Shape(tuple(new_points), self.t0, self.velocity)

	# finds the centroid (center of mass) of the shape at time t
	# see: https://en.wikipedia.org/wiki/Centroid#Of_a_polygon
	def centroid(self, t):
		# appends the first point to the end of the list, required for this
		looped = list(self.points)
		looped.append(self.points[0])

		area = self.area()
		c_x = 0
		c_y = 0
		for i in range(0, len(looped) - 1):
			c_x += ((looped[i][0] + looped[i+1][0]) * 
				(looped[i][0]*looped[i+1][1] - looped[i+1][0]*looped[i][1]))
			c_y += ((looped[i][1] + looped[i+1][1]) * 
				(looped[i][0]*looped[i+1][1] - looped[i+1][0]*looped[i][1]))

		c_x *= 1.0/(6.0*area)
		c_y *= 1.0/(6.0*area)
		return Vector((c_x + self.velocity[0] * t, c_y + self.velocity[1] * t))

	# Finds the area of the shape
	# This differs from the signed area, as area below the x axis is still positive here
	# see: https://en.wikipedia.org/wiki/Shoelace_formula
	def area(self):
		area = 0
		j = len(self.points) - 1
		for i in range(0, len(self.points)):
			area += (self.points[j][0] + self.points[i][0]) * (self.points[j][1] - self.points[i][1])
			j = i

		return abs(area / 2)

class Matrix(object):

	# values should be a tuple of tuples (list of columns (which are vectors))
	def __init__(self, values):
		self.values = values

	def __str__(self):
		str_list = []
		for col in self.values:
			str_list.append(str(col))
		return ''.join(str_list)

	# multiplies the matrix by the vector
	def mult(self, vector):
		result = Vector((0,) * len(vector)) # tuple of the length of the vector

		for i in range(0, len(vector)):
			# loop over the incoming vector's coordinates and the matrix's vectors
			col = self.values[i]
			vector_val = vector[i]
			result_col = col.scalar(vector_val)
			result = result.add(result_col)

		return result