import tkinter as tk

import random
import numpy as np
import sys

from linalgebra import *
from simulator import *

class RRT(object):

	# pixels traveled per second by simbot
	traversal_rate = 50

	# length of randomly created branches
	branch_len_min = 20.0
	branch_len_max = 180.0

	# the smaller, the fewer branches are created
	branch_weight = 10

	# update in seconds
	time_step = .5

	# the maximum time the simulator runs to
	max_time = 1

	# the distance between a new node and the goal to decide we've met the goal
	success_radius = 50.0

	# how many seconds to display as a forwards choice on the slider
	forward = 5

	def __init__(self, root):
		self.size = 7
		self.speed = 20
		self.base = Vector((200, 180))
		self.first_node = None
		self.goal = Vector((300, 350))

		self.sim = None
		self.root = root

		self.rrt_index = 0

		self.data = {}
		self.connects = {}
		self.name_to_node = {} # converts a name to a node

		self.found_goal = False

		self.top_time = 0

		# probability of creating a new branch off of an existing one, checked each loop cycle
		self.branch_creation = self.update_branch_creation()

	def __str__(self):
		str_list = []
		str_list.append("RRT: [")

		for key, values in self.data.items():
			str_list.append(str(key) + " ")
			for value in values:
				str_list.append(str(value) + " ")

		str_list.append("]")
		return ''.join(str_list)

	def create_rrt(self):
		# need a second node to be able to run validity
		first_node = self.add_node(self.base, [], 0)
		self.first_node = first_node
		self.add_branch(0, 0)

	def update(self, t):
		base = self.name_to_node.get(0)

		visited = None

		if self.sim and base:
			if t > self.top_time:
				self.top_time = t
				visited = self.add_branches(t)
				if visited:
					return visited

			self.sim.display_sim(t)

		return visited

	# creates a node which is at the given x, y; connected to connections; and has a name
	def add_node(self, loc, connection_nodes, t):
		new_node = Node(self.rrt_index, loc, t)

		# loop over names of connections and create a new one

		connections = []
		for connection_node in connection_nodes:
			other_connect = self.connect_name(connection_node, new_node)
			if not other_connect in self.connects:
				connections.append(self.add_connect(new_node, connection_node, t))
		self.data[new_node] = connections

		self.name_to_node[self.rrt_index] = new_node

		self.rrt_index += 1

		return new_node

	def add_connect(self, start, end, t):
		new_connect = Connection(start, end, t)
		
		new_connect_name = self.connect_name(start, end)
		self.connects[new_connect_name] = new_connect
		return new_connect

	# loops over all connections to a node and marks how long they are
	def create_lengths(self, node, length_before, visited):
		visited.append(node)
		for connection in self.data[node]:
			next_node = connection.end
			if not next_node in visited:
				length = (node.loc.subtract(next_node.loc).len() / RRT.traversal_rate
					if (node and next_node) else 0) # distance between node and next_node
				length_before += length
				if connection.len is 0:
					connection.len = length_before
				if next_node.len is 0:
					next_node.len = length_before
				self.create_lengths(next_node, length_before, visited)

	def node_name(self, node):
		return str(node.name) if node else "None"

	def connect_name(self, start, end):
		return self.node_name(start) + ":" + self.node_name(end)

	# creates a series of random branches off of each existing node
	def add_branches(self, t):
		visited = None
		data = self.data.copy()
		for key in data.keys():
			add_branch = random.random() <= self.update_branch_creation()
			if add_branch:
				new_visited = self.add_branch(key.name, t)
				if new_visited:
					visited = new_visited
		return visited

	# creates a branch in a random direction with given name off of given trunk
	def add_branch(self, trunk_name, t):
		trunk = self.name_to_node[trunk_name]

		# find angle between the trunk node and the goal node
		del_x = self.goal[0] - trunk[0]
		del_y = self.goal[1] - trunk[1]
		goal_a = math.atan(del_y/del_x)
		# random number between [0, 2 pi), measured counterlockwise from the horizontal
		# rand_a = ((random.random() - random.random() # random number in [-1, 1], weighted towards 0
		# 		+ 1)*math.pi # random number in [0, 2pi], weighted towards pi
		# 		+ (goal_a - math.pi) # weight the random numbers towards goal.a
		# 		) % 2*math.pi # fit the number into [0, 2pi]
		# rand_a = random.random() * 2.0*math.pi
		# rand_a = ((np.random.randn() # assume this is usually [-2pi, 2pi], weighted to 0
		# 		+ 2*math.pi)/2 # [0, 2pi], weighted to 2pi
		# 		+ (goal_a - math.pi) # weighted to goal_a
		# 		) % 2*math.pi
		rand_a = np.random.normal(goal_a, .2) % 2.0*math.pi

		del_h = self.dist_to_goal(trunk)

		while True:
			rand_dist = random.random() * self.branch_len_max + self.branch_len_min
			# rand_dist = random.random() * (del_h + 30) + (del_h - 30)

			rand_x = math.cos(rand_a) * rand_dist
			rand_y = math.sin(rand_a) * rand_dist
			rand_loc = Vector((rand_x, rand_y)).add(trunk.loc)
			if rand_loc[0] > 0 and rand_loc[0] < 400 and rand_loc[1] > 0 and rand_loc[1] < 400:
				# within the canvas
				break

			rand_a += math.pi/10

		new_branch = self.add_node(rand_loc, [], t)

		dist_to_goal = self.dist_to_goal(new_branch)

		new_connect = self.add_connect(trunk, new_branch, t)
		self.data[trunk].append(new_connect)
		self.create_lengths(self.first_node, 0, [])
		self.validity(self.add_connect(None, self.first_node, 0), t)

		if dist_to_goal <= self.success_radius:
			visited = self.find_goal_path(new_branch, [])
			# if visited:
			# 	print "testing at", t
			# 	for item in visited:
			# 		print "visited: ", item

			return visited

	# finds a path from to_find to the goal
	def find_goal_path(self, to_find, visited):

		if to_find is self.first_node:
			# visited.append(self.first_node)
			return visited

		for node, connections in self.data.items():
			# find a node which connects to to_find
			for connection in connections:
				if to_find is connection.end:
					if not connection.valid: # the connection to the node isn't valid
						return None
					visited.append(connection)
					# then look for a node which connects to that one
					return self.find_goal_path(node, visited)

	# the distance between the goal and the node
	def dist_to_goal(self, node):
		return node.loc.subtract(self.goal).len()

	# check validity of node paths, moves downards through connections to in_connect.end
	def validity(self, in_connect, t):
		for connection in self.data[in_connect.end]:
			connection.valid = True
			connection.end.valid = True
			if self.intersects_obs(connection, t):
				connection.valid = False
			# if this node isn't valid, nothing it connects to is
			# ignore the way you came from
			if (((not connection.valid) or (not in_connect.valid)) and 
				in_connect.end is not connection.end):
				connection.valid = False
				connection.end.valid = False
			if in_connect.end is not connection.end:
				self.validity(connection, t)

	# Returns true if the connection intersects any obstacle
	def intersects_obs(self, connection, t):
		for obstacle in self.sim.obstacles:
			if self.intersects_ob(connection, obstacle, t):
				return True
		return False

	# Checks each side of the obstacle and see if it intersects the connection
	# see https://www.cdn.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
	def intersects_ob(self, connection, obstacle, t):
		# allows the last vertex to loop back to the first one
		vertices = list(obstacle.absolute_pos(t).points)
		vertices.append(vertices[0])

		for i in range(0, len(vertices) - 1):
			side = Connection(vertices[i], vertices[i+1], 0)
			side_1 = side.get_rotate(connection[0])
			side_2 = side.get_rotate(connection[1])

			connect_1 = connection.get_rotate(side[0])
			connect_2 = connection.get_rotate(side[1])

			# if both sets go different directions, then the side intersects the connection
			if (side_1 is not side_2) and (connect_1 is not connect_2):
				return True

		return False

	# see https://www.desmos.com/calculator/zw6bjoeph2 for the probability outputted
	# see https://www.desmos.com/calculator/2iovtlu2fn for average nodes created each loop cycle
	# both are based on the number of existing nodes
	def update_branch_creation(self):
		self.branch_creation = 1 - math.tanh(len(self.data)/RRT.branch_weight)
		return self.branch_creation


# represents a single node in the rrt
class Node(object):
	def __init__(self, name, loc, t):
		self.name = name
		self.size = 7
		self.loc = loc
		self.t = t
		self.len = 0

		self.valid = True

	def __getitem__(self, index):
		return self.loc[index]

	def __str__(self):
		return ("[" + str(self.name) + ": " + str(self.valid) + " " + str(self.t + self.len) + " " + str(self.loc) +  "]")

# connects two nodes
class Connection(object):

	def __init__(self, start, end, t):
		self.start = start
		self.end = end
		self.t = t
		self.len = 0

		self.valid = True
		# self.valid = end.name is not 1

	def __str__(self):
		return ("Connect: [" + str(self.valid) + " (" + 
			str(self.start) + " to " + str(self.end) + ") ]")

	def __getitem__(self, index):
		if index is 0:
			return self.start
		elif index is 1:
			return self.end

	# returns the direction the connection rotates relative to the input vector
	# see https://www.geeksforgeeks.org/orientation-3-ordered-points/
	# returns True --> clockwise; False --> counterclockwise
	def get_rotate(self, vector):
		direction = ((self.end[1] - self.start[1]) * (vector[0] - self.end[0]) -
			(self.end[0] - self.start[0]) * (vector[1] - self.end[1]))
		return direction > 0

def main():

	null = Vector((0, 0, 0))

	# one pentagon, one not square to the axes, and one with a velocity
	ob1 = Shape((
		Vector((-40, -40)), 
		Vector((40, -40)), 
		Vector((60, 0)), 
		Vector((40, 40)), 
		Vector((-40, 40))
		),
		Vector((100, 100, 0)), Vector((30, -30, 0)))

	line = Connection(Vector((200, 180)), Vector((160.308, 314.109)), 0)
	# side = Connection(Vector((0, 0)), Vector((100, 0)), 0)

	ob2 = Shape((
		Vector((0, -40)), 
		Vector((40, 0)), 
		Vector((0, 40)), 
		Vector((-40, 0))
		), 
		Vector((200, 240, 0)), Vector((0, 10, -math.pi/3)))

	ob3 = Shape((
		Vector((-50, -20)), 
		Vector((50, -20)), 
		Vector((50, 20)), 
		Vector((-50, 20))
		), 
		Vector((300, 300, 0)), Vector((6, 0, math.pi/10)))
		
	ob4 = Shape((
		Vector((-30, -10)), 
		Vector((20, -20)), 
		Vector((10, 20)), 
		Vector((-50, 20))
		), 
		Vector((50, 300, 0)), Vector((30, -10, 0)))

	obstacles = (ob1, ob2, ob3, ob4)

	root = tk.Tk()
	rrt = RRT(root)
	sim = Simulator(root, obstacles, rrt)
	rrt.sim = sim

	# print rrt.intersects_obs(line)

	# print side.get_rotate(Vector((120, 20)))
	# sim.canvas.create_line(100, 100, 190, 110)

	root.mainloop()

if __name__ == "__main__":
	sys.exit(main())