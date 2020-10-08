import tkinter as tk

import math

from linalgebra import *

class Simulator(object):
	def __init__(self, root, obstacles, rrt):
		self.canvas = None
		self.root = root

		self.canvas_width = 400
		self.canvas_height = 400

		self.obstacles = obstacles
		self.rrt = rrt

		self.obstacle_pointers = {}
		self.centroid_pointers = {}
		self.rrt_node_pointers = {}
		self.rrt_connection_pointers = {}
		self.rrt_label_pointers = {}
		self.timestamp_pointer = None

		self.finish_time = -1
		self.visited = []
		self.visited_nodes = []
		self.to_end = None

		self.start_draw()

	def start_draw(self):
		self.root.geometry(str(self.canvas_width + 100) + "x" + str(self.canvas_height))

		self.canvas = tk.Canvas(self.root, width=self.canvas_width, height=self.canvas_height)
		self.canvas.pack(side='left')

		stop = tk.Button(self.root, text='Exit')
		stop.pack(side='bottom')
		stop.bind('<Button-1>', self.stop_prog)

		self.canvas.bind("<Button-1>", self.set_goal)

	def set_goal(self, event):
		self.rrt.goal = Vector((event.x, event.y))
		self.start_prog()

	def start_prog(self, event=None):
		visited = self.rrt.create_rrt()

		self.rrt.max_time = self.rrt.forward

		self.draw_goal()

		time = tk.Scale(from_=0, to=self.rrt.max_time, length=(self.canvas_height - 25), resolution=.1, 
			activebackground='orchid3', troughcolor='orchid1', state=tk.DISABLED,
			command=self.update)
		time.pack(side='right')
		self.time = time
		self.time.config(state="normal")

	def stop_prog(self, event=None):
		self.root.quit()

	def display_sim(self, t, event=None):
		self.draw_rrt(t)
		self.draw_obstacles(t)
		self.draw_base()
		self.draw_timestamp(t)

		# self.root.after(int(self.rrt.time_step * 1000), self.display_sim, t + self.rrt.time_step)

	# updates the rrt, generating more nodes
	def update(self, event=None):
		curr_t = self.time.get()
		visited = self.rrt.update(curr_t)
		self.rrt.branch_weight = 5 + curr_t
		# found a goal node
		if visited and self.finish_time is -1:
			max_time = (visited[0].end.t + visited[0].end.len + .1)

			for item in visited:
				self.visited_nodes.append(item.end)
				print ("visited", item.end)

			print ("time: ", max_time)

			self.finish_time = max_time
			self.visited = visited
			self.time.configure(to=max_time, activebackground='green3', troughcolor='OliveDrab2')
		# haven't found the goal yet, keep generating more time
		elif self.finish_time is -1:
			self.time.configure(to=curr_t + self.rrt.forward)

	# displays a timestamp in the upper left corner
	def draw_timestamp(self, t):
		if not self.timestamp_pointer:
			self.timestamp_pointer = self.canvas.create_text(30, 10, text="t = " + str(t))
		else:
			self.canvas.itemconfig(self.timestamp_pointer, text="t = " + str(t))

	# draws a dot for the robot
	def draw_base(self):
		self.canvas.create_oval(self.draw_dot((
			self.rrt.base[0], 
			self.rrt.base[1]),
			self.rrt.size), 
			fill='green')

	# draws a dot for the goal node
	def draw_goal(self):
		self.canvas.create_oval(self.draw_dot((
			self.rrt.goal[0], 
			self.rrt.goal[1]),
			self.rrt.size), 
			fill='dodger blue')
		self.canvas.create_text(
			self.rrt.goal[0],
			self.rrt.goal[1] - 14,
			fill='black',
			text='Goal')

	# draws the rrt by looping over each node and each node's connections to other nodes
	def draw_rrt(self, t):
		for node, connections in self.rrt.data.items():

			color = 'PaleGreen1' if node.valid else 'salmon'

			# draw the connections too
			self.draw_connections(t, connections)

			if node and (node.t + node.len) <= t:
				if not node in self.rrt_node_pointers: # first time
					self.rrt_node_pointers[node] = self.canvas.create_oval(self.draw_dot((
						node.loc[0],
						node.loc[1]),
						node.size),
						fill=color)
					self.rrt_label_pointers[node] = self.canvas.create_text(
						node.loc[0], 
						node.loc[1] - 14, 
						fill='black',
						text=str(math.ceil((node.t + node.len)*10)/10)) # round to one decimal place
				else: # all later instances
					if node in self.visited_nodes:
						color = 'RoyalBlue1' if self.at_finish_time(t) else color
					self.canvas.itemconfig(self.rrt_node_pointers[node], fill=color, outline=color)
					self.canvas.itemconfig(self.rrt_label_pointers[node], fill='black')
			elif self.rrt_node_pointers.get(node):
				self.canvas.itemconfig(self.rrt_node_pointers[node], fill='white', outline='white')
				self.canvas.itemconfig(self.rrt_label_pointers[node], fill='white')
				self.canvas.tag_lower(self.rrt_label_pointers[node])
				self.canvas.tag_lower(self.rrt_node_pointers[node])

	# returns True if t is equal to self.finish_time 
	# (ie: the moment when the path to the goal exists)
	def at_finish_time(self, t):
		return abs(t - self.finish_time) < .1

	def draw_connections(self, t, connections):
		for connection in connections:
				
			color = ('PaleGreen1' if connection.valid else 'salmon')

			# description of the connection
			node = connection.start
			connect_name = self.rrt.connect_name(connection.start, connection.end)
			connect_pointer = self.rrt_connection_pointers.get(connect_name)

			if connection and node and (connection.t + connection.len) <= t:
				if not connect_pointer:
					# get the actual node, not the name of it
					other_node = connection.end

					connect_pointer = self.canvas.create_line(
						node.loc[0],
						node.loc[1],
						other_node.loc[0],
						other_node.loc[1],
						fill=color,
						width=4)
					self.rrt_connection_pointers[connect_name] = connect_pointer
					self.canvas.tag_lower(connect_pointer)
				else:
					if connection in self.visited:
						color = 'RoyalBlue1' if self.at_finish_time(t) else color
					self.canvas.itemconfig(connect_pointer, fill=color)

			elif self.rrt_connection_pointers.get(connect_name):
				self.canvas.itemconfig(connect_pointer, fill='white')
				self.canvas.tag_lower(connect_pointer)

		if self.at_finish_time(t):
			if self.to_end:
				self.canvas.itemconfig(self.to_end, fill='RoyalBlue1')
				self.canvas.tag_raise(self.to_end)
			else:
				self.to_end = self.canvas.create_line(
					self.visited_nodes[0].loc[0],
					self.visited_nodes[0].loc[1],
					self.rrt.goal[0],
					self.rrt.goal[1],
					fill='RoyalBlue1',
					width=4)
				self.canvas.tag_raise(self.to_end)
		elif self.to_end:
			self.canvas.itemconfig(self.to_end, fill='white')
			self.canvas.tag_lower(self.to_end)

	# returns the connection which goes end -> start
	def invert(self, connection):
		name = self.rrt.connect_name(connection.end, connection.start)
		return self.rrt.connects[name]

	# loops over the obstacles and draws them in turn at time = t
	def draw_obstacles(self, t):
		for obstacle in self.obstacles:
			a = obstacle.velocity[2]
			absolute_obs = obstacle.absolute_pos(t)
			absolute_points = []

			for abs_point in absolute_obs.points:
				# we need to loop over this since we have vectors, not a list of points
				absolute_points.append(abs_point[0])
				absolute_points.append(abs_point[1])

			if not obstacle.t0 in self.obstacle_pointers:
				obstacle_pointer = self.canvas.create_polygon(absolute_points, fill='light blue')
				# use t0 as a key, since we can assume no two shapes start atop each other
				self.obstacle_pointers[obstacle.t0] = obstacle_pointer
				self.canvas.tag_raise(obstacle_pointer)
			else:
				# modify the existing obstacle
				obstacle_pointer = self.obstacle_pointers.get(obstacle.t0)
				self.canvas.coords(obstacle_pointer, tuple(absolute_points))

			if not obstacle.t0 in self.centroid_pointers:
				# draw a dot at the centroid of the shape
				# need to add to the t0 point to get the absolute location
				absolute_centroid = self.draw_dot(obstacle.centroid(t).add(obstacle.t0), 3)
				self.centroid_pointers[obstacle.t0] = self.canvas.create_oval(absolute_centroid, 
					fill="steel blue", outline="")
			else:
				centroid_pointer = self.centroid_pointers.get(obstacle.t0)
				absolute_centroid = self.draw_dot(obstacle.centroid(t).add(obstacle.t0), 3)
				self.canvas.coords(centroid_pointer, absolute_centroid)

	# there is no built-in method for drawing a dot, so this implements one
	# returns the coordinates for a dot
	def draw_dot(self, coords, size):
		return (coords[0] - size, coords[1] - size, coords[0] + size, coords[1] + size)