import numpy as np
import re
import numpy.linalg as LA
from queue import PriorityQueue
from skimage.morphology import medial_axis
from skimage.util import invert
from enum import Enum
from shapely.geometry import Polygon, Point
from bresenham import bresenham
import sys

def collinearity(p1, p2, p3, epsilon): 
	collinear = False
	p1= np.append(p1, 1)
	p2= np.append(p2, 1)
	p3= np.append(p3, 1)
	mat= np.vstack((p1, p2, p3))
	det= np.linalg.det(mat)
	if det < epsilon:
		collinear = True

	return collinear

def create_grid(data, drone_altitude, safety_distance):
	"""
	Returns a grid representation of a 2D configuration space
	based on given obstacle data, drone altitude and safety distance
	arguments.
	"""

	# minimum and maximum north coordinates
	north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
	north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

	# minimum and maximum east coordinates
	east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
	east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

	# given the minimum and maximum coordinates we can
	# calculate the size of the grid.
	north_size = int(np.ceil(north_max - north_min))
	east_size = int(np.ceil(east_max - east_min))

	# Initialize an empty grid
	grid = np.zeros((north_size, east_size))

	# Populate the grid with obstacles
	for i in range(data.shape[0]):
		north, east, alt, d_north, d_east, d_alt = data[i, :]
		if alt + d_alt + safety_distance > drone_altitude:
			obstacle = [
				int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
				int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
				int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
				int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
			]
			grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

	return grid

def heuristic(position, goal_position):
	return np.linalg.norm(np.array(position) - np.array(goal_position))

'''
def lat_lon(file):
	#Read the lat0, lon0 from colliders data into floating point values
	with open(file) as File:
		lat_lon = File.readline()
	split = re.split(', ', lat_lon)
	lat = re.search('lat0 (.*)', split[0]).group(1)
	lon = re.search('lon0 (.*)', split[1]).group(1)
	return float(lat), float(lon)
'''

# Assume all actions cost the same.
class Action(Enum):
	"""
	An action is represented by a 3 element tuple.

	The first 2 values are the delta of the action relative
	to the current grid position. The third and final value
	is the cost of performing the action.
	"""

	WEST = (0, -1, 1)
	EAST = (0, 1, 1)
	NORTH = (-1, 0, 1)
	SOUTH = (1, 0, 1)
	NORTH_WEST = (-1, -1, np.sqrt(2))
	NORTH_EAST = (-1, 1, np.sqrt(2))
	SOUTH_WEST = (1, -1, np.sqrt(2))
	SOUTH_EAST = (1, 1, np.sqrt(2))

	@property
	def cost(self):
		return self.value[2]

	@property
	def delta(self):
		return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
	"""
	Returns a list of valid actions given a grid and current node.
	"""
	valid_actions = list(Action)
	n, m = grid.shape[0] - 1, grid.shape[1] - 1
	x, y = current_node

	# check if the node is off the grid or
	# it's an obstacle

	if x - 1 < 0 or grid[x - 1, y] == 1:
		valid_actions.remove(Action.NORTH)
	if x + 1 > n or grid[x + 1, y] == 1:
		valid_actions.remove(Action.SOUTH)
	if y - 1 < 0 or grid[x, y - 1] == 1:
		valid_actions.remove(Action.WEST)
	if y + 1 > m or grid[x, y + 1] == 1:
		valid_actions.remove(Action.EAST)

	if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
		valid_actions.remove(Action.NORTH_WEST)
	if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1] == 1:
		valid_actions.remove(Action.NORTH_EAST)
	if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
		valid_actions.remove(Action.SOUTH_WEST)
	if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:
		valid_actions.remove(Action.SOUTH_EAST)

	return valid_actions

def a_star(grid, h, start, goal):

	path = []
	path_cost = 0
	queue = PriorityQueue()
	queue.put((0, start))
	visited = set(start)

	branch = {}
	found = False
	
	while not queue.empty():
		item = queue.get()
		current_node = item[1]
		if current_node == start:
			current_cost = 0.0
		else:              
			current_cost = branch[current_node][0]
			
		if current_node == goal:        
			print('Found a path.')
			found = True
			break
		else:
			for action in valid_actions(grid, current_node):
				# get the tuple representation
				da = action.delta
				next_node = (current_node[0] + da[0], current_node[1] + da[1])
				branch_cost =  action.cost + current_cost
				queue_cost = branch_cost + h(next_node, goal)
				
				if next_node not in visited:                
					visited.add(next_node)               
					branch[next_node] = (branch_cost, current_node, action)
					queue.put((queue_cost, next_node))
			 
	if found:
		# retrace steps
		n = goal
		path_cost = branch[n][0]
		path.append(goal)
		while branch[n][1] != start:
			path.append(branch[n][1])
			n = branch[n][1]
		path.append(branch[n][1])

	return path[::-1], path_cost, found



def extract_polygons(data):

	polygons = []
	for i in range(data.shape[0]):
		north, east, alt, d_north, d_east, d_alt = data[i, :]
		
		corners = [(north - d_north, east - d_east), 
				   (north - d_north, east + d_east), 
				   (north + d_north, east + d_east), 
				   (north + d_north, east - d_east)]
		
		height = alt + d_alt

		p = Polygon(corners)
		polygons.append((p, height))

	return polygons

def collides(polygons, point):

	for (p,h) in polygons:
		if p.contains(Point(point)):
			return (True, h)
	return (False, h)

def find_start_goal(skel, start, goal):

	start= np.array(start)
	goal = np.array(goal)
	skel_point = np.array(np.transpose(skel.nonzero()))
	start_dist = (np.linalg.norm(skel_point - start, axis = 1)).argmin()
	near_start = skel_point[start_dist]
	goal_dist = (np.linalg.norm(skel_point - goal, axis = 1)).argmin()
	near_goal = skel_point[goal_dist]
	return near_start, near_goal

def bres(path, grid):
	i = 0
	while True:
		if i > (len(path)-3):
			break
		p1 = path[i]
		j = len(path) - 1
		while True:
			if j == (i+1):
				break
			p2 = path[j]
			cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
			in_collision = False
			for c in cells:
				if c[0] < 0 or c[1] <0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
					in_collision = True
					break
				if grid[c[0], c[1]] == 1:
					in_collision = True
					break
			if in_collision == False:
				for r in range(i+1,j):
					path.remove(path[i+1])
				break
			else:
				j = j-1
		i = i+1

	return path

def coll(path):
	i = 0
	while True:
		
		if collinearity(path[i], path[i+1], path[i+2], epsilon=1e-2) == True:
			path.remove(path[i+1])
			if i > len(path) - 3:
				break
		else:
			i = i + 1
			if i > len(path) - 3:
				break
	return path


def heading(path):
	path[0] = list(path[0])
	path[0].append(0)
	for i in range(0, len(path)-1):
		p1 = path[i]
		p2 = path[i+1]
		head = np.arctan2((p2[1] - p1[1]), (p2[0] - p1[0]))
		path[i+1] = list(path[i+1])
		path[i+1].append(head)

	return path


