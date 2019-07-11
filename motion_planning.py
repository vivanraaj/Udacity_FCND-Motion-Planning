import argparse
import time
import msgpack
from enum import Enum, auto
import numpy.linalg as LA
import re

import numpy as np

from queue import PriorityQueue
from planning_utils import *

from skimage.morphology import medial_axis
from skimage.util import invert

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

import sys

class States(Enum):
	# auto below: Instances are replaced with an appropriate value for Enum members.
	MANUAL = auto()
	ARMING = auto()
	TAKEOFF = auto()
	WAYPOINT = auto()
	LANDING = auto()
	DISARMING = auto()
	PLANNING = auto()


class MotionPlanning(Drone):

	def __init__(self, connection):
		super().__init__(connection)

		self.target_position = np.array([0.0, 0.0, 0.0])
		self.waypoints = []
		self.in_mission = True
		self.check_state = {}

		# initial state
		self.flight_state = States.MANUAL

		# register all your callbacks here
		self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
		self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
		self.register_callback(MsgID.STATE, self.state_callback)

	def local_position_callback(self):
		if self.flight_state == States.TAKEOFF:
			if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
				self.waypoint_transition()
		elif self.flight_state == States.WAYPOINT:
			if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
				if len(self.waypoints) > 0:
					self.waypoint_transition()
				else:
					if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
						self.landing_transition()

	def velocity_callback(self):
		if self.flight_state == States.LANDING:
			if abs(self.local_position[2]) < (self.goal_altitude - 5 + .01):
				self.disarming_transition()   

			#if self.global_position[2] - self.global_home[2] < 0.1:
			#    if abs(self.local_position[2]) < 0.01:
			#        self.disarming_transition()

	def state_callback(self):
		if self.in_mission:
			if self.flight_state == States.MANUAL:
				self.arming_transition()
			elif self.flight_state == States.ARMING:
				if self.armed:
					self.plan_path()
			elif self.flight_state == States.PLANNING:
				self.takeoff_transition()
			elif self.flight_state == States.DISARMING:
				if ~self.armed & ~self.guided:
					self.manual_transition()

	def arming_transition(self):
		self.flight_state = States.ARMING
		print("arming transition")
		self.arm()
		self.take_control()

	def takeoff_transition(self):
		self.flight_state = States.TAKEOFF
		print("takeoff transition")
		self.takeoff(self.target_position[2])

	def waypoint_transition(self):
		self.flight_state = States.WAYPOINT
		print("waypoint transition")
		self.target_position = self.waypoints.pop(0)
		print('target position', self.target_position)
		self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

	def landing_transition(self):
		self.flight_state = States.LANDING
		print("landing transition")
		self.land()

	def disarming_transition(self):
		self.flight_state = States.DISARMING
		print("disarm transition")
		self.disarm()
		self.release_control()

	def manual_transition(self):
		self.flight_state = States.MANUAL
		print("manual transition")
		self.stop()
		self.in_mission = False

	def send_waypoints(self):
		print("Sending waypoints to simulator ...")
		data = msgpack.dumps(self.waypoints)
		self.connection._master.write(data)

	def plan_path(self):
		self.flight_state = States.PLANNING
		print("Searching for a path ...")
		#TARGET_ALTITUDE = 5
		#SAFETY_DISTANCE = 5

		#self.target_position[2] = TARGET_ALTITUDE

		# TODO: read lat0, lon0 from colliders into floating point values
		
		file_colliders = 'colliders.csv'

		with open(file_colliders) as File:
			lat_lon = File.readline()
		split = re.split(', ', lat_lon)
		lat0 = re.search('lat0 (.*)', split[0]).group(1)
		lon0 = re.search('lon0 (.*)', split[1]).group(1)
		print('Home Lattidute: {0}, Home Longitude: {1}'.format(lat0, lon0))
		
		# TODO: set home position to (lon0, lat0, 0)
		self.set_home_position = (lon0, lat0, 0)

		# TODO: retrieve current global position
		# Read in obstacle map
		data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

		
		
		start_loc = [self._longitude, self._latitude, self._altitude]
		start = global_to_local(start_loc, self.global_home)
		print("Local start position")
		print(start)

		# TODO: adapt to set goal as latitude / longitude position and convert
		goal_gps = input('Enter goal GPS coords like (long,lat) without the parenthesis :')
		g_gps = goal_gps.split(',')
		goal_gps = np.array([float(g_gps[0]), float(g_gps[1]), 0.0])


		#Goal location converted to local coordinate frame
		goal_loc = global_to_local(goal_gps, self.global_home)
		print("Local goal location")
		print(goal_loc)
		
		# minimum and maximum of obstacle coordinates
		# for usage in obstacle
		north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
		east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
		north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))
		east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))


		if (goal_loc[0] < north_min or goal_loc[0] > north_max) or (goal_loc[1] < east_min or goal_loc[1] > east_max):
			self.disarming_transition()
			sys.exit("The goal is off the grid")
		
		if LA.norm(goal_loc[0:2] - start[0:2]) < 5:
			self.disarming_transition()
			sys.exit("The goal is the same position as the start")

		# TODO: convert to current local position using global_to_local()
		# Verify if the goal location is situated on top of the obstacle top
		polygons = poly(data)
		collision_probable = crossover(polygons, goal_loc)
		if collision_probable[0] == True:
			print("The drone will land on the top")
			TARGET_ALTITUDE = collision_probable[1] + 5
			self.goal_altitude = collision_probable[1] + 5
		else:
			print("Landing on the ground!")
			TARGET_ALTITUDE = 5
			self.goal_altitude = 5
		
		if TARGET_ALTITUDE < (-self.local_position[2] + 5):
			TARGET_ALTITUDE = int(-self.local_position[2] + 5)
			
		SAFETY_DISTANCE = 5
		self.target_position[2] = TARGET_ALTITUDE
		print('Target altitude set to {0} meters'.format(TARGET_ALTITUDE))

		#Create a grid with obstacle data
		grid = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
		frame = medial_axis(invert(grid))

		#Find a start.goal location on the frame
		frame_start, frame_goal = start_pos(frame, [start[0] - north_min, start[1] - east_min], [goal_loc[0] - north_min, goal_loc[1] - east_min])

		#Find a path on the frame
		# implement a star for path searching
		path, cost, found = a_star(invert(frame).astype(np.int), heuristic, tuple(frame_start), tuple(frame_goal))
		if found == False:
			self.disarming_transition()
			sys.exit("Unable to find a path!")
		# if path is possible
		path.append((int(goal_loc[0]) - int(north_min), int(goal_loc[1]) - int(east_min)))

		# TODO: prune path to minimize number of waypoints
		if len(path) > 0:
			print("Pruning Waypoints")
			#Use collinearity, bresenham to prune path and get headings
			path = coll(path)
			path = bres(path, grid)
			path = heading(path)
			
			# Convert path to waypoints
			waypoints = [[p[0] + int(north_min), p[1] + int(east_min), TARGET_ALTITUDE, p[2]] for p in path]
			print("Waypoints: ", waypoints)
		self.waypoints = waypoints
		print("set wayponits")

		# TODO: send waypoints to sim (this is just for visualization of waypoints)
		# self.send_waypoints()

	def start(self):
		self.start_log("Logs", "NavLog.txt")

		print("starting connection")
		self.connection.start()

		# Only required if they do threaded
		# while self.in_mission:
		#    pass

		self.stop_log()

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument('--port', type=int, default=5760, help='Port number')
	parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
	args = parser.parse_args()

	conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
	drone = MotionPlanning(conn)
	time.sleep(1)

	drone.start()
