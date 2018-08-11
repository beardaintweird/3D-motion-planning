import argparse
import time
import msgpack
import csv
from enum import Enum, auto
from sklearn.neighbors import KDTree

import numpy as np

from planning_utils import a_star, a_star_graph, heuristic, create_grid, extract_polygons, create_samples, collides, can_connect, create_graph
from visualize import visualize_points
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
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
        self.SAFETY_DISTANCE = 12
        self.TARGET_ALTITUDE = max(20,-int(self.local_position[2])+self.SAFETY_DISTANCE*1.5)

        self.obstacles = []
        self.adjustments = [(self.SAFETY_DISTANCE * 2, 0, 0), (-(self.SAFETY_DISTANCE * 2), 0, 0), (0, self.SAFETY_DISTANCE * 2, 0), (0, -(self.SAFETY_DISTANCE * 2), 0), (self.SAFETY_DISTANCE * 2, -(self.SAFETY_DISTANCE * 2), 0), (-(self.SAFETY_DISTANCE * 2), -(self.SAFETY_DISTANCE * 2), 0), (self.SAFETY_DISTANCE * 2, self.SAFETY_DISTANCE * 2, 0),(-(self.SAFETY_DISTANCE * 2), self.SAFETY_DISTANCE * 2, 0)]

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
            if np.linalg.norm(self.local_velocity[2]) < 0.2:
                self.disarming_transition()

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
        print(self.target_position)
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


        self.target_position[2] = self.TARGET_ALTITUDE

        lat0 = ''
        lon0 = ''
        with open('colliders.csv', newline='') as c:
            reader = csv.reader(c)
            lat0, lon0 = next(reader)
            split = lat0.split()
            split1 = lon0.split()
            lat0, lon0 = float(split[1]), float(split1[1])
        self.set_home_position(lon0, lat0, 0)
        print('latitude {0}, longitude {1}'.format(self._latitude, self._longitude))
        local_position = global_to_local(self.global_position, self.global_home)
        # print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         # self.local_position))
        print(local_position)
        print(local_position)
        print(local_position)
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, self.TARGET_ALTITUDE, self.SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        start = (int(local_position[0]), int(local_position[1]), self.TARGET_ALTITUDE)
        # goal = (750,370) # -- random goal
        # goal = (-310.2389,-439.2315,172) # -- random goal (need to append to to_keep)

        #create graph representation
        self.obstacles = extract_polygons(data, self.SAFETY_DISTANCE)
        samples = create_samples(data, 400)
        X = []
        for p in self.obstacles:
            X.append(p[0].bounds)
        poly_tree = KDTree(X)
        to_keep = []
        for point in samples:
            if not collides(self.obstacles, point, poly_tree):
                to_keep.append(point)

        start_adjusted = start
        i = 0
        while collides(self.obstacles, start_adjusted, poly_tree) and i < len(self.adjustments):
            print('collision with start point!', i, len(self.adjustments) - 1)
            start_adjusted = start
            start_adjusted = (start_adjusted[0] + self.adjustments[i][0],start_adjusted[1] + self.adjustments[i][1], start_adjusted[2] + self.adjustments[i][2])
            i += 1
        to_keep.append(start_adjusted)

        print('before collision check', len(samples), 'after collision check', len(to_keep))
        print('number of obstacles', len(self.obstacles))
        idx = np.random.randint(0,len(to_keep)-1)
        goal = to_keep[idx]

        g = create_graph(to_keep, 8, poly_tree, self.obstacles)



        # find the nodes closest to the start and goal positions
        node_tree = KDTree(to_keep)
        coords = [[start[0],start[1],0],[goal[0],goal[1],0]]
        ind = node_tree.query(coords, k=1, return_distance=False)
        start_approx, goal_approx = to_keep[ind[0][0]], to_keep[ind[1][0]]

        # TODO: adapt to set goal as latitude / longitude position and convert

        # Run A* to find a path from start to goal
        print('Local Start and Goal: ', start, goal)
        print('Approximate Local Start and Goal: ', start_approx, goal_approx)

        # grid_start = (start[0], start[1])
        # grid_goal = (-north_offset + 10, -east_offset + 10)
        # path, _ = a_star(grid, heuristic, grid_start, grid_goal)

        path, path_cost = a_star_graph(g, heuristic, start_approx, goal_approx)
        print(path)
        if len(path) == 0:
            self.plan_path()
        else:
            # Convert path to waypoints
            waypoints = [[int(p[0]), int(p[1]), int(p[2]), 0] for p in path]
            print(waypoints)
            # Set self.waypoints
            self.waypoints = waypoints
            # send waypoints to sim (this is just for visualization of waypoints)
            self.send_waypoints()
        # visualize_points(grid,data,path,g,start_approx)



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
