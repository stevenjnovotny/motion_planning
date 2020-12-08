import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star_graph, heuristic, create_grid_and_edges, closest_graph_point, check_edge
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

import pkg_resources
pkg_resources.require("networkx==2.1")
import networkx as nx

from pruning import get_pruned_path
import matplotlib as mpl
mpl.use('tkagg')
import matplotlib.pyplot as plt


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
            #print(self.global_position, self.global_home)
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
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
        print('alt:', self.local_position, self.target_position)
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
        print(self.waypoints)
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 15
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # read lat0, lon0 from colliders into floating point values
        filename = 'colliders.csv'
        with open(filename) as fd:
            line = fd.readline().split(',')
        lat0 = float(line[0].split(' ')[-1])   
        lon0 = float(line[1].split(' ')[-1])
        print('start position: lat: {}. lon: {}'.format(lat0, lon0))

        # set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # retrieve current global position
        global_position = [self._longitude, self._latitude, self._altitude]
 
        # convert to current local position using global_to_local()
        local_position = global_to_local(global_position=self.global_position, global_home=self.global_home)
        print(local_position)
        print(self.local_position)

        
        print('global home {0}, \nposition {1}, \nlocal position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        t_start = time.time()
        grid, north_offset, east_offset, edges = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print('time to create grid and edegs: {:.4} seconds'.format((time.time() - t_start)))
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        
        # Define starting point on the grid (this is just grid center)
        grid_start = (75, 25)
        
        # convert start position to current position rather than map center
        grid_start = (int(local_position[0]) - north_offset, int(local_position[1])-east_offset)

        # Set goal as some arbitrary position on the grid
        global_goal = ( -122.397730, 37.796581, 0)  # middle of park
        #global_goal = (-122.398764, 37.793361, 0) # intersection

        # adapt for goal set as latitude / longitude position and convert

        global_goal_wp = global_to_local(global_goal, self.global_home)
        grid_goal = (global_goal_wp[0]-north_offset, global_goal_wp[1] - east_offset)
        print('local goal:', grid_goal)
        #grid_goal = (-north_offset + 150, -east_offset + 150)   
        #grid_goal = (750, 370)

        # Run A* to find a path from start to goal       
        print('Local Start and Goal: ', grid_start, grid_goal)
        print('executing A*')
        t_start = time.time()
        g = nx.Graph()
        for e in edges:
            p1 = e[0]
            p2 = e[1]
            w = np.linalg.norm(np.array(p2) - np.array(p1))
            g.add_edge(p1, p2, weight=w)
        start_node = closest_graph_point(g, grid_start)
        goal_node = closest_graph_point(g, grid_goal)
        print('end points: ', start_node, goal_node)

        print('time for A*: {:.4} seconds'.format((time.time() - t_start)))

        path, cost = a_star_graph(g, heuristic, start_node, goal_node)
        print('path length: {}    path cost: {}'.format(len(path),cost))

        # prune path to minimize number of waypoints
        pruned_path = get_pruned_path(path, grid)
        print('path length after pruning: {}'.format(len(pruned_path)))

        # if True:

        #     plt.imshow(grid, origin='lower', cmap='Greys') 

        #     for e in edges:
        #         p1 = e[0]
        #         p2 = e[1]
        #         plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')
            
        #     for i in range(len(pruned_path)-1):
        #         p1 = pruned_path[i]
        #         p2 = pruned_path[i+1]
        #         plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'g-')
                                
        #     plt.plot(grid_start[1], grid_start[0], 'gx', markeredgewidth=6, markersize=12)
        #     plt.plot(grid_goal[1], grid_goal[0], 'gx', markeredgewidth=6, markersize=12)

        #     plt.xlabel('EAST', fontsize=20)
        #     plt.ylabel('NORTH', fontsize=20)
        #     #plt.show()
        #     plt.savefig('pruned_plan.png', format= 'png')

        # # Convert path to waypoints
        waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), TARGET_ALTITUDE, 0] for p in pruned_path]
        self.waypoints = waypoints
        # send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required threaded
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
