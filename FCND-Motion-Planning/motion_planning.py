import argparse
import time
import msgpack
from enum import Enum, auto
import random
import matplotlib.pyplot as plt


import numpy as np

from planning_utils import a_star, heuristic, create_grid, prune_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

DRONE_SPEED = 8.9 # Found experimentaly
class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()

goal_ll = [0,0]
class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.path_to_plt = []
        self.start_position_ = []
        self.goal_position_ = []
        self.grid_ = []

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)
    def convert_range(self,original_value, original_range, new_range):
        # todo, change variable names
        original_min, original_max = original_range
        new_min, new_max = new_range
        original_value_normalized = (original_value - original_min) / (original_max - original_min)
        new_value = new_min + original_value_normalized * (new_max - new_min)
        return new_value

    def compute_deadband_radius(self, deadband_range, range_spead, local_velocity):

        drone_velocity = np.linalg.norm(local_velocity[0:2])
        deadband_radius = self.convert_range(drone_velocity, range_spead, deadband_range)
        return deadband_radius

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < self.compute_deadband_radius([0.1, 5], [0, DRONE_SPEED], self.local_velocity):
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
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
        self.plot_path()

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)
    def obtain_lat0_lon0(self,data):
        with open(data) as f:
            first_row = f.readline().strip('\n')
            pose_ = first_row.split(", ")
            lat0 = float(pose_[0].split(" ")[1])
            lon0 = float(pose_[1].split(" ")[1])
        return lat0, lon0
    def plot_path(self):
        plt.imshow(self.grid_, cmap='Greys', origin='lower')

        # For the purposes of the visual the east coordinate lay along
        # the x-axis and the north coordinates long the y-axis.
        plt.plot(self.start_position_[1], self.start_position_[0], 'x')
        plt.plot(self.goal_position_[1], self.goal_position_[0], 'x')

        if self.path_to_plt is not None:
            pp = np.array(self.path_to_plt)
            plt.plot(pp[:, 1], pp[:, 0], 'g')

        plt.xlabel('EAST')
        plt.ylabel('NORTH')
        plt.show()

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        lat0, lon0 = self.obtain_lat0_lon0("colliders.csv")
        print('Lat0:{0}, Lat0:{1}'.format(lat0, lon0))
        
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0,0)

        # TODO: retrieve current global position
        current_global_position = self.global_position

 
        # TODO: convert to current local position using global_to_local()
        # east_local, west_local, down_local = global_to_local(current_global_position,self.global_home())
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        self.grid_ = grid
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        # grid_start = (-north_offset, -east_offset)
        # TODO: convert start position to current position rather than map center
        start_position = (int(-north_offset+self.local_position[0]), int(-east_offset+self.local_position[1]))
        grid_start = start_position
        self.start_position_ = grid_start
        # Set goal as some arbitrary position on the grid
        # grid_goal = (random.randint(0, -north_offset*2), random.randint(0, -east_offset*2))
        # grid_goal = (int(start_position[0] + 10), int(start_position[1] + 10))

        # TODO: adapt to set goal as latitude / longitude position and convert
        # convert global goal to local goal position 
        goal_ll_np = np.array([goal_ll[1], goal_ll[0], 0])
        goal_local_position = global_to_local(goal_ll_np, self.global_home)
        # include map offset
        grid_goal = (int(-north_offset+goal_local_position[0]), int(-east_offset+goal_local_position[1]))
        self.goal_position_ = grid_goal

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal) #include in planning_utils
        print(len(path))
        # TODO: prune path to minimize number of waypoints
        prune_path_ = prune_path(path)
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        self.path_to_plt = prune_path_
        print(len(prune_path_))
        # Convert path to waypoints 
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in prune_path_]
        # Include heading 
        for wp in range(len(waypoints)-1):
            print(wp)
            wp1 = waypoints[wp]
            wp2 = waypoints[wp+1]
            print(wp1)
            waypoints[wp+1][3] = np.arctan2((wp2[1]-wp1[1]), (wp2[0]-wp1[0]))
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()
        

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
    parser.add_argument('--gps_goal_lat', type=float, default=37.794955, help="Goal lat")
    parser.add_argument("-gps_goal_lon", type=float, default=-122.397962, help="Goal longitud")
    args = parser.parse_args()
    goal_ll[0] = args.gps_goal_lat
    goal_ll[1] = args.gps_goal_lon

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
