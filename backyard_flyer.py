# -*- coding: utf-8 -*-
"""
Solution to the Backyard Flyer Project.
"""

import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


def euclidean_distance(a, b):
    """

    Returns euclidian distance between points a and b
    """
    dist = np.linalg.norm(a - b)
    return dist


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
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
            altitude = -1.0 * self.local_position[2]

            # check if altitude is within 95% of target
            if altitude > 0.95 * self.target_position[2]:
                # calculate flight path
                self.all_waypoints = self.calculate_box()
                self.waypoint_transition()

            # nothing else to do in takeoff mode.
            return

        elif self.flight_state == States.WAYPOINT:
            if euclidean_distance(self.target_position[0:2], self.local_position[0:2]) < 1.0:
                if len(self.all_waypoints) > 0:
                    self.waypoint_transition()
                elif euclidean_distance(self.local_velocity[0:2], self.target_position[0:2]) < 1.0:
                    self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1 \
                    and abs(self.local_position[2]) < 0.01:
                # Houston, the Eagle has landed.
                self.disarming_transition()

    def state_callback(self):
        if not self.in_mission:
            # nothing to do
            return

        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            if not self.armed and not self.guided:
                self.manual_transition()

    def calculate_box(self):
        # keep flying at the same altitude.
        return [[10.0, 0.0, 3.0],
                [10.0, 10.0, 3.0],
                [0.0, 10.0, 3.0],
                [0.0, 0.0, 3.0]]

    def arming_transition(self):
        print("arming transition")
        self.take_control()
        self.arm()
        # set the current location to be the home position
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])

        self.flight_state = States.ARMING

    def takeoff_transition(self):
        print("takeoff transition")
        target_altitude = 3.0  # safely flying at 3 meters off the ground
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print("waypoint transition")
        self.target_position = self.all_waypoints.pop(0)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0.0)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.release_control()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        print("manual transition")
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """
        Initialize logs and start connection.
        Call super class method (run event loop)
        Then exit when done.
        """
        self.start_log("Logs", "NavLog.txt")
        # connection done in super class
        super().start()
        self.stop_log()


if __name__ == "__main__":
    conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
