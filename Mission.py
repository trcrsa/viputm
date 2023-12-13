from config import POINTS_OF_INTEREST as POIs
import numpy as np
import math

class Mission:
    def __init__(self, sender, waypoints, initial_pos, start_time=0):
        """ param sender: the operator that sent this mission to the UTM.
            param initial_pos: a list of [xi, yi, zi] obtained from UAM.get_status()
            param waypoints: a list of one or more mission destinations
            [ [x, y, z, wait time, height to land],
              [x, y, z, wait time, height to land],
              [x, y, z, wait time, height to land]...]
            If height to land = None, the aircraft will hover in place for the wait time instead of landing.
            The string name of a POI in config can be used to replace x, y, z.
            The wait time of the last destination should always be 0. """

        self.id = (sender.aircraft_name, len(sender.missions))
        self.operator = sender
        self.in_progress = False
        self.takeoff_height = 0.5
        self.speed = 0.5
        self.start_pos = list(initial_pos)
        self.start_time = start_time
        self.points = []
        self.POI = []
        self.landing_heights = []
        self.wait_times = []

        for i in waypoints:
            if isinstance(i[0], str):
                point = list(POIs[i[0]])
                point[2] += point.pop()
                self.points.append(point)
                self.wait_times.append(i[1])
                self.POI.append(i[0])
            else:
                self.points.append([i[j] for j in range(3)])
                self.wait_times.append(i[3])
                self.POI.append(False)

            self.landing_heights.append(i[-1])

        self.external_delay = 0.5  # processing delay (seconds) to be accounted for in pre-flight mission maps
        self.resolution = 100  # how many discrete (time, position) points exist in the mission map for each second

        self.map = []
        """  A numpy array of [  [t0, x0, y0, z0],
                                 [t1, x1, y1, z1],
                                 [t2, x2, y2, z2], ...]
             Represents the predicted position of a drone flying the mission at each point in time. """

        self.itinerary = {}
        """ A dictionary of { destination: [arrival time, departure time],
                              destination: [arrival time, departure time], ...,
                              destination: [arrival time, end time] }
            Each destination is a (x, y, z) tuple, or a string of a POI name if the destination is a POI. """

        self.end_time = 0
        self.update_mission_info()

    def preflight_edit(self, n, newLocation=False, newWaitTime=False, newLandHeight=False, newStartTime=False):
        """ Makes changes to a mission. Must bfor index, PAL ie done before the drone begins the mission.
            If a param is False, that part of the mission will not be changed.
            :raises Exception: 'Cannot edit a mission that already started' """

        if self.in_progress:
            raise Exception(f'Cannot edit in-progress mission {self.id}.')

        if isinstance(newLocation, str):
            self.POI[n] = newLocation
            newLocation = list(POIs[newLocation])
            newLocation[2] += newLocation.pop()
            self.itinerary = {}

        if newLocation:
            self.points[n] = newLocation
        if newWaitTime:
            self.wait_times[n] = newWaitTime
        if newStartTime:
            self.start_time = newStartTime
        if newLandHeight != False:  # since landHeight can be None, I can't use "if newLandHeight"
            self.landing_heights[n] = newLandHeight

        self.end_time = 0
        self.update_mission_info()

    def update_mission_info(self):
        """ Updates the mission map and the mission itinerary."""

        # initial takeoff
        starting_state = np.insert(np.array(self.start_pos, dtype=float), 0, self.start_time)  # [t, x, y, z]
        takeoff_spot = np.copy(starting_state[1:])
        takeoff_spot[-1] += self.takeoff_height
        self.map = self.get_path_map(starting_state, takeoff_spot, self.speed)

        for ind, destination in enumerate(self.points):
            is_final_stop = (destination is self.points[-1])  # True if this is the last destination, False otherwise

            # path to destination
            main_path = self.get_path_map(self.map[-1], destination, self.speed)
            self.map = np.concatenate((self.map, main_path[1:]))
            self.map = np.round(self.map, 3)

            if self.POI[ind]:
                currStop = self.POI[ind]
            else:
                currStop = tuple(destination)
            self.itinerary[currStop] = [self.map[-1][0] - self.external_delay]

            # landing and takeoff paths, if we are landing at this spot
            if self.landing_heights[ind] is not None:
                # landing
                prev_spot = np.copy(self.map[-1][1:])
                landing_spot = np.copy(self.map[-1][1:])
                landing_spot[-1] = self.landing_heights[ind]
                landing_path = self.get_path_map(self.map[-1], landing_spot, self.speed, not is_final_stop)
                self.map = np.concatenate((self.map, landing_path[1:]))

                if is_final_stop:  # end if this is the last destination
                    self.map = np.round(self.map, 3)
                    end_time = self.map[-1, 0]
                    self.itinerary[currStop].append(end_time)
                    self.end_time = end_time
                    return

                # waiting stationary after landing
                wait_period = self.get_stationary_map(self.map[-1], self.wait_times[ind])
                self.map = np.concatenate((self.map, wait_period[1:]))

                # takeoff
                takeoff_path = self.get_path_map(self.map[-1], prev_spot, self.speed)
                self.map = np.concatenate((self.map, takeoff_path[1:]))

            else:
                # waiting stationary in midair
                wait_period = self.get_stationary_map(self.map[-1], self.wait_times[ind])
                self.map = np.concatenate((self.map, wait_period[1:]))

            self.itinerary[currStop].append(self.map[-1][0])

    def get_path_map(self, initial_state, destination, speed, do_ext_delay=True):
        """ Helper function used in update_mission_info.
            :param initial_state: initial [t, x, y, z].
            :param destination: final [x, y, z].
            :param speed: speed of aircraft in m/s.
            :param do_ext_delay: wether or not to account for processing delays.
            :returns: a numpy array of [ [t0, x0, y0, z0],
                                        [t1, x1, y1, z1],
                                        [t2, x2, y2, z2], ...] between two (time, position) pairs."""

        distance = math.dist(initial_state[1:], destination)
        travel_time = np.round(distance / speed, 2)  # TODO round this to correct precision based on resolution
        final_state = np.insert(destination, 0, initial_state[0] + travel_time)

        path_map = np.linspace(initial_state, final_state, int(travel_time * self.resolution) + 1)

        if do_ext_delay:
            ext_delay = self.get_stationary_map(final_state, self.external_delay)
            path_map = np.concatenate((path_map, ext_delay[1:]))

        return path_map

    def get_stationary_map(self, initial_state, wait_time):
        """ Same as get_path_map, except the array is of the aircraft waiting stationary at a position.
            :param initial_state: initial [t, x, y, z].
            :param wait_time: time(s) to wait at this [x, y, z]. """

        final_state = np.copy(initial_state)
        final_state[0] += wait_time
        return np.linspace(initial_state, final_state, int(wait_time * self.resolution) + 1)

    def __str__(self):
        print(f'Mission {self.id}, starting at {self.start_pos} at t={self.start_time}:')
        for i, destination in enumerate(self.points):
            if self.landing_heights[i] is None:
                print(f'Go to {destination} and hover there for {self.wait_times[i]}s.')
            else:
                print(f'Go to {destination}, land at height {self.landing_heights[i]}, and wait for {self.wait_times[i]}s.')
        return ''


if __name__ == '__main__':
    class DummyOperator:
        aircraft_name = 0
        missions = [0]
    np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

    m1 = Mission(DummyOperator(), [['office', 2, 0.815], ['heli5', 0, 0]], [1.260, 2.400, 0])
    m2 = Mission(DummyOperator(), [['office', 2, 0.815], ['heli3', 0, 0]], [1.055, 2.790, 0])

    print(m1.itinerary)
    m1.preflight_edit(0, 'office_lp1')
    print(m1.itinerary)
