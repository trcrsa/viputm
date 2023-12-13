from UAM import UAM
from Mission import Mission
import config
import time

class Operator:
    def __init__(self, aircraft_name, utm):
        """
        Object for managing a single aircraft and communicating to the UTM.
        The same instance of the UTM class must be used for every operator in the script.
        """

        self.uam = UAM(aircraft_name)
        self.uam.connect()
        self.utm = utm
        self.aircraft_name = aircraft_name
        self.POIs = config.POINTS_OF_INTEREST
        self.missions = []
        self.min_vbat = 3.8
        self.start_pos_info = None  # helps un-reserve the final stop of the previous mission when a new one begins

    def send_mission(self, waypoints, start_time=0):
        """
        Generates a mission object, checks it by the UTM to ensure it will not cause any conflicts, and stores the
        edited mission in self.missions. If fly == True, waits start_time seconds and begins flying the mission.

        :param waypoints: a list of one or more mission destinations
            [[x, y, z, wait time, height to land],
            [x, y, z, wait time, height to land],
            [x, y, z, wait time, height to land],
            ...]
        :param start_time: time (s) after the request is sent to start the mission.
        If height to land = None, the aircraft will hover in place for the wait time instead of landing.
        The string name of a point of interest in config can be used to replace x, y, z.
        """
        _, _, _, xi, yi, zi, vbat = self.uam.get_status(getVbat=False)
        request = Mission(self, waypoints, [xi, yi, zi], start_time=start_time)
        mission = self.utm.resolve_request(request)
        self.missions.append(mission)

    def fly(self):
        """ Flies the earliest requested mission that hasn't already been flown. """
        mission = self.missions.pop(0)
        self.utm.confirmed_missions.remove(mission)
        self.utm.ongoing_missions.append(mission)
        numStops = len(mission.landing_heights)
        mission.in_progress = True
        print(f'Mission {mission.id} just started')

        for indx, destination in enumerate(mission.points):
            self.uam.set_waypoints([tuple(destination)], mission.landing_heights[indx])
            self.uam.start()

            if self.start_pos_info:
                self.utm.reserved.remove(self.start_pos_info)
                self.start_pos_info = None

            self.uam.block_while_flying()

            time.sleep(mission.wait_times[indx])

            if mission.landing_heights[indx] is not None:
                self.uam.take_off()

            if indx != numStops:
                self.utm.reserved.remove(mission.POI[indx])
            else:
                self.start_pos_info = mission.POI[indx]

        self.uam.land()
        self.utm.ongoing_missions.remove(mission)
