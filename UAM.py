from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from motionControl import MotionControl
from cflib.crtp import init_drivers
from cflib.utils import uri_helper
from threading import Thread
import time
import math

try:
    from config import DRONE_LIST
except Exception:
    pass


class UAM:
    DEFAULT = None

    def __init__(
        self, aircraft_name, aircraft_info=DEFAULT, min_volts_per_cell=3.6, *, kwargs={}
    ):
        """
        Used for high level waypoint navigation and connection to Crazyflie.

        :param aircraft_name: The name of the aircraft, requires that the name exists as an object in Vicon, and in config with proper information
        :param aircraft_info: A tuple of the URI to use to connect to the Crazyflie and the number of cells the drone uses.
                           Default URI is "radio://0/{channel}/2M/E7E7E7E7{drone_id}", channel and drone_id sourced from config
                           Default cells is sourced from config
        :param min_volts_per_cell: The minimum voltage per cell required to take off, default = 3.6
        :param kwargs: Keyword arguments to pass on to MotionControl class as a dictionary, default is empty
        """
        if aircraft_info is self.DEFAULT:
            drone_id, channel, cells = DRONE_LIST[aircraft_name]
            URI = f"radio://0/{channel}/2M/E7E7E7E7{drone_id}"
        else:
            URI, cells = aircraft_info

        init_drivers()

        self._aircraft_name = aircraft_name
        self._uri = uri_helper.uri_from_env(default=URI)
        self._scf = SyncCrazyflie(self._uri)
        self._waypoint_info = None
        self._resume_waypoint_info = None
        self._fly_thread = None
        self._connected = False
        self._grounded = True
        self._motion_control_args = (
            self._scf,
            aircraft_name,
            min_volts_per_cell * cells,
            kwargs,
        )
        self._motion_controller = None

    def set_waypoints(self, waypoints, land_height_at_end=0):
        """
        Set the internal waypoints before starting the movement.

        :param waypoints: A list of tuples (X (m), Y (m), Z (m)(optional), velocity (m/s)(optional), rotation speed (deg/s)(optional)) relative to
                          global position in order of visitation. The optional sections of the tuple can be omitted for example (X, Y)
        :param land_height_at_end: The height to land at (in meters) after finishing waypoints, None means don't land, default=0
        """
        self._waypoint_info = waypoints, land_height_at_end

    def start(self):
        """
        Begins following internal waypoints at default speed. Takes off if grounded. Will end stop state.
        If already following waypoints, will stop and follow new waypoints.

        :raises Exception: 'Cannot start when disconnected' when not connected
        :raises Exception: 'Cannot start path without waypoints' when waypoints haven't been set
        """
        if not self._connected:
            raise Exception("Cannot start when disconnected")
        if self._waypoint_info is None:
            raise Exception("Cannot start path without waypoints")
        if self._fly_thread is not None:
            self._motion_controller.stop()
            self._fly_thread.join()

        self._motion_controller.all_clear()

        if self._grounded:
            self.take_off()

        def fly_waypoints(waypoints, land_at_end):
            self._resume_waypoint_info = waypoints.copy(), land_at_end
            for coords in waypoints.copy():
                self._motion_controller.go_to(*coords)
                if self._motion_controller.is_stopped():
                    self._fly_thread = None
                    return
                self._resume_waypoint_info[0].pop(0)
            if land_at_end:
                self.land(land_at_end)
            self._waypoint_info = None
            self._resume_waypoint_info = None
            self._fly_thread = None

        self._fly_thread = Thread(target=fly_waypoints, args=self._waypoint_info)
        self._fly_thread.start()

    def connect(self):
        """
        Connects to the aircraft, called automatically when entering with statement

        :raises Exception: 'Failed to open link' when open link using URI fails
        :raises Exception: MotionControl init errors will be raised, i.e. low battery and Vicon cannot connect
        """
        if not self._connected:
            try:
                self._scf.open_link()
                self._connected = True    
            except Exception:
                raise Exception(f"Failed to open link with {self._uri}")

            *args, kwargs = self._motion_control_args
            self._motion_controller = MotionControl(*args, **kwargs)

    def disconnect(self):
        """
        Finishes path if following, lands if airborne, then disconnects from the aircraft
        """
        if self._fly_thread:
            self._fly_thread.join()
        if not self._grounded:
            self.land()
        if self._connected:
            self._motion_controller.close_vicon()
            self._motion_controller = None
            self._scf.close_link()
            self._connected = False

    def stop(self):
        """
        Stops aircraft from moving, aircraft may still rotate in place.
        """
        self._motion_controller.stop()

    def resume(self):
        """
        If aircraft is in stop state, resume previous path, ends stop state.
        """
        if self._motion_controller.is_stopped():
            if not self._resume_waypoint_info:
                raise Exception("No waypoints to resume")
            self._waypoint_info = self._resume_waypoint_info
            self._motion_controller.all_clear()
            self.start()

    def block_while_flying(self):
        """
        Blocks while the aircraft is following its waypoints.
        """
        if self._fly_thread:
            self._fly_thread.join()

    def land(self, landing_height=DEFAULT, velocity=DEFAULT, override_stop=True):
        """
        Lands the aircraft immediately

        :param landing_height: The height to land at, default is aircraft default.
        :param velocity: The velocity in m/s to land at, default is aircraft default.
        :param override_stop: Whether to land even if the aircraft is in a stop state, default = True
        """
        if not self._grounded:
            if not override_stop and self._motion_controller.is_stopped():
                return

            self._motion_controller.all_clear()
            self._motion_controller.land(landing_height, velocity)
            self._grounded = True

    def get_status(self, getVbat=True):
        """
        Gets the current status of the aircraft as a tuple.

        :returns: aircraft name, whether connected (bool), whether grounded (bool), x in meters, y in meters, z in meters, battery voltage in volts
        """
        x, y, z, vbat = self._motion_controller.get_status(getVbat)
        return self._aircraft_name, self._connected, self._grounded, x, y, z, vbat
    
    def get_velocity(self, dt=0.01):
        """
        Gets the velocity of the aircraft in m/s. 
        returns: velocity vector (3-tuple), velocity magnitude
        """
        x0, y0, z0, _ = self._motion_controller.get_status(getVbat=False)
        time.sleep(dt)
        x1, y1, z1, _ = self._motion_controller.get_status(getVbat=False)
        Vx = (x1 - x0) / dt
        Vy = (y1 - y0) / dt
        Vz = (z1 - z0) / dt
        speed = math.sqrt(Vx ** 2 + Vy ** 2 + Vz ** 2)
        return Vx, Vy, Vz, speed
    
    def get_acceleration(self, dt=0.01):
        """
        Gets the acceleration vector of the aircraft in m/s squared. 
        returns: acceleration vector (3-tuple), acceleration magnitude
        """
        vx0, vy0, vz0, _ = self.get_velocity()
        time.sleep(dt)
        vx1, vy1, vz1, _ = self.get_velocity()
        Ax = (vx1 - vx0) / dt
        Ay = (vy1 - vy0) / dt
        Az = (vz1 - vz0) / dt
        Amag = math.sqrt(Ax ** 2 + Ay ** 2 + Az ** 2)
        return Ax, Ay, Az, Amag

    def take_off(self, height=DEFAULT, velocity=DEFAULT):
        """
        Commands the aircraft to take off if grounded, will not take off if stopped.

        :param height: The height to take off to, default is aircraft default.
        :param velocity: The velocity in m/s to take off at, default is aircraft default.
        """
        if self._grounded and self._connected:
            if self._motion_controller.take_off(height, velocity) != False:
                self._grounded = False

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
