import math
import time
import socket
import struct
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from scipy.spatial.transform import Rotation
from threading import Thread
from cflib.utils import uri_helper
from config import DRONE_LIST

import cflib.crtp


def send_extpose(cf, x, y, z, rot):
    """
    Send the current Crazyflie X, Y, Z position and attitude as
    a scipy.spatial.transform.Rotation object. This is going to
    be forwarded to the Crazyflie's position estimator.

    :param cf: Crazyflie object to forward pose information to
    :param x: x position in meters to be forwarded to the Crazyflie
    :param y: y position in meters to be forwarded to the Crazyflie
    :param z: z position in meters to be forwarded to the Crazyflie
    :param rot: scipy.spatial.transform.Rotation whose orientation
                will be forwarded to the Crazyflie
    """
    quat = rot.as_quat()
    cf.extpos.send_extpose(x, y, z, quat[0], quat[1], quat[2], quat[3])


def adjust_orientation_sensitivity(cf, orientation_std_dev):
    """
    Adjusts the Crazyflie's orientation sensitivity parameter

    :param cf: The Crazyflie object whose parameter will be changed
    :orientation_std_dev: The new value of the orientation sensitivity
    """
    cf.param.set_value("locSrv.extQuatStdDev", orientation_std_dev)


def activate_kalman_estimator(cf):
    """
    Activates the kalman estimator of a Crazyflie object

    :param cf: The Crazyflie object whose kalman estimator is activated
    """
    cf.param.set_value("stabilizer.estimator", "2")


def parseInfo(data, name):
    """
    Parses the UDP object stream from the Vicon System to get pose
    information, returns x,y,z (in mm) and x,y,z euler angles (in rad)
    If the data is malformed or does not exist, raises Exception

    :param data: The data from the Vicon UDP object stream
    :param name: The name of the object whose position is wanted
    """
    
    SHIFT = 75
    items = data[4]
    name = name.encode("utf8") + b"\x00" * (24 - len(name))
    for i in range(items):
        item_shift = SHIFT * i
        if data[8 + item_shift : 32 + item_shift] != name:
            continue
        (trans_x,) = struct.unpack("d", data[32 + item_shift : 40 + item_shift])
        (trans_y,) = struct.unpack("d", data[40 + item_shift : 48 + item_shift])
        (trans_z,) = struct.unpack("d", data[48 + item_shift : 56 + item_shift])
        (rot_x,) = struct.unpack("d", data[56 + item_shift : 64 + item_shift])
        (rot_y,) = struct.unpack("d", data[64 + item_shift : 72 + item_shift])
        (rot_z,) = struct.unpack("d", data[72 + item_shift : 80 + item_shift])

        if math.isnan(trans_x) or math.isnan(trans_y) or math.isnan(trans_z):
            break
        if trans_x == trans_y == trans_z == 0:
            break

        return trans_x, trans_y, trans_z, rot_x, rot_y, rot_z
    raise Exception()


class MotionControl:
    DEFAULT = None

    def __init__(
        self,
        crazyflie,
        drone_name,
        min_voltage,
        default_velocity=0.5,
        default_height=0.5,
        default_rotation_speed=30,
        default_landing_height=0.0,
        orientation_std_dev=8e-2,
        UDP_ADDRESS=("255.255.255.255", 51001),
    ):
        """
        Construct an instance of a MotionControl

        :param crazyflie: A Crazyflie or SyncCrazyflie instance
        :param drone_name: The name of the drone object in Vicon
        :param min_voltage: The minimum voltage to fly with
        :param default_velocity: The default velocity to use
        :param default_height: The default height to fly at
        :param default_rotation_speed: The default speed to rotate to face the direction of movement in deg/sec, 0 if no rotation
        :param default_landing_height: Landing height (zero if not specified); for landing on objects off the ground
        :param UDP_ADDRESS: A tuple with the IP address and port number to connect to the Vicon
        """

        if isinstance(crazyflie, SyncCrazyflie):
            self._cf = crazyflie.cf
        else:
            self._cf = crazyflie

        self._x, self._y, self._z = 0, 0, 0
        self._relative_yaw, self._absolute_yaw = 0, 0
        self._vbat = 0
        self._is_flying = False
        self._moving = False
        self._use_vicon = True
        self._default_height = default_height
        self._default_velocity = default_velocity
        self._default_landing_height = default_landing_height
        self._default_rotation_speed = default_rotation_speed
        self._hl_commander = self._cf.high_level_commander
        self._vicon_data = drone_name, UDP_ADDRESS
        self._stop = False

        print("vbat", self._get_vbat())

        if self._vbat < min_voltage:
            raise Exception(f"Battery below {min_voltage} at {self._vbat}")

        adjust_orientation_sensitivity(self._cf, orientation_std_dev)
        activate_kalman_estimator(self._cf)
        self._start_vicon(*self._vicon_data)

    def _wait(self, wait_time):
        """
        Blocks for the specified amount of time or until a self._stop is true

        :param wait_time: The number of seconds the wait at maximum
        """
        start = time.time()
        while time.time() - start < wait_time and not self._stop:
            time.sleep(0.01)

    def _start_vicon(self, drone_name, UDP_address):
        """
        Starts a thread that will receive and parse Vicon data, sending it to the drone
        raises Exception if the Vicon doesn't connect or object is not in UDP stream

        :param drone_name: The name of the object in Vicon that corresponds to the drone
        :param UDP_address: The address for the Vicon to connect to as a tuple of (IP_Address, Port)
        """

        def update_position(drone_name, UDP_address):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            sock.bind(UDP_address)
            init_absolute = True
            set_absolute_rate = 10
            iter = 0
            while self._use_vicon:
                data, _ = sock.recvfrom(256)
                try:
                    trans_x, trans_y, trans_z, rot_x, rot_y, rot_z = parseInfo(
                        data, drone_name
                    )
                except Exception:
                    continue

                trans_x /= 1000
                trans_y /= 1000
                trans_z /= 1000
                r = Rotation.from_euler("xyz", [rot_x, rot_y, rot_z], degrees=False)

                send_extpose(self._cf, trans_x, trans_y, trans_z, r)

                self._x = trans_x
                self._y = trans_y
                self._z = trans_z
                self._relative_yaw = rot_z

                if init_absolute:
                    self._absolute_yaw = rot_z
                    init_absolute = False

                if iter != set_absolute_rate:
                    iter += 1
                else:
                    rotations = 1  # 1 -1 2 -2 ...
                    new_angle = rot_z
                    while abs(self._absolute_yaw - new_angle) > math.pi:
                        new_angle = rot_z + rotations * math.tau
                        rotations = -rotations if rotations > 0 else -rotations + 1
                    self._absolute_yaw = new_angle
                    iter = 0

        t = Thread(target=update_position, args=(drone_name, UDP_address), daemon=True)
        t.start()
        time.sleep(1)
        if self._x == self._x == self._x == self._relative_yaw == 0:
            raise Exception(
                f"Not receiving Vicon data from {drone_name} on {UDP_address[0]}:{UDP_address[1]}"
            )

    def go_to(self, x, y, z=DEFAULT, velocity=DEFAULT, rotation_speed=DEFAULT):
        """
        Go to a position

        :param x: X coordinate
        :param y: Y coordinate
        :param z: Z coordinate
        :param velocity: The velocity (meters/second)
        :param rotation_speed: The rotation speed (degrees/second)
        :return:
        """
        if rotation_speed is self.DEFAULT:
            rotation_speed = self._default_rotation_speed
        if z is self.DEFAULT:
            z = self._z
        if velocity is self.DEFAULT:
            velocity = self._default_velocity

        dx = x - self._x
        dy = y - self._y
        dz = z - self._z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if distance < 0.07:
            return
        if math.sqrt(dx * dx + dy * dy) < 0.07:
            rotation_speed = 0

        if self._stop:
            return
        if rotation_speed > 0:
            """
            If your curious why the drone sometimes over turns and then corrects itself
            when facing in the direction it's moving, this loop is why. Without this loop,
            The drone will sometimes not face exactly in the direction it's supposed to.
            This is because there is no good way (that I can figure out) to face the direction
            it's moving. When giving a _hl_commander.go_to it can either be relative or absolute,
            when it's relative, the position is correct but the angle can be wrong sometimes,
            this is the option I choose. The alternative is that the position is wrong, ie. if
            you used _hl_commander.go_to(self._x, self._y, self._z, angle, duration_s) then the
            x, y and z terms lag behind the drone's actual position (no idea why) so it's worse.
            Therefore, it's better to just correct. 3 is probably overkill but I don't think it
            has much of a negative impact vs 2.
            """
            correction_factor = 3
            for _ in range(correction_factor):
                angle = math.atan2(dy, dx)
                da = angle - self._relative_yaw
                da = da + math.tau if da < -math.pi else da
                da = da - math.tau if da > math.pi else da
                duration_s = abs(da) / (rotation_speed * math.pi / 180)

                self._hl_commander.go_to(0, 0, 0, da, duration_s, relative=True)
                self._wait(duration_s)

            if self._stop:
                return

        angle = self._absolute_yaw

        duration_s = distance / velocity
        self._moving = True
        self._hl_commander.go_to(x, y, z, angle, duration_s)
        self._wait(duration_s)
        self._moving = self._stop

    def stop(self):
        """
        Stops the drone, if the drone is landing or taking off, it will not stop
        If the drone is moving or rotating, it will stop immediately.
        While in the stop command, MotionControl will ignore all commands.
        Including landing and taking off.
        """
        self._stop = True
        time.sleep(0.05)
        if self._moving:
            self._hl_commander.go_to(0, 0, 0, 0, 0, relative=True)

    def all_clear(self):
        """
        Removes the stop state. If the drone was not in a stop state, does nothing
        """
        self._stop = False

    def land(self, landing_height=DEFAULT, velocity=DEFAULT):
        """
        Lands the drone.

        :param landing_height: The height off the ground to land the drone at. Used to land on raised platforms
                               The default value is the the default_landing_height value set at initialization
        :param velocity: The velocity to land the drone at. Default is default_velocity set at initialization
        """
        if self._stop:
            return
        if landing_height is self.DEFAULT:
            landing_height = self._default_landing_height

        if velocity is self.DEFAULT:
            velocity = self._default_velocity

        landing_height -= 0.1  # Compensate for high landing
        if self._is_flying:
            duration_s = (self._z - landing_height) / velocity
            if duration_s > 0:
                self._hl_commander.land(landing_height, duration_s, yaw=None)
                time.sleep(duration_s)

            self._hl_commander.stop()
            self._is_flying = False

    def get_status(self, getVbat=True):
        """
        Returns the position (in units of meters) relative to the coordinate axis
        and battery voltage as a tuple

        :returns: x, y, z, vbat
        """
        if getVbat:
        	return self._x, self._y, self._z, self._get_vbat()
        else:
        	return self._x, self._y, self._z, 0

    def take_off(self, height=DEFAULT, velocity=DEFAULT):
        """
        Takes off the drone. Returns False if take_off was prevented by stop state

        :param height: The height from the take off surface to take off to.
                       The default value is the the default_height value set at initialization
        :param velocity: The velocity to take off at. Default is default_velocity set at initialization
        """
        if self._stop:
            return False

        if height is self.DEFAULT:
            height = self._default_height

        if velocity is self.DEFAULT:
            velocity = self._default_velocity

        if self._is_flying:
            raise Exception("Already flying")

        if not self._cf.is_connected():
            raise Exception("Crazyflie is not connected")

        self._is_flying = True

        height += self._z

        duration_s = height / velocity

        self._hl_commander.takeoff(height, duration_s, yaw=None)
        time.sleep(duration_s)

    def _get_vbat(self):
        """
        Returns the current battery voltage
        """
        lg_stab = LogConfig(name="Stabilizer", period_in_ms=10)
        lg_stab.add_variable("pm.vbat", "float")

        def log_stab_callback(timestamp, data, logconf):
            for name, value in data.items():
                if name == "pm.vbat":
                    self._vbat = float(value)

        self._cf.log.add_config(lg_stab)
        lg_stab.data_received_cb.add_callback(log_stab_callback)
        lg_stab.start()
        time.sleep(5)
        lg_stab.stop()
        return self._vbat

    def close_vicon(self):
        """
        Closes link to Vicon system, stops sending position information
        """
        if not self._is_flying:
            self._use_vicon = False

    def is_stopped(self):
        """
        Returns whether the drone is in the stop state
        """
        return self._stop


def threaded_log(scf, logconf, time):
    t = Thread(target=log, args=(scf, logconf, time), daemon=True)
    t.start()


def log_stab_callback(timestamp, data, logconf):
    for name, value in data.items():
        print(f"{name} {value:.2f}")


def log(scf, logconf, t):
    cf = scf.cf
    cf.log.add_config(logconf)

    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    time.sleep(t)
    logconf.stop()


if __name__ == "__main__":
    drone_name = "Drone04"

    drone_id, radio, cells = DRONE_LIST[drone_name]

    # URI to the Crazyflie to connect to
    uri = uri_helper.uri_from_env(default=f"radio://0/{radio}/2M/E7E7E7E7{drone_id}")

    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name="Stabilizer", period_in_ms=10)
    # lg_stab.add_variable("stateEstimate.x", "float")
    # lg_stab.add_variable("stateEstimate.y", "float")
    # lg_stab.add_variable("stateEstimate.z", "float")

    # lg_stab.add_variable("stateEstimate.roll", "float")
    # lg_stab.add_variable("stateEstimate.pitch", "float")
    lg_stab.add_variable("stateEstimate.yaw", "float")
    # lg_stab.add_variable("pm.vbat", "float")

    with SyncCrazyflie(uri) as scf:
        mc = MotionControl(scf, drone_name, 3.6 * cells)

        # mc._on_pose = lambda pose: send_extpose_rot_matrix(cf, *pose)
        # lg_stab = LogConfig(name="Stabilizer", period_in_ms=10)
        # cf.log.add_config(lg_stab)
        # lg_stab.start()
        # time.sleep(0.1)
        # lg_stab.stop()

        # threaded_log(scf, lg_stab, 1000)
        # hl = PositionHlCommander(scf)
        # hl.take_off()
        # time.sleep(1)
        # hl.land()
        # mc._use_vicon = False

        # with PositionHlCommander(scf) as hl:
        #     hl.go_to(2, 2, 1)
        # time.sleep(1000)

        def s():
            mc.take_off()
            mc.go_to(1, 2, 1)
            mc.land()

        def w():
            time.sleep(30)

        t = Thread(target=s)
        t.start()
        t.join()
        # time.sleep(3)
        # mc.stop()
        # time.sleep(10)
        # mc._stop = False
        # mc.land()
        # mc.take_off()
        # mc.go_to(1, 1, 1)
        # mc.land()
        # time.sleep(5)
        # mc.stop()
        # t.join()

        # mc.take_off()
        # # # mc.go_to(1.1, 1, 1)
        # # # mc.go_to(1, 1.1, 1)
        # mc.go_to(3.340, 4.325, 0.815 + 0.5)
        # mc.go_to((3.340 + 1.055) / 2, (6.325 + 2.790) / 2, 0.815 + 0.5)
        # mc.go_to(1.055, 2.790, 0 + 0.5)
        # mc.go_to(3.340, 4.325, 0.815 + 0.5)
        # mc.go_to((3.340 + 1.055) / 2, (6.325 + 2.790) / 2, 0.815 + 0.5)
        # mc.go_to(1.055, 2.790, 0 + 0.5)
        # mc.land()

        # time.sleep(16.5)
        # print("stop")
        # mc.stop()
        # time.sleep(10)
        # mc._stop = False
        # mc.land()
