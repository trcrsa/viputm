import asyncio
import math
import time
from threading import Thread
import socket
import struct
from scipy.spatial.transform import Rotation
from config import POINTS_OF_INTEREST, TAKE_OFF_HEIGHT, DRONE_LIST
from udpReader import parseInfo

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper

# The name of the rigid body in Vicon that represents the Crazyflie
rigid_body_name = "Bolt"  # "Drone04"


drone_id, channel, cells = DRONE_LIST[rigid_body_name]

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default=f"radio://0/{channel}/2M/E7E7E7E7{drone_id}")


# True: send position and orientation; False: send position only
send_full_pose = True

# When using full pose, the estimator can be sensitive to noise in the orieFalsetation data when yaw is close to +/- 90
# degrees. If this is a problem, increase orientation_std_dev a bit. The default value in the firmware is 4.5e-3.
orientation_std_dev = 8.0e-3


class ViconHandler(Thread):
    def __init__(self, body_name, UDP_IP="255.255.255.255", UDP_PORT=51001):
        self.UDP_IP = UDP_IP
        self.UDP_PORT = UDP_PORT

        Thread.__init__(self)

        self.body_name = body_name
        self.on_pose = None
        self.connection = None
        self._stay_open = True

        self.start()

    def close(self):
        self._stay_open = False
        self.join()

    def run(self):
        asyncio.run(self._life_cycle())

    async def _life_cycle(self):
        await self._connect()
        while self._stay_open:
            await asyncio.sleep(1)
        await self._close()

    async def _connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.sock.bind((self.UDP_IP, self.UDP_PORT))

        time.sleep(5)

        while self._stay_open:
            data, _ = self.sock.recvfrom(256)
            try:
                trans_x, trans_y, trans_z, rot_x, rot_y, rot_z = parseInfo(
                    data, rigid_body_name
                )
            except Exception:
                return

            # print(
            #     f"{trans_x:.2f} {trans_y:.2f} {trans_z:.2f} {rot_x:.2f} {rot_y:.2f} {rot_z:.2f}"
            # )

            trans_x /= 1000
            trans_y /= 1000
            trans_z /= 1000
            r = Rotation.from_euler("xyz", [rot_x, rot_y, rot_z], degrees=False)

            if self.on_pose:
                self.on_pose([trans_x, trans_y, trans_z, r.as_matrix()])
            else:
                print("problem")
                exit()

    async def _close(self):
        self._stay_open = False


def send_extpose_rot_matrix(cf, x, y, z, rot):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a (3x3)
    rotaton matrix. This is going to be forwarded to the Crazyflie's
    position estimator.
    """
    quat = Rotation.from_matrix(rot).as_quat()

    if send_full_pose:
        cf.extpos.send_extpose(x, y, z, quat[0], quat[1], quat[2], quat[3])

    else:
        cf.extpos.send_extpos(x, y, z)


def adjust_orientation_sensitivity(cf):
    cf.param.set_value("locSrv.extQuatStdDev", orientation_std_dev)


def activate_kalman_estimator(cf):
    cf.param.set_value("stabilizer.estimator", "2")

    # Set the std deviation for the quaternion data pushed into the
    # kalman filter. The default value seems to be a bit too low.
    cf.param.set_value("locSrv.extQuatStdDev", 0.06)


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
    if t > 0:
        time.sleep(t)
        logconf.stop()


def fly_to(building):
    with PositionHlCommander(scf, default_height=TAKE_OFF_HEIGHT) as hl:
        # threaded_log(scf, lg_stab, 3)
        # hl.go_to(1.054, 2.79, 0.5)
        # hl.go_to(1, 1, 1)
        x, y, z, h = POINTS_OF_INTEREST[building]
        hl.go_to(x, y, z + h)
        hl.set_landing_height(z)
        hl._default_velocity = 0.1


if __name__ == "__main__":
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name="Stabilizer", period_in_ms=10)
    lg_stab.add_variable("stateEstimate.x", "float")
    lg_stab.add_variable("stateEstimate.y", "float")
    lg_stab.add_variable("stateEstimate.z", "float")

    lg_stab.add_variable("stateEstimate.roll", "float")
    lg_stab.add_variable("stateEstimate.pitch", "float")
    lg_stab.add_variable("stateEstimate.yaw", "float")
    # lg_stab.add_variable("pm.vbat", "float")

    # Connect to Vicon
    vicon = ViconHandler(rigid_body_name)

    # lg_stab.add_variable("stateEstimate.y", "float")
    # lg_stab.add_variable("stateEstimate.z", "float")

    # lg_stab.add_variable("stateEstimate.roll", "float")
    # lg_stab.add_variable("stateEstimate.pitch", "float")
    # lg_stab.add_variable("stateEstimate.yaw", "float")
    with SyncCrazyflie(uri) as scf:
        cf = scf.cf

        # Set up a callback to handle data from Vicon
        vicon.on_pose = lambda pose: send_extpose_rot_matrix(cf, *pose)

        adjust_orientation_sensitivity(cf)
        activate_kalman_estimator(cf)

        threaded_log(scf, lg_stab, 1000)
        # time.sleep(1000)
        time.sleep(2)

        # fly_to("factory_high")
        # fly_to("hospital")
        fly_to("office")
        fly_to("lp1")
        # with PositionHlCommander(scf) as hl:
        #     time.sleep(1)
        # hl = PositionHlCommander(scf)
        # hl.take_off()
        # print(hl)
        # time.sleep(1)
        # hl.land()

        # with PositionHlCommander(scf, default_height=0.5) as hl:
        #     # threaded_log(scf, lg_stab, 3)
        #     # hl.go_to(1.054, 2.79, 0.5)
        #     # hl.go_to(1, 1, 1)
        #     hl.go_to(3.2, 4.2, 1.5)
        #     hl.set_landing_height(0.85)
        #     hl._default_velocity = 0.1

        # with PositionHlCommander(scf) as hl:
        #     # threaded_log(scf, lg_stab, 3)
        #     # hl.go_to(3.2, 4.2, 1.5)
        #     hl.go_to(1.055, 2.8, 0.5)
        #     # hl.set_landing_height(.85)
        #     hl._default_velocity = 0.1

    vicon.close()
