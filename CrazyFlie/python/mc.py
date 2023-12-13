import logging
import cflib
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E701")

logging.basicConfig(level=logging.ERROR)

if __name__ == "__main__":
    cflib.crtp.init_drivers()
    # We connect when the Sync object is created
    with SyncCrazyflie(URI) as scf:
        # We take off when the commander is created
        with MotionCommander(scf) as mc:
            pass
            # Move one meter forward
            # mc.up(0.2)
            # for _ in range(25):
            #     mc.forward(0.1)

            # mc.turn_right(180)
            # mc.forward(2.5, velocity=0.5)

            # mc.down(0.2)

            # The Crazyflie lands when leaving this "with" section
        # When leaving this "with" section, the connection is automatically closed
