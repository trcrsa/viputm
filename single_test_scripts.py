from UAM import UAM
import time

"""
This script tests some corner cases in the UAM and makes sure they have the right behavior.
It also demonstrates flying without pointing in the direction of movement and faster flying.

"""
if __name__ == "__main__":
    aircraft_name = "Drone02"

    with UAM(aircraft_name) as uam:
        uam.take_off()
        uam.stop()
        time.sleep(2)
        uam.land()
        time.sleep(1)
        uam.set_waypoints([(1, 1, 1, 1, 0)])  # Will move at 1m/s and not rotate
        uam.start()
        uam.block_while_flying()
        uam.set_waypoints([(2, 2, 1.3, 1, 20)], None)  # Rotate slowly
        uam.start()
        uam.block_while_flying()
        uam.stop()
        uam.land(override_stop=False)  # Should not land
        time.sleep(10)
        uam.land()
