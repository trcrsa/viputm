from UAM import UAM
import time

"""
This is a simple example to demonstate how stopping and resuming works with the UAM.
After getting in position (the road near the office), the drone will travel towards it's
desination, then after 8 seconds, it will stop and wait for 5 seconds, hovering. Then,
it will resume it's journey and land near the apartment building.

Since uam.start() returns immediately, the time.sleep(8) will wait 8 seconds after begining
to move. The resume function has the drone continue on it's previous path. You might notice
that this script is not deterministic, this is because the stopping takes place based on
time, so it will not always stop in the same place.
"""

if __name__ == "__main__":
    aircraft_name = "CineWhoop"

    with UAM(aircraft_name) as uam:
        # This part is to get the drone in position.
        uam.set_waypoints(
            [(2.275, 4.065, 1)],  # Over the road by office
            None,  # Don't land, just hover after finishing
        )
        uam.start()
        uam.block_while_flying()

        # This is the key part of the example, the stopping and resuming
        uam.set_waypoints([(2.275, 0.5)])  # Road near apartments
        uam.start()
        time.sleep(8)  # After traveling for 8 seconds, stop

        uam.stop()
        time.sleep(5)  # Stop for 5 seconds
        uam.resume()  # Then resume journey
