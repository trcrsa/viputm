from UAM import UAM
import time
from config import POINTS_OF_INTEREST

"""
In this example script, a Crazyflie will take off, fly to the center of the traffic circle,
land on the Office building for 1 second. Fly over the factory, land on the Hospital for a
while and then fly over the apartment building to the parking lot before landing on the
center helipad.

This example shows how there are multiple ways to "wait" for the aircraft to finish following the path
You can, call block_while_flying or disconnect or let the with statement end (which also just calls disconnect).
Obviously, disconnect will disconnect after following the path so it's not always what you want.
"""


if __name__ == "__main__":
    aircraft_name = "Drone02"

    # When entering the with statement, the UAM will connect to the Crazyflie
    with UAM(aircraft_name) as uam:
        uam.set_waypoints(
            [
                (2.28, 2.79, 1),  # Traffic circle
                (3.340, 4.325, 0.815 + 0.5),  # Office building
            ],
            0.815,  # Land on Office building
        )
        uam.start()
        uam.block_while_flying()
        time.sleep(1)

        uam.take_off(0.5)
        uam.set_waypoints(
            [
                (5.115, 2.475, 1.350 + 0.3),  # Factory
                (4.885, 1.200),  # Hospital
            ],
            1.35,
        )
        uam.start()

        uam.disconnect()  # Land on Hospital and disconnect
        # uam.block_while_flying()

        uam.connect()  # Reconnect, this will take a bit
        uam.take_off(0.3)

        uam.set_waypoints(
            [
                (1.3, 1, 1.5),  # Parking lot
                (1.055, 2.790, 0.5),  # Center landing pad
            ]
        )
        uam.start()
    # When exiting the with statement, the Crazyflie will finish
    # following it's path, land, and then disconnect
