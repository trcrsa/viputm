from UAM import UAM
import time

"""
This is a simple example to demonstate how stopping and resuming works with the UAM.
After getting in position (the road near the office), the drone will travel towards it's
desination, then after 8 seconds, it will stop and wait for 5 seconds, hovering. Then,
it will redirect to visit the office building then land on the factory building.

This is almost identical to resume, except instead of following the original path, it
follows a new arbitrary path.
"""

if __name__ == "__main__":
    aircraft_name = "Drone02"

    with UAM(aircraft_name) as uam:
        # This part is to get the drone in position.
        uam.set_waypoints(
            [(2.275, 4.065, 1)],  # Over the road by office
            None,  # Don't land, just hover after finishing
        )
        uam.start()
        uam.block_while_flying()

        # This is the key part of the example, the stopping and rerouting
        uam.set_waypoints([(2.275, 0.5)])  # Road near apartments
        uam.start()
        time.sleep(8)  # After traveling for 8 seconds, stop

        uam.stop()
        uam.set_waypoints(
            [
                (3.340, 4.325, 1.5),  # Office building
                (5.115, 3.080),  # Factory building
            ],
            0.530,  # Factory building height
        )
        time.sleep(5)  # Stop for 5 seconds
        uam.start()  # Then reroute to new waypoints
