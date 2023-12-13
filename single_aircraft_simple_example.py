from UAM import UAM
import time

"""
This is the most simple example script. It will take off, go to the office building land.

See the Crazyflie ICD or read the Docstrings for more details on the specifics of UAM methods.

This example shows the basic usage of the class. You will first set the waypoints, and then tell the
aircraft to begin following those waypoints. You can have the aircraft land on an elevated surface by
giving a landing_height_at_end value to set_waypoints.
"""


# If you use config (which is generally easier since you only have to set it up once)
# then you may have to change the file. Details can be found on the UAM ICD on the wiki

use_config = True  # This simply flags if you want to use config or enter the information manually
# If you set this to False, the second part of this example will run and you will have to modify it.
# Also you will have to change the other scripts accordingly

aircraft_name = "Drone02"  # This is the name of the drone, it should be a Vicon object and an entry in config.py


if use_config and __name__ == "__main__":
    # When entering the with statement, the UAM will connect to the Crazyflie
    with UAM(aircraft_name) as uam:
        uam.set_waypoints(
            [(3.340, 4.325, 0.815 + 0.5)],  # 0.5m above the office building
            0.815,  # Land on the office building, at height 0.815m
        )
        uam.start()
    # When exiting the with statement, the Crazyflie will finish
    # following it's path, land, and then disconnect

# Alternatively, instead of using config to automatically get URI and number of cells, you can specify that info yourself

if not use_config and __name__ == "__main__":
    URI = "radio://0/80/2M/E7E7E7E7E7"  # URI, this is the default one
    num_cells = 1  # The number of cells of the battery

    with UAM(aircraft_name, (URI, num_cells)) as uam:
        uam.set_waypoints(
            [(3.340, 4.325, 0.815 + 0.5)],  # 0.5m above the office building
            0.815,  # Land on the office building, at height 0.815m
        )
        uam.start()
    # When exiting the with statement, the Crazyflie will finish
    # following it's path, land, and then disconnect
