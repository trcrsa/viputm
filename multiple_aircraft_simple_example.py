from UAM import UAM
import socket
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

aircraft_name = ["Drone04", "Drone02"]  # This is the names of the drones, it should be a Vicon object and an entry in config.py

if __name__ == "__main__":

    # When entering the with statement, the UAM will connect to the Crazyflie
    with UAM(aircraft_name[0]) as uam0, UAM(aircraft_name[1]) as uam1, UAM(aircraft_name[2]) as uam2:
    	
        uam0.set_waypoints(
            [
                (2.28, 2.79, 1),  # Traffic circle
                (3.340, 4.325, 0.815 + 0.5),  # Office building
            ],
            0.815,  # Land on Office building
        )
        uam1.set_waypoints(
            [
                (2.28-0.2, 2.79-0.2 , 1.5),  # Traffic circle
             #  (3.340-0.2, 4.325-0.2, 0.815+0.5),  # Office building
             	(4.885, 1.200, 1.7), # Hospital building
            ],
            1.5,  # Land on Office building
        )
        uam2.set_waypoints(
            [
                (2.28+0.2, 2.79 +0.2, 1),  # Traffic circle
                (5.115, 3.080, 0.530),  # Office building
            ],
            0.53,  # Land on Office building
        )
        uam0.start()
        uam1.start()
        uam2.start()
        # uam0.block_while_flying()
        # uam2.block_while_flying()
        # time.sleep(1)

        # uam0.take_off(0.5)
        # uam2.take_off(0.5)
        # uam0.set_waypoints(
        #     [
        #         (5.115, 2.475, 1.350 + 0.3),  # Factory
        #         (4.885, 1.200),  # Hospital
        #     ],
        #     1.35,
        # )
        # uam2.set_waypoints(
        #     [
        #         (5.115, 2.475, 1.350 + 0.3),  # Factory
        #         (4.885, 1.200),  # Hospital
        #     ],
        #     1.35,
        # )
        # uam0.start()
        # uam2.start()

        # uam0.disconnect()  # Land on Hospital and disconnect
        # uam2.disconnect()  # Land on Hospital and disconnect
        # # uam.block_while_flying()

        # uam0.connect()  # Reconnect, this will take a bit
        # uam2.connect()
        # uam0.take_off(0.3)
        # uam2.take_off(0.3)

        # uam0.set_waypoints(
        #     [
        #         (1.3, 1, 1.5),  # Parking lot
        #         (1.260, 2.4051, 0.5),  # landing pad 5
        #     ]
        # )
        # uam2.set_waypoints(
        #     [
        #         (1.3, 1, 1.5),  # Parking lot
        #         (0.84985, 3.1733, 0.03465),  # landing pad 1
        #     ]
        # )
        # uam0.start()
        # uam2.start()
    # When exiting the with statement, the Crazyflie will finish
    # following it's path, land, and then disconnect
