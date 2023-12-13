HOST, PORT = "localhost", 9999

# values are in meters
POINTS_OF_INTEREST = {
    # name: (x, y, z, free height above)
    "helibase": (0.850, 3.170, 0, 0.6),
    "heli1": (0.850, 3.170, 0, 0.6),
    "heli2": (1.260, 3.170, 0, 0.6),
    "heli3": (1.055, 2.790, 0, 0.6),
    "heli4": (0.855, 2.400, 0, 0.6),
    "heli5": (1.260, 2.400, 0, 0.6),
    "hospital": (4.885, 1.200, 1.350, 0.3),
    "factory_low": (5.115, 3.080, 0.530, 0.5),
    "factory_high": (5.115, 2.475, 0.780, 0.5),
    "office": (3.340, 4.325, 0.815, 0.5),
    "office_lp1": (2.890, 4.694, 0.815, 0.5),
    "office_lp2": (3.692, 4.749, 0.815, 0.5),
    "office_lp3": (3.7267, 3.916, 0.815, 0.5),
    "office_lp4": (2.904, 4.743, 0.815, 0.5),
    "apartment": (3.470, 1.075, 1.340, 0.3),
}

DRONE_LIST = {
    # Name: (address id  channel  cells)
    "Bolt": ("E7", 80, 2),
    "Drone02": ("02", 80, 1),
    "Drone04": ("04", 82, 1),
    "CineWhoop": ("E7", 80, 4),
    "Drone00": ("00", 80, 1),
    "Drone01": ("01", 80, 1),
    "Drone03": ("03", 80, 1)
}

DEFAULT_DRONE1 = 'Drone02'
DEFAULT_DRONE2 = 'Drone04'

TAKE_OFF_HEIGHT = 0.3
