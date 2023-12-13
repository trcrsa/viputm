import threading
import numpy as np
import math

class UTM:
    def __init__(self):
        self.operators = []  # utm.operators must be populated with all the operators before running UTM functions
        self.confirmed_missions = []
        self.ongoing_missions = []

        self.reserved = []

        self.landing_groups = {'office': ('office_lp1', 'office_lp2'),
                               'helibase': ('heli2', 'heli3', 'heli4', 'heli5')}

        self.minimum_separation = 0.3  # if two drones get closer than this (meters), it is considered a conflict
        self.grace_period = 3  # number of seconds after a drone has exited a landing spot before it is considered clear

    def resolve_request(self, request):
        """ This method should look at the requested mission, all the other other confirmed missions, and edit
        the request to ensure no conflicts will happen between it and any other confirmed missions.
        Then, it will return the edited request and append it to self.confirmed_missions to "confirm" it."""

        # this block ensures no two missions will land at the same spot during conflicting times
        destinationNum = 0
        for destination in request.itinerary.keys():

            if self.is_reserved(destination):

                try:
                    possible_alternate_locations = self.landing_groups[destination]
                except KeyError:  # case where desired destination has no valid alternate locations
                    return False, destination

                for index, PAL in enumerate(possible_alternate_locations):
                    if not self.is_reserved(PAL):
                        request.preflight_edit(destinationNum, PAL)
                        break
                    if index == len(possible_alternate_locations):
                        return False, destination

            destinationNum += 1

        self.confirmed_missions.append(request)
        return request

    def is_reserved(self, landing_spot):
        """ Returns False if the landing spot is OK to land at.
            Returns True if the aircraft must land at a different location.
            :param landing_spot: a string name (eg. 'office') of a landing spot. """

        if landing_spot not in self.reserved:
            self.reserved.append(landing_spot)
            return False

        return True

    def find_conflict_region(self, m1, m2):
        """ Returns (t0, t1, [x0, y0]): the earliest time interval where there is a conflict between the two missions
        and the position of mission1 at the start of that conflict. Returns None if there is no conflict.
        If they are too close on the x-y plane, it is considered a conflict no matter the z distance between them. """

        last_start = max(m1.start_time, m2.start_time)
        first_end = min(m1.end_time, m2.end_time)
        if first_end - last_start < 0:
            return  # cannot have conflicts if the missions are never in the air at the same time

        dt = 1 / m1.resolution
        times = np.round(np.arange(last_start, first_end + dt, dt), 2)
        conflict_pos = [0, 0]
        conflict_time = []

        for t in times:
            pos1 = m1.map[np.where(m1.map == t)[0][0]][1:-1]
            pos2 = m2.map[np.where(m2.map == t)[0][0]][1:-1]

            if math.dist(pos1, pos2) < self.minimum_separation and len(conflict_time) == 0:
                conflict_time.append(t)
                conflict_pos = pos1
            if math.dist(pos1, pos2) > self.minimum_separation and len(conflict_time) != 0:
                conflict_time.append(t)
                return conflict_time[0], conflict_time[1], conflict_pos

        if len(conflict_time) != 0:  # case where missions have the same final destination at the same time
            conflict_time.append(t)
            return conflict_time[0], conflict_time[1], conflict_pos
        
    def fly_all(self):
        """ uses threads to fly all the operators' missions at their start times. """

        operator_threads = []
        for operator in self.operators:
            if len(operator.missions) != 0:
                op_thread = threading.Timer(operator.missions[0].start_time, operator.fly)
                operator_threads.append(op_thread)

        for op_thread in operator_threads:
            op_thread.start()

"""
import socket

from config import HOST, PORT, POINTS_OF_INTEREST
import threading
import sys
import time

TIMEOUT = 5
timer = None
clients = []
server_socket = None

def decode_waypoints(waypoints):
    return [
        tuple([float(value) for value in point.split(",")])
        for point in waypoints.split()
    ]


def encode_waypoints(waypoints):
    return " ".join([",".join(map(str, point)) for point in waypoints])


def handle_command(request):
    try:
        tuples = decode_waypoints(request)
        assert all([len(t) == 4 for t in tuples])
    except Exception:
        return "Malformed Request"


def handle_client_connection(client_socket, address):
    reset_timer()
    clients.append(client_socket)

    while True:
        request = client_socket.recv(1024).decode("utf-8")
        if request == "exit":
            client_socket.send("Leaving server".encode("utf-8"))
            break
        response = handle_command(request)
        try:
            client_socket.send(response.encode("utf-8"))
        except BrokenPipeError:
            break

    print(f"Client {address[0]}:{address[1]} disconnected")
    client    def __init__(
        self, aircraft_name, aircraft_info=DEFAULT, min_volts_per_cell=3.6, *, kwargs={}
    ):_socket.close()
    clients.remove(client_socket)
    if len(clients) == 0:
        start_timer()


def start_timer():
    global timer
    timer = threading.Timer(TIMEOUT, shutdown_server)
    timer.start()


def reset_timer():
    global timer
    if timer:
        timer.cancel()


def shutdown_server():
    print(f"No clients joined within {TIMEOUT}s. Shutting down the server.")
    if server_socket:
        server_socket.shutdown(socket.SHUT_RDWR)
        server_socket.close()
    exit()


def run_server():
    global server_socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    print(f"Server listening on {HOST}:{PORT}")

    while True:
        try:
            client, address = server_socket.accept()
        except:
            break
        print(f"Connected to: {address[0]}:{address[1]}")
        t = threading.Thread(target=handle_client_connection, args=(client, address))
        t.start()

if __name__ == '__main__':
    run_server()"""