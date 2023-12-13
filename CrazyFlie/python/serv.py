import socket
from config import *

def handle_command(dir, value):
        dir = dir.lower()
        match dir:
            case "forward":
                mc.forward(value)
            case "back":
                mc.back(value)
            case "left":
                mc.left(value)
            case "right":
                mc.right(value)
            case "down":
                mc.down(value)
            case "up":
                mc.up(value)
            case "land":
                return
            case _:
                raise "command not found"


def handle_client_connection(client_socket):
    while True:
        request = client_socket.recv(1024).decode('utf-8')
        if request == "land":
            client_socket.send("Server is closing".encode('utf-8'))
            return
        messages = request.split()
        try:
            handle_command(messages[0], float(messages[1]))
            response = "Done performing: " + request
        except Exception:
            response = "Invalid command: " + request
        client_socket.send(response.encode('utf-8'))

def run_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    print(f"Server listening on {HOST}:{PORT}")

    client_socket, addr = server_socket.accept()
    print(f"Client connected: {addr}")
    handle_client_connection(client_socket)

    server_socket.close()


import logging
import cflib
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
import time 
import socketserver


if __name__ == '__main__':
    uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

    logging.basicConfig(level=logging.ERROR)

    cflib.crtp.init_drivers()
    with SyncCrazyflie(uri) as scf:
        with MotionCommander(scf) as mc:
            run_server()
