import socket
from config import HOST, PORT
import time
import logging
import cflib
from threading import Thread
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
import time
import sys


def handle_response(response):
    permission, value = response.split(": ")
    if permission != "Request granted":
        return None
    return value


class DroneController:
    def __init__(self, radio):
        self.flying = False
        self.connected = False
        uri = uri_helper.uri_from_env(default=radio)
        logging.basicConfig(level=logging.ERROR)
        cflib.crtp.init_drivers()
        self.scf = SyncCrazyflie(uri)
        self.mc = MotionCommander(self.scf)

    def connect(self):
        if not self.connected:
            self.scf.open_link()
            self.connected = True

    def takeoff(self):
        self.connect()
        if not self.flying:
            self.mc.take_off()
            self.flying = True

    def land(self):
        if self.flying:
            self.flying = False
            self.mc.land()

    def disconnect(self):
        self.land()
        if self.connected:
            self.connected = False
            self.scf.close_link()

    def handle_commands(self, command):
        if not self.connected or not self.flying:
            match command:
                case "connect":
                    self.connect()
                case "takeoff":
                    self.takeoff()
                case _:
                    pass
            return

        match command:
            case "land":
                self.flying = False
                self.mc.land()
                return
            case "disconnect":
                self.disconnect()
                return
            case _:
                pass

        dir, value = command.split()
        value = float(value)
        match dir:
            case "forward":
                self.mc.forward(value)
            case "back":
                self.mc.back(value)
            case "left":
                self.mc.left(value)
            case "right":
                self.mc.right(value)
            case "down":
                self.mc.down(value)
            case "up":
                self.mc.up(value)


def run_client():
    try:
        uri = f"radio://{sys.argv[1]}/80/2M/E7E7E7E7{sys.argv[2]}"
        assert len(sys.argv[2]) == 2
    except Exception:
        print(
            "Improper arguments, requires specifying USB # and last two digits of address (00 to FF)"
        )
        exit()
    print(f'Connected to server! Using uri "{uri}"')
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    max_connection_attempts = 10
    for connection_attempt in range(1, max_connection_attempts + 1):
        try:
            client_socket.connect((HOST, PORT))
            break
        except Exception as e:
            print(
                f'"{e}" on connection attempt {connection_attempt}/{max_connection_attempts}'
            )
            time.sleep(1.5)
            if connection_attempt == max_connection_attempts:
                raise Exception(f"Connection Failed {max_connection_attempts} times")

    dc = DroneController(uri)
    while True:
        message = input(
            "Enter a command (type 'exit' to disconnect from server): "
        ).strip()
        client_socket.send(message.encode("utf-8"))
        if message == "exit":
            dc.disconnect()
            break
        response = client_socket.recv(1024).decode("utf-8")
        print(f"got response: {response}")
        command = handle_response(response)
        if command is not None:
            dc.handle_commands(command)

    client_socket.close()


if __name__ == "__main__":
    run_client()
    # thread = Thread(target=run_client)
    # thread.start()
