import socket
from config import HOST, PORT
import threading
import sys
import time

TIMEOUT = 5
timer = None
clients = []
server_socket = None


def handle_command(request):
    valid_commands = ["takeoff", "land", "connect", "disconnect"]
    valid_dirs = [
        "forward",
        "back",
        "left",
        "right",
        "up",
        "down",
    ]
    messages = request.split()
    try:
        dir, value = messages[0], float(messages[1])
        dir = dir.lower()
        if dir not in valid_dirs:
            return f"Invalid Command: {request}"
        return f"Request granted: {dir} {value}"
    except Exception:
        command = request.lower().strip()
        if command not in valid_commands:
            return f"Invalid Command: {request}"
        return f"Request granted: {command}"


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
    client_socket.close()
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


if __name__ == "__main__":
    run_server()
