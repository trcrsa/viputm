import socket
from config import *
import time

def run_client():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    max_connection_attempts = 10
    for connection_attempt in range(max_connection_attempts):
        try:
            client_socket.connect((HOST, PORT))
            break
        except Exception as e:
            print(f'"{e}" on connection attempt {connection_attempt+1}/{max_connection_attempts}')
            time.sleep(1.5)
            if connection_attempt == max_connection_attempts-1:
                raise Exception(f"Connection Failed {max_connection_attempts} times")

    print("Connected!")
    while True:
        message = input("Enter a command (type 'land' to land): ")
        client_socket.send(message.encode('utf-8'))
        if message == "land":
            break
        response = client_socket.recv(1024).decode('utf-8')
        print(response)

    client_socket.close()

if __name__ == '__main__':
    run_client()
