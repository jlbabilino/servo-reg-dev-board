import socket
import time
import struct

CMD_PORT = 15397
TELEM_PORT = 15509

PICO_IP = "192.168.1.20"

try:
    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(0.5)
                s.connect((PICO_IP, CMD_PORT))

                print("Connected")
                while True:
                    value = s.recv(1)
                    if not value == b'\xa4':
                        raise Exception()

        except TimeoutError:
            print("Timeout, trying again")
        except ConnectionResetError:
            print("Connection reset unexpectedly")
        except Exception:
            print("Wrong magic number!")
        
except KeyboardInterrupt:
    print("Exiting")