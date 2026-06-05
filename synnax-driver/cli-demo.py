import socket
import time
from prompt_toolkit import PromptSession
from prompt_toolkit.patch_stdout import patch_stdout
from rich import print
import asyncio

CMD_PORT = 15397
TELEM_PORT = 15509

PICO_IP = "192.168.1.20"

def udp_thread(stop_flag: threading.Event):
    print("Starting worker UDP thread")

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_sock:
        while not stop_flag.is_set():
            print("Sending UDP data")
            dat = 43
            udp_sock.sendto(dat.to_bytes(), (PICO_IP, TELEM_PORT))
            time.sleep(0.1)

def udp_thread_2(stop_flag: threading.Event):
    print("Starting secondary worker thread")

    while not stop_flag.is_set():
        test_input = input("Enter input: ")
        print("Echo: " + test_input)



try:
    while True:
        udp_stop_flag = threading.Event()
        udp_thread = threading.Thread(target=udp_thread, args=(udp_stop_flag,))
        udp_thread_2 = threading.Thread(target=udp_thread_2, args = (udp_stop_flag,))

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as tcp_sock:
                tcp_sock.settimeout(0.5)
                tcp_sock.connect((PICO_IP, CMD_PORT))

                print("[bold cyan]Connected")

                udp_thread.start()
                udp_thread_2.start()
                while True:
                    value = tcp_sock.recv(1)
                    if not value == b'\xa4':
                        raise Exception()

        except TimeoutError:
            print("[bold yellow]Timeout, trying again...")
        except ConnectionResetError:
            print("[bold red]Connection reset unexpectedly, reconnecting...")
        except Exception:
            print("[bold red]Wrong magic number!")
        finally:
            udp_stop_flag.set()
        
except KeyboardInterrupt:
    print("Exiting")


async def main():
    pass
    asyncio.open_connection

    


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Ctrl+C Detected. Shutting down...")