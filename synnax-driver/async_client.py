import asyncio
from prompt_toolkit import PromptSession
from prompt_toolkit.patch_stdout import patch_stdout
from prompt_toolkit import print_formatted_text, HTML
import concurrent.futures
import struct
import math

CMD_PORT = 15397
TELEM_PORT = 15509

PICO_IP = "192.168.1.20"

class UDPClientProtocol:
    def __init__(self):
        pass

    def connection_made(self, transport: asyncio.DatagramTransport):
        print("Connection made")
        self.transport = transport

    def datagram_received(self, data, addr):
        x = struct.unpack('<f', data)[0]
        as_deg = math.degrees(x)
        print(f"Data: {as_deg}")

    def error_received(self, exc):
        print(f"Error received: {exc}")

    def connection_lost(self, exc):
        print("Connection closed")

async def input_prompt(telem_transport: asyncio.DatagramTransport):
    session = PromptSession()

    try:
        while True:
            try:
                user_input = await session.prompt_async("Enter command > ", set_exception_handler=False)
                try_int = int(user_input)
                telem_transport.sendto(bytes([try_int]))
                print_formatted_text(f"Sent {try_int}")
            except (KeyboardInterrupt, EOFError) as e:
                return
    except asyncio.CancelledError:
        pass

async def watchdog_receiver(cmd_reader: asyncio.streams.StreamReader):
    while True:
        try:
            async with asyncio.timeout(2.0):
                MAGIC = b'\xa4'
                
                value = await cmd_reader.read(1)

                if value != MAGIC:
                    print_formatted_text("Disconnected")
                    return
                continue
            return
        except ConnectionAbortedError:
            print_formatted_text("Connection lost")
            return
        except TimeoutError:
            print_formatted_text("Timed out")
            return

        

async def main():
    while True:
        print_formatted_text(HTML("<ansiblue>Looking for Pico server to connect to</ansiblue>"))
        while True:
            try:
                async with asyncio.timeout(0.5):
                    cmd_reader, cmd_writer = await asyncio.open_connection(PICO_IP, CMD_PORT)
                break
            except TimeoutError:
                continue
            except asyncio.CancelledError:
                return

        print_formatted_text(HTML("<ansigreen>Connected!</ansigreen>"))
        try:
            with patch_stdout():
                print_formatted_text("Starting await")

                loop = asyncio.get_running_loop()

                telem_transport, telem_protocol = await loop.create_datagram_endpoint(
                    lambda: UDPClientProtocol(), local_addr=('0.0.0.0', TELEM_PORT), remote_addr=(PICO_IP, TELEM_PORT))

                watchdog_task = asyncio.create_task(watchdog_receiver(cmd_reader), name = "watchdog")
                input_task = asyncio.create_task(input_prompt(telem_transport), name = "input-task")

                await asyncio.wait([watchdog_task, input_task],
                            return_when = concurrent.futures.FIRST_COMPLETED)
                
                
                if watchdog_task.done():
                    # Must have lost connection
                    input_task.cancel()
                    telem_transport.close()

                if input_task.done():
                    # Must have hit ctrl+C
                    return
                


        except TypeError as e:
            print_formatted_text(f"Exception in main loop: {e}")
        finally:
            print_formatted_text("Closing TCP Connection")
            cmd_writer.close()
            try:
                await cmd_writer.wait_closed()
            except ConnectionAbortedError:
                pass

if __name__ == '__main__':
    asyncio.run(main())
    print_formatted_text("Ctrl+C Detected. Shutting down...")
