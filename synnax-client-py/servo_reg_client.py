import asyncio
from prompt_toolkit import PromptSession
from prompt_toolkit.patch_stdout import patch_stdout
from prompt_toolkit import print_formatted_text, HTML
import concurrent.futures
import struct
import math
import synnax as sy
import synnax.framer.streamer as sy_str
import servo_reg_com

CMD_PORT = 15397
TELEM_PORT = 15509

PICO_IP = "192.168.1.20"

class UDPClientProtocol(asyncio.BaseProtocol):
    def __init__(self, synnax_writer: sy.Writer, time_ch: sy.Channel, motor_angle_ch: sy.Channel):
        self.synnax_writer = synnax_writer
        self.time_ch = time_ch
        self.motor_angle_ch = motor_angle_ch

    def connection_made(self, transport: asyncio.DatagramTransport):
        print("Connection made")
        self.transport = transport

    def datagram_received(self, data: bytes, addr):
        # x = struct.unpack('<f', data)[0]
        (value, num_bytes) = servo_reg_com.deserialize(servo_reg_com.TelemToPC, data)
        
        if isinstance(value, servo_reg_com.TelemToPC_MotorPosition):
            as_deg = math.degrees(value[0])
            print(f"Data: {as_deg}")
            # self.synnax_writer.write(channels_or_data=[self.time_ch.key, self.motor_angle_ch.key], series=[[sy.TimeStamp.now()], [as_deg]])
            # self.synnax_writer.commit()

    def error_received(self, exc):
        print(f"Error received: {exc}")

    def connection_lost(self, exc):
        print("Connection closed")

# Old one that prompts on console
# async def input_prompt(telem_transport: asyncio.DatagramTransport, synnax_streamer: sy_str.AsyncStreamer, motor_angle_ch: sy.Channel):
#     session = PromptSession()

#     try:
#         while True:
#             try:
#                 user_input = await session.prompt_async("Enter command > ", set_exception_handler=False)
#                 try_float = math.radians(float(user_input))
#                 telem_transport.sendto(struct.pack('<f', try_float))
#                 print_formatted_text(f"Sent {try_float}")
#             except (KeyboardInterrupt, EOFError) as e:
#                 return
#     except asyncio.CancelledError:
#         pass

async def input_prompt(telem_transport: asyncio.DatagramTransport, synnax_streamer: sy_str.AsyncStreamer, motor_cmd_ch: sy.Channel, time_ch: sy.Channel):
    try:
        async for frame in synnax_streamer:
            # print(frame)
            if motor_cmd_ch.key in frame:
                print("here")
                time_values = frame[time_ch.key]
                motor_cmd_values = frame[motor_cmd_ch.key]

                for value in motor_cmd_values:
                    if isinstance(value, float):
                        try_float = math.radians(float(value))
                        packet = servo_reg_com.serialize(servo_reg_com.TelemFromPC_MotorPositionSetpoint(try_float))
                        telem_transport.sendto(packet)
                        print_formatted_text(f"Sent {try_float}")
    except asyncio.CancelledError:
        pass

async def heartbeat_sender(cmd_writer: asyncio.streams.StreamWriter):
    heartbeat_packet = servo_reg_com.serialize(servo_reg_com.CmdFromPC_Heartbeat())

    try:
        while True:
            try:
                await asyncio.sleep(0.1)

                cmd_writer.write(heartbeat_packet)
                await cmd_writer.drain()
                
                # async with asyncio.timeout(2.0):
                #     MAGIC = b'\xa4'
                    
                #     value = await cmd_reader.read(1)

                #     if value != MAGIC:
                #         print_formatted_text("Disconnected")
                #         return
                #     continue
                # return
            except ConnectionAbortedError:
                print_formatted_text("Connection lost")
                return
            except TimeoutError:
                print_formatted_text("Timed out")
                return
    except asyncio.CancelledError:
        pass

async def main():
    # CLIENT SIDE CODE
    client = sy.Synnax(
        host="localhost",
        port=9090,
        username="synnax",
        password="seldon",
        secure=False,
    )

    # Index Channel
    time_ch = client.channels.create(
        name = "motor_timestamp",
        data_type = sy.DataType.TIMESTAMP,
        is_index = True,
        retrieve_if_name_exists = True
    )

    # Data Channel
    motor_angle_ch = client.channels.create(
        name = "motor_angle",
        data_type = sy.DataType.FLOAT32,
        retrieve_if_name_exists = True,
        index = time_ch.key
    )

    motor_cmd_ch = client.channels.create(
        name = "motor_cmd",
        data_type = sy.DataType.FLOAT32,
        retrieve_if_name_exists=True,
        index = time_ch.key,
    )

    # Clear all data from channels
    # ts_del_start = sy.TimeRange(
    #     start = sy.TimeStamp.MIN,
    #     end = sy.TimeStamp.MAX
    # )

    # client.delete(
    #     [time_ch.key, motor_angle_ch.key],
    #     ts_del_start
    # )

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
                with client.open_writer(start = sy.TimeStamp.now(), channels = [time_ch.key, motor_angle_ch.key]) as synnax_writer:
                    synnax_streamer = await client.open_async_streamer(channels=[time_ch.key, motor_cmd_ch.key])

                    print_formatted_text("Starting await")

                    loop = asyncio.get_running_loop()

                    telem_transport, telem_protocol = await loop.create_datagram_endpoint(
                        lambda: UDPClientProtocol(synnax_writer, time_ch, motor_angle_ch), local_addr=('0.0.0.0', TELEM_PORT), remote_addr=(PICO_IP, TELEM_PORT))

                    watchdog_task = asyncio.create_task(heartbeat_sender(cmd_writer), name = "watchdog")
                    input_task = asyncio.create_task(input_prompt(telem_transport, synnax_streamer, motor_cmd_ch, time_ch), name = "input-task")

                    await asyncio.wait([watchdog_task, input_task],
                                return_when = concurrent.futures.FIRST_COMPLETED)
                    
                    
                    if watchdog_task.done():
                        # Must have lost connection
                        input_task.cancel()
                        telem_transport.close()
                        synnax_writer.close()

                    if input_task.done():
                        # Must have hit ctrl+C
                        return



        except TypeError as e:
            print_formatted_text(f"Exception in main loop: {e}")
        finally:
            print_formatted_text("Closing TCP Connection")
            telem_transport.close()
            cmd_writer.close()
            synnax_writer.close()
            client.close()
            # try:
            #     await cmd_writer.wait_closed()
            # except ConnectionAbortedError:
            #     pass

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    print_formatted_text("Ctrl+C Detected. Shutting down...")
