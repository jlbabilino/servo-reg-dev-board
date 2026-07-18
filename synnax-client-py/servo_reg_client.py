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
import numpy as np

CMD_PORT = 15397
TELEM_PORT = 15509

PICO_IP = "192.168.1.20"


class UDPClientProtocol(asyncio.BaseProtocol):
    def __init__(
        self, synnax_writer: sy.Writer, time_ch: sy.Channel, motor_angle_ch: sy.Channel
    ):
        self.synnax_writer = synnax_writer
        self.time_ch = time_ch
        self.motor_angle_ch = motor_angle_ch

    def connection_made(self, transport: asyncio.DatagramTransport):
        print("Connection made")
        self.transport = transport

    def datagram_received(self, data: bytes, addr: tuple[str, int]):

        value, num_bytes = servo_reg_com.deserialize(servo_reg_com.TelemToPC, data)

        if isinstance(value, servo_reg_com.TelemToPC_MotorPosition):
            val_rad = value[0]
            # print(f"Data: {as_deg}")
            self.synnax_writer.write(
                channels_or_data=[self.time_ch.key, self.motor_angle_ch.key],
                series=[[sy.TimeStamp.now()], [val_rad]],
            )
            self.synnax_writer.commit()

    def error_received(self, exc):
        print(f"Error received: {exc}")

    def connection_lost(self, exc):
        print("Connection closed")


# async def console_input_prompt(telem_transport: asyncio.DatagramTransport, motor_angle_ch: sy.Channel):
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


async def output_to_synnax(
    cmd_tcp_reader: asyncio.StreamReader,
    motor_state_ch: sy.Channel,
    time_ch: sy.Channel,
    synnax_writer: sy.Writer,
):
    while True:
        data = await cmd_tcp_reader.read(128)
        response, num_bytes = servo_reg_com.deserialize(
            servo_reg_com.ResponseToPC, data
        )

        if isinstance(response, servo_reg_com.ResponseToPC_Disabled):
            synnax_writer.write(
                channels_or_data=[time_ch.key, motor_state_ch.key],
                series=[[sy.TimeStamp.now()], [0]],
            )
            synnax_writer.commit()
        elif isinstance(response, servo_reg_com.ResponseToPC_EnabledPositionControl):
            synnax_writer.write(
                channels_or_data=[time_ch.key, motor_state_ch.key],
                series=[[sy.TimeStamp.now()], [1]],
            )
            synnax_writer.commit()
        elif isinstance(response, servo_reg_com.ResponseToPC_EnabledSpeedControl):
            synnax_writer.write(
                channels_or_data=[time_ch.key, motor_state_ch.key],
                series=[[sy.TimeStamp.now()], [2]],
            )
            synnax_writer.commit()


# Takes values from Synnax channels and sends them to the Pico over ethernet
async def input_cmd_from_synnax(
    cmd_tcp_writer: asyncio.StreamWriter,
    synnax_streamer_cmd: sy_str.AsyncStreamer,
    motor_cmd_ch: sy.Channel,
):
    try:
        async for frame in synnax_streamer_cmd:

            motor_cmd_values = frame[motor_cmd_ch.key]

            for value in motor_cmd_values:
                if isinstance(value, np.number):
                    match value:
                        case 0:
                            packet = servo_reg_com.serialize(
                                servo_reg_com.CmdFromPC_Disable()
                            )
                            cmd_tcp_writer.write(packet)
                            await cmd_tcp_writer.drain()
                        case 1:
                            packet = servo_reg_com.serialize(
                                servo_reg_com.CmdFromPC_EnablePositionControl()
                            )
                            cmd_tcp_writer.write(packet)
                            await cmd_tcp_writer.drain()
                        case 2:
                            packet = servo_reg_com.serialize(
                                servo_reg_com.CmdFromPC_EnableSpeedControl()
                            )
                            cmd_tcp_writer.write(packet)
                            await cmd_tcp_writer.drain()

    except asyncio.CancelledError:
        pass


async def input_speed_setpoint_from_synnax(
    telem_transport: asyncio.DatagramTransport,
    synnax_streamer: sy_str.AsyncStreamer,
    motor_speed_setpoint_ch: sy.Channel,
):
    try:
        async for frame in synnax_streamer:

            motor_speed_setpoint_values = frame[motor_speed_setpoint_ch.key]

            for value in motor_speed_setpoint_values:
                if isinstance(value, np.number):
                    motor_speed = value
                    packet = servo_reg_com.serialize(
                        servo_reg_com.TelemFromPC_MotorSpeedSetpoint(float(motor_speed))
                    )
                    telem_transport.sendto(packet)

    except asyncio.CancelledError:
        pass

async def input_position_setpoint_from_synnax(
    telem_transport: asyncio.DatagramTransport,
    synnax_streamer: sy_str.AsyncStreamer,
    motor_position_setpoint_ch: sy.Channel,
):
    try:
        async for frame in synnax_streamer:

            motor_position_setpoint_values = frame[motor_position_setpoint_ch.key]

            for value in motor_position_setpoint_values:
                if isinstance(value, np.number):
                    motor_position = value
                    packet = servo_reg_com.serialize(
                        servo_reg_com.TelemFromPC_MotorPositionSetpoint(float(motor_position))
                    )
                    telem_transport.sendto(packet)

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

            except ConnectionAbortedError:
                print_formatted_text("Connection lost")
                return
            except TimeoutError:
                print_formatted_text("Timed out")
                return
    except asyncio.CancelledError:
        pass


async def main():

    client = sy.Synnax(
        host="localhost",
        port=9090,
        username="synnax",
        password="seldon",
        secure=False,
    )

    motor_telem_ts_ch = client.channels.create(
        name="motor_telem_ts",
        data_type=sy.DataType.TIMESTAMP,
        is_index=True,
        retrieve_if_name_exists=True,
    )
    motor_position_encoder_ch = client.channels.create(
        name="motor_position_encoder",
        data_type=sy.DataType.FLOAT32,
        retrieve_if_name_exists=True,
        index=motor_telem_ts_ch.key,
    )

    motor_position_setpoint_ts_ch = client.channels.create(
        name="motor_position_setpoint_ts",
        data_type=sy.DataType.TIMESTAMP,
        is_index=True,
        retrieve_if_name_exists=True,
    )
    motor_position_setpoint_ch = client.channels.create(
        name="motor_position_setpoint",
        data_type=sy.DataType.FLOAT32,
        retrieve_if_name_exists=True,
        index=motor_position_setpoint_ts_ch.key,
    )

    motor_speed_setpoint_ts_ch = client.channels.create(
        name="motor_speed_setpoint_ts",
        data_type=sy.DataType.TIMESTAMP,
        is_index=True,
        retrieve_if_name_exists=True,
    )
    motor_speed_setpoint_ch = client.channels.create(
        name="motor_speed_setpoint",
        data_type=sy.DataType.FLOAT32,
        retrieve_if_name_exists=True,
        index=motor_speed_setpoint_ts_ch.key,
    )

    motor_cmd_ts_ch = client.channels.create(
        name="motor_cmd_ts",
        data_type=sy.DataType.TIMESTAMP,
        is_index=True,
        retrieve_if_name_exists=True,
    )
    motor_cmd_ch = client.channels.create(
        name="motor_cmd",
        data_type=sy.DataType.INT32,
        retrieve_if_name_exists=True,
        index=motor_cmd_ts_ch.key,
    )

    motor_state_ts_ch = client.channels.create(
        name="motor_state_ts",
        data_type=sy.DataType.TIMESTAMP,
        is_index=True,
        retrieve_if_name_exists=True,
    )
    motor_state_ch = client.channels.create(
        name="motor_state",
        data_type=sy.DataType.INT32,
        retrieve_if_name_exists=True,
        index=motor_state_ts_ch.key,
    )

    while True:
        print_formatted_text(
            HTML("<ansiblue>Looking for Pico server to connect to</ansiblue>")
        )
        while True:
            try:
                async with asyncio.timeout(0.5):
                    cmd_tcp_reader, cmd_tcp_writer = await asyncio.open_connection(
                        PICO_IP, CMD_PORT
                    )
                break
            except TimeoutError:
                continue
            except asyncio.CancelledError:
                return

        print_formatted_text(HTML("<ansigreen>Connected!</ansigreen>"))
        try:
            with patch_stdout():
                with (
                    client.open_writer(
                        start=sy.TimeStamp.now(),
                        channels=[motor_telem_ts_ch.key, motor_position_encoder_ch.key],
                    ) as synnax_writer_pos_enc,
                    client.open_writer(
                        start=sy.TimeStamp.now(),
                        channels=[motor_state_ts_ch.key, motor_state_ch.key],
                    ) as synnax_writer_state,
                ):
                    synnax_streamer_cmd = await client.open_async_streamer(
                        channels=[motor_cmd_ts_ch.key, motor_cmd_ch.key]
                    )
                    synnax_streamer_position_setpoint = await client.open_async_streamer(
                        channels=[
                            motor_position_setpoint_ts_ch.key,
                            motor_position_setpoint_ch.key,
                        ]
                    )
                    synnax_streamer_speed_setpoint = await client.open_async_streamer(
                        channels=[
                            motor_speed_setpoint_ts_ch.key,
                            motor_speed_setpoint_ch.key,
                        ]
                    )

                    print_formatted_text("Starting await")

                    loop = asyncio.get_running_loop()

                    telem_transport, telem_protocol = (
                        await loop.create_datagram_endpoint(
                            lambda: UDPClientProtocol(
                                synnax_writer_pos_enc,
                                motor_telem_ts_ch,
                                motor_position_encoder_ch,
                            ),
                            local_addr=("0.0.0.0", TELEM_PORT),
                            remote_addr=(PICO_IP, TELEM_PORT),
                        )
                    )

                    heartbeat_task = asyncio.create_task(
                        heartbeat_sender(cmd_tcp_writer), name="watchdog"
                    )
                    input_cmd_from_synnax_task = asyncio.create_task(
                        input_cmd_from_synnax(
                            cmd_tcp_writer, synnax_streamer_cmd, motor_cmd_ch
                        ),
                        name="input-from_synnax-task",
                    )
                    input_position_setpoint_from_synnax_task = asyncio.create_task(
                        input_position_setpoint_from_synnax(
                            telem_transport,
                            synnax_streamer_position_setpoint,
                            motor_position_setpoint_ch,
                        ),
                        name="input-position-setpoint-from-synnax-task",
                    )
                    input_speed_setpoint_from_synnax_task = asyncio.create_task(
                        input_speed_setpoint_from_synnax(
                            telem_transport,
                            synnax_streamer_speed_setpoint,
                            motor_speed_setpoint_ch,
                        ),
                        name="input-speed-setpoint-from-synnax-task",
                    )
                    output_to_synnax_task = asyncio.create_task(
                        output_to_synnax(
                            cmd_tcp_reader,
                            motor_state_ch,
                            motor_state_ts_ch,
                            synnax_writer_state,
                        ),
                        name="output-to-synnax-task",
                    )

                    await asyncio.wait(
                        [
                            heartbeat_task,
                            input_cmd_from_synnax_task,
                            input_position_setpoint_from_synnax_task,
                            input_speed_setpoint_from_synnax_task,
                            output_to_synnax_task,
                        ],
                        return_when=concurrent.futures.FIRST_COMPLETED,
                    )

                    if heartbeat_task.done():
                        # Must have lost connection
                        input_cmd_from_synnax_task.cancel()
                        input_position_setpoint_from_synnax_task.cancel()
                        input_speed_setpoint_from_synnax_task.cancel()
                        telem_transport.close()
                        synnax_writer_pos_enc.close()
                        synnax_writer_state.close()

                    if (
                        input_cmd_from_synnax_task.done()
                        or input_position_setpoint_from_synnax_task.done()
                        or input_speed_setpoint_from_synnax_task.done()
                    ):
                        # Must have hit ctrl+C
                        return

        except TypeError as e:
            print_formatted_text(f"Exception in main loop: {e}")
        finally:
            print_formatted_text("Closing TCP Connection")
            telem_transport.close()
            cmd_tcp_writer.close()
            synnax_writer_pos_enc.close()
            synnax_writer_state.close()
            client.close()
            # try:
            #     await cmd_writer.wait_closed()
            # except ConnectionAbortedError:
            #     pass


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    print_formatted_text("Ctrl+C Detected. Shutting down...")
