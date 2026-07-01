use core::net::Ipv4Addr;
use core::net::SocketAddrV4;

use embassy_futures::select::Either;
use embassy_futures::select::Either3;
use embassy_futures::select::select;
use embassy_futures::select::select3;
use embassy_rp::gpio;
use embassy_rp::peripherals::SPI1;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_sync::watch;
use embassy_time::Duration;
use embassy_time::Ticker;
use embassy_time::Timer;
use embedded_hal_bus::spi::ExclusiveDevice;
use fixed::traits::ToFixed;
use serde::Deserialize;
use serde::Serialize;
use w5500_hl::Tcp;
use w5500_hl::Udp;
use w5500_ll::Interrupt;
use w5500_ll::SocketCommand;
use w5500_ll::SocketInterrupt;
use w5500_ll::SocketInterruptMask;
use w5500_ll::SocketStatus;
use w5500_ll::eh1::vdm::W5500;
use w5500_ll::net::Eui48Addr;
use w5500_ll::{Registers, Sn};

use crate::constants::HEARTBEAT_MAX_ALLOWED;
use crate::data::MOTOR_CURRENT_POSITION;
use crate::data::MOTOR_POSITION_SETPOINT;
use crate::data::MOTOR_SPEED_SETPOINT;

pub type ExclusiveW5500 = W5500<
    ExclusiveDevice<
        embassy_rp::spi::Spi<'static, SPI1, embassy_rp::spi::Async>,
        gpio::Output<'static>,
        embassy_time::Delay,
    >,
>;

const CMD_SOCKET: Sn = Sn::Sn0;
const CMD_PORT: u16 = 15397;
const TELEM_SOCKET: Sn = Sn::Sn1;
const TELEM_PORT: u16 = 15509;

// Static IPV4 Config
const IP_ADDR: Ipv4Addr = Ipv4Addr::new(192, 168, 1, 20); // Pico's static IP
const PC_ADDR: Ipv4Addr = Ipv4Addr::new(192, 168, 1, 10); // TODO: get this from TCP, don't hardcode it
const GATEWAY: Ipv4Addr = Ipv4Addr::new(192, 168, 1, 1);
const SUBNET: Ipv4Addr = Ipv4Addr::new(255, 255, 255, 0);
const MAC_ADDR: Eui48Addr = Eui48Addr::new(0x02, 0x00, 0x11, 0x22, 0x33, 0x44); // arbitrary

const ENABLED_ANIM: crate::rgb_led::Command =
    crate::rgb_led::Command::Looping(crate::anim::Animation::Pulse(crate::anim::Pulse::new(
        color::palette::css::RED.discard_alpha(),
        embassy_time::Duration::from_millis(100),
        embassy_time::Duration::from_millis(125),
        embassy_time::Duration::from_millis(250),
        embassy_time::Duration::from_millis(400),
        2,
    )));

const DISABLED_ANIM: crate::rgb_led::Command = crate::rgb_led::Command::Looping(
    crate::anim::Animation::FadeInFadeOut(crate::anim::FadeInFadeOut::new(
        color::palette::css::BLUE.discard_alpha(),
        embassy_time::Duration::from_secs(3),
    )),
);

/// Network messages outline:
///
/// Here's how we'll use each hardware socket:
/// Sn0: TCP - CMD - high-level signals like enable/disable, watchdog, etc.
/// Sn1: UDP - TELEM - high-frequency signals like motor's position, commanded position, etc.
///
/// Once CMD is established, there is only one purpose. Watchdog hasn't been implemented yet.
/// It simply waits for a packet with a u8 enum value. This indiciates what mode to put
/// the servo reg in
///
/// The TELEM socket is established immediately after the CMD socket is. Pico will
/// immediately start sending feedback telemetry like the motor's position in radians.
/// It will also try to receive data on defined channels like a commanded motor
/// position.
///

#[derive(Copy, Clone, defmt::Format, Serialize, Deserialize)]
pub enum CmdFromPC {
    Disable,
    Heartbeat,
    PIDSet(f32, f32, f32),
    PIDGet,
    Enable,
}

#[derive(Copy, Clone, defmt::Format, Serialize, Deserialize)]
pub enum TelemToPC {
    MotorAngle(f32),
}

#[derive(Copy, Clone, defmt::Format, Serialize, Deserialize)]
pub enum TelemFromPC {
    MotorPositionSetpoint(f32),
    MotorSpeedSetpoint(f32),
}

struct Heartbeat {}

#[derive(Copy, Clone, defmt::Format)]
pub enum NetworkStatus {
    Disconnected,
    Connected,
}

#[embassy_executor::task]
pub async fn network_task(
    mut w5500: ExclusiveW5500,
    mut w5500_int: gpio::Input<'static>,
    status_ind: watch::Sender<'static, CriticalSectionRawMutex, NetworkStatus, 4>,
    cmd_channel: channel::Sender<'static, CriticalSectionRawMutex, CmdFromPC, 16>,
) {
    // Goal is to make this code have no panic points
    // Must loop until we are able to configure it.
    // Since W5500 module can be physically disconnected from the Pico (it's
    // not soldered yet), it may loop here until you plug it in
    loop {
        match configure_w5500(&mut w5500) {
            Ok(_) => {
                break;
            }
            Err(msg) => {
                defmt::error!(
                    "Failed to do initial configuration of W5500 (is the W5500 module plugged in?): {}",
                    msg
                );
            }
        }
        Timer::after_secs(1).await;
    }

    let w5500_lock = Mutex::<NoopRawMutex, ExclusiveW5500>::new(w5500);

    // Accept connections, transfer data, disconnect, then repeat
    loop {
        match handle_connection(&w5500_lock, &mut w5500_int, &status_ind, &cmd_channel).await {
            Ok(_) => {
                // Must have gracefully disconnected
                defmt::info!("Connection closed");
                Timer::after_secs(1).await;
                continue;
            }
            Err(msg) => {
                defmt::error!("Connection aborted: {}", msg);
                status_ind.send(NetworkStatus::Disconnected);
                Timer::after_secs(1).await;
                continue;
            }
        }
    }
}

async fn handle_connection(
    w5500_mutex: &Mutex<NoopRawMutex, ExclusiveW5500>,
    mut w5500_int: &mut gpio::Input<'static>,
    status_ind: &watch::Sender<'static, CriticalSectionRawMutex, NetworkStatus, 4>,
    cmd_channel: &channel::Sender<'static, CriticalSectionRawMutex, CmdFromPC, 16>,
) -> Result<(), &'static str> {
    // Make sure W5500 starts in a disconnected state. For example, if pico
    // restarted and the w5500 is still connected to something, we close it
    // forcefully

    {
        let mut w5500 = w5500_mutex.lock().await;
        force_close_cmd(&mut w5500).await?;

        // We assume W5500 CMD socket is disconnected/closed here

        // Indicate this closed state initialization
        status_ind.send(NetworkStatus::Disconnected);

        // Get W5500 in a TCP listening state
        defmt::info!("Listening for CMD TCP server on port {}...", CMD_PORT);

        let mut w5500 = w5500_mutex.lock().await;
        w5500
            .tcp_listen(CMD_SOCKET, CMD_PORT)
            .map_err(|_| "Failed to put W5500 in TCP listening mode")?;

        // Set only the interrupts we care about for waiting for a connection

        let mut w5500 = w5500_mutex.lock().await;
        configure_interrupts_pre_con(&mut w5500);
    }

    // Wait for interrupt pin to go low
    w5500_int.wait_for_low().await;

    // Handle all four interrupts enabled previously
    {
        let mut w5500 = w5500_mutex.lock().await;
        if w5500 // 1. IP Conflict
            .ir()
            .map_err(|_| "Failed to get W5500 interrupt status")?
            .conflict()
        {
            w5500
                .set_ir(Interrupt::DEFAULT.set_conflict())
                .map_err(|_| "Failed to clear IP conflict interrupt")?;
            return Err("IP conflict detected");
        }
        if w5500 // 2. Destination host unreachable
            .ir()
            .map_err(|_| "Failed to get W5500 interrupt status")?
            .unreach()
        {
            w5500
                .set_ir(Interrupt::DEFAULT.set_unreach())
                .map_err(|_| "Failed to clear host unreachable interrupt")?;
            return Err("Destination host unreachable");
        }
        let cmd_ir = w5500
            .sn_ir(CMD_SOCKET)
            .map_err(|_| "Failed to get CMD socket interrupt register")?;

        // 3. CMD socket timeout
        if cmd_ir.timeout_raised() {
            w5500
                .set_sn_ir(CMD_SOCKET, SocketInterrupt::TIMEOUT_MASK)
                .map_err(|_| "Failed to clear CMD timeout interrupt")?;
            return Err("CMD socket timed out");
        }
        if !cmd_ir.con_raised() {
            // Should panic here because this could only happen if the code for
            // changing interrupts was written incorrectly
            return Err(
                "Only CON interrupt for CMD socket should be enabled for W5500, check code",
            );
        }

        // Now we know CMD connection must have been raised, so clear it

        let con_clr = SocketInterrupt::DEFAULT.clear_con();
        w5500.set_sn_ir(CMD_SOCKET, con_clr).unwrap();

        // Now we know W5500 is connected to client, so signal that
        defmt::info!("CMD TCP Connected!");
        status_ind.send(NetworkStatus::Connected);

        // Configure interrupts to listen for disconnects and data
        configure_interrupts_active_con(&mut w5500);
    }

    // Set up a mutex since we need to have access to the W5500 in the RX and
    // TX loops simultaneously

    let heartbeat_signal = Signal::<NoopRawMutex, Heartbeat>::new();

    // Race these functions. If any end, we must restart connection
    //   1. pulse check. If this ends, heartbeat stopped being received
    //   2. Receiver loop. If this ends, must have gracefully disconnected
    //   3. Transmission loop. This should never end
    match select3(
        pulse_check(&heartbeat_signal),
        active_con_loop(&w5500_mutex, &mut w5500_int, &heartbeat_signal, cmd_channel),
        push_telemetry(&w5500_mutex),
    )
    .await
    {
        Either3::First(_) => {
            return Err("Didn't get a hearbeat from client in time");
        }
        Either3::Second(Ok(_)) => {
            // This is the "ideal" path, graceful disconnection
            defmt::info!("Client disconnected gracefully");
        }
        Either3::Second(Err(msg)) => {
            // Propagate error
            return Err(msg);
        }
        Either3::Third(Ok(_)) => {
            // strangely, ok is bad in this case because the loop should never end
            return Err("Failed to push telemetry data");
        }
        Either3::Third(Err(msg)) => {
            return Err(msg);
        }
    }

    // Now we know client gracefully disconnected
    status_ind.send(NetworkStatus::Disconnected);
    Ok(())
}

/// Keeps looping until it stops receiving the heartbeat signal in time
async fn pulse_check(heartbeat_signal: &Signal<NoopRawMutex, Heartbeat>) {
    loop {
        match select(heartbeat_signal.wait(), Timer::after(HEARTBEAT_MAX_ALLOWED)).await {
            Either::First(_) => {
                // Got heartbeat in time, keep going
                continue;
            }
            Either::Second(_) => {
                // Timed out, must disconnect
                return;
            }
        };
    }
}

async fn active_con_loop(
    w5500_mutex: &Mutex<NoopRawMutex, ExclusiveW5500>,
    w5500_int: &mut gpio::Input<'static>,
    heartbeat_signal: &Signal<NoopRawMutex, Heartbeat>,
    cmd_channel: &channel::Sender<'static, CriticalSectionRawMutex, CmdFromPC, 16>,
) -> Result<(), &'static str> {
    loop {
        w5500_int.wait_for_low().await;
        // Interrupts we need to handle:
        // 1. IP conflict
        // 2. Destination host unreachable
        // 3. CMD socket disconnected
        // 5. CMD socket timed out
        // 4. CMD socket received data
        // 6. TELEM socket received data
        // 7. TELEM socket timed out
        let cmd_ir = {
            let mut w5500 = w5500_mutex.lock().await;
            let ir = w5500
                .ir()
                .map_err(|_| "Failed to get W5500 interrupt register")?;
            if ir.conflict() {
                w5500
                    .set_ir(Interrupt::DEFAULT.set_conflict())
                    .map_err(|_| "Failed to set W5500 interrupt register")?;
                return Err("IP conflict detected");
            }
            if ir.unreach() {
                w5500
                    .set_ir(Interrupt::DEFAULT.clear_unreach())
                    .map_err(|_| "Failed to set W5500 interrupt register")?;
                return Err("Destination host unreachable");
            }

            let cmd_ir = w5500
                .sn_ir(CMD_SOCKET)
                .map_err(|_| "Failed to get CMD socket interrupt register")?;
            if cmd_ir.discon_raised() {
                w5500
                    .set_sn_ir(CMD_SOCKET, SocketInterrupt::DEFAULT.clear_discon())
                    .map_err(|_| "Failed to clear CMD socket disconnect interrupt")?;
                return Ok(());
            }
            if cmd_ir.timeout_raised() {
                // defmt::info!("CMD TCP socket timed out!");
                w5500
                    .set_sn_ir(CMD_SOCKET, SocketInterrupt::DEFAULT.clear_timeout())
                    .map_err(|_| "Failed to clear CMD socket timeout interrupt")?;
                return Err("CMD socket timed out");
            }
            cmd_ir
        };
        if cmd_ir.recv_raised() {
            let mut w5500 = w5500_mutex.lock().await;
            w5500
                .set_sn_ir(CMD_SOCKET, SocketInterrupt::DEFAULT.clear_recv())
                .map_err(|_| "Failed to clear CMD socket recv interrupt")?;
            let mut buff = [0; 64];
            const PACKET_SIZE: usize = size_of::<CmdFromPC>();
            let num_bytes_read = w5500
                .tcp_read(CMD_SOCKET, &mut buff)
                .map_err(|_| "Failed to read TCP message on CMD socket")?;
            let bytes_slice: &[u8] = &buff[..num_bytes_read as usize];
            if num_bytes_read as usize != PACKET_SIZE {
                return Err("Received packet of incorrect size on CMD socket");
            }

            let cmd_from_pc = postcard::from_bytes::<CmdFromPC>(bytes_slice)
                .map_err(|_| "Failed to deserialize CMD packet from PC")?;
            heartbeat_signal.signal(Heartbeat {});
            drop(w5500); // yield w5500 back before await point
            cmd_channel.send(cmd_from_pc).await;
        }

        {
            let mut w5500 = w5500_mutex.lock().await;
            let telem_ir = w5500
                .sn_ir(TELEM_SOCKET)
                .map_err(|_| "Failed to get TELEM socket interrupt")?;

            if telem_ir.timeout_raised() {
                w5500
                    .set_sn_ir(TELEM_SOCKET, SocketInterrupt::DEFAULT.clear_timeout())
                    .map_err(|_| "Failed to clear TELEM socket timeout interrupt")?;
                return Err("TELEM socket timed out");
            }
            if telem_ir.recv_raised() {
                w5500
                    .set_sn_ir(TELEM_SOCKET, SocketInterrupt::DEFAULT.clear_recv())
                    .map_err(|_| "Failed to clear TELEM socket recv interrupt")?;
                let mut buf = [0; 64];
                const PACKET_SIZE: usize = size_of::<TelemFromPC>();
                let (num_bytes_read, _) = w5500
                    .udp_recv_from(TELEM_SOCKET, &mut buf)
                    .map_err(|_| "Failed to receive UDP data on TELEM socket")?;
                let bytes_slice: &[u8] = &buf[..num_bytes_read as usize];
                if num_bytes_read as usize != PACKET_SIZE {
                    return Err("Received packet of incorrect size on TELEM socket");
                }
                let telem_from_pc = postcard::from_bytes::<TelemFromPC>(bytes_slice)
                    .map_err(|_| "Failed to deserialize data received on TELEM socket")?;
                match telem_from_pc {
                    TelemFromPC::MotorPositionSetpoint(value) => {
                        MOTOR_POSITION_SETPOINT.lock(|cell| cell.set(value.to_fixed()));
                    }
                    TelemFromPC::MotorSpeedSetpoint(value) => {
                        MOTOR_SPEED_SETPOINT.lock(|cell| cell.set(value));
                    }
                }
            }
        }
    }
}

async fn push_telemetry(
    w5500_mutex: &Mutex<NoopRawMutex, ExclusiveW5500>,
) -> Result<(), &'static str> {
    let mut ticker = Ticker::every(Duration::from_hz(500));
    loop {
        {
            let mut w5500 = w5500_mutex.lock().await;
            let motor_angle_packet =
                TelemToPC::MotorAngle(MOTOR_CURRENT_POSITION.lock(|cell| cell.get().to_num()));
            const PACKET_SIZE: usize = size_of::<TelemToPC>();
            let mut buf: [u8; PACKET_SIZE] = [0; PACKET_SIZE];
            postcard::to_slice(&motor_angle_packet, &mut buf)
                .map_err(|_| "Failed to serialize TelemToPC into a byte buffer")?;

            let num_bytes = w5500
                .udp_send_to(TELEM_SOCKET, &buf, &SocketAddrV4::new(PC_ADDR, TELEM_PORT))
                .map_err(|_| "Failed to send UDP telemetry packet")?;
            if num_bytes as usize != PACKET_SIZE {
                return Err("Telemetry packet size mismatch");
            }
        }
        ticker.next().await;
    }
}

fn configure_w5500(w5500: &mut ExclusiveW5500) -> Result<(), &'static str> {
    w5500
        .set_sipr(&IP_ADDR)
        .map_err(|_| "Failed to set W5500 Static IP")?;
    w5500
        .set_gar(&GATEWAY)
        .map_err(|_| "Failed to set W5500 Gateway")?;
    w5500
        .set_subr(&SUBNET)
        .map_err(|_| "Failed to set W5500 Subnet")?;
    w5500
        .set_shar(&MAC_ADDR)
        .map_err(|_| "Failed to set W5500 MAC Address")?;

    // May as well bind the UDP port now
    w5500
        .udp_bind(TELEM_SOCKET, TELEM_PORT)
        .map_err(|_| "Failed to bind UDP port")?;

    Ok(())
}

fn configure_interrupts_pre_con(w5500: &mut ExclusiveW5500) -> Result<(), &'static str> {
    // In this state we are just waiting for a connection to bind, so let's
    // ignore all interrupts except the CMD connection one and anything that
    // could disrupt the state of "listening" (thus preventing a connection)

    // This ends up being these interrupts:
    // 1. IP Conflict
    // 2. Destination host unreachable
    // 3. CMD socket connection made
    // 4. CMD socket timeout

    const INT_MASK: w5500_ll::Interrupt = w5500_ll::Interrupt::DEFAULT.set_conflict().set_unreach();
    w5500
        .set_ir(INT_MASK)
        .map_err(|_| "Failed to clear stale W5500 interrupts")?; // clear stale interrupts
    w5500
        .set_imr(INT_MASK)
        .map_err(|_| "Failed to unmask W5500 interrupts")?; // unmask the interrupts

    // Enable interrupts for just the CMD socket
    const SOCKET_INT_MASK: u8 = CMD_SOCKET.bitmask();
    w5500
        .set_simr(SOCKET_INT_MASK)
        .map_err(|_| "Failed to enable CMD socket interrupts")?; // unmask the interrupts

    // Interrupts for the CMD socket
    const CMD_SOCKET_INT_MASK: SocketInterruptMask = SocketInterruptMask::ALL_MASKED
        .unmask_con()
        .unmask_timeout();
    w5500
        .set_sn_ir(CMD_SOCKET, CMD_SOCKET_INT_MASK)
        .map_err(|_| "Failed to clear CMD socket interrupts")?;
    w5500
        .set_sn_imr(CMD_SOCKET, CMD_SOCKET_INT_MASK)
        .map_err(|_| "Failed to unmask CMD socket interrupts")?;

    Ok(())
}

fn configure_interrupts_active_con(w5500: &mut ExclusiveW5500) -> Result<(), &'static str> {
    // The interrupts enabled for an active connection are:
    // 1. IP conflict
    // 2. Destination host unreachable
    // 3. CMD socket disconnected
    // 4. CMD socket received data
    // 5. CMD socket timed out
    // 6. TELEM socket received data
    // 7. TELEM socket timed out

    const INT_MASK: w5500_ll::Interrupt = w5500_ll::Interrupt::DEFAULT.set_conflict().set_unreach();
    w5500
        .set_ir(INT_MASK)
        .map_err(|_| "Failed to clear stale W5500 interrupts")?; // clear stale interrupts
    w5500
        .set_imr(INT_MASK)
        .map_err(|_| "Failed to unmask W5500 interrupts")?; // unmask the interrupts

    // Enable interrupts for our two sockets
    const SOCKET_INT_MASK: u8 = CMD_SOCKET.bitmask() | TELEM_SOCKET.bitmask();
    w5500
        .set_simr(SOCKET_INT_MASK)
        .map_err(|_| "Failed to set socket interrupts")?;

    // Interrupts for the CMD socket
    const CMD_SOCKET_INT_MASK: SocketInterruptMask = SocketInterruptMask::ALL_MASKED
        .unmask_discon()
        .unmask_recv()
        .unmask_timeout();
    w5500
        .set_sn_ir(CMD_SOCKET, CMD_SOCKET_INT_MASK)
        .map_err(|_| "Failed to clear CMD socket interruptps")?;
    w5500
        .set_sn_imr(CMD_SOCKET, CMD_SOCKET_INT_MASK)
        .map_err(|_| "Failed to unmask CMD socket interrupts")?;

    // Interrupts for the TELEM socket
    const TELEM_SOCKET_INT_MASK: SocketInterruptMask = SocketInterruptMask::ALL_MASKED
        .unmask_recv()
        .unmask_timeout();
    w5500
        .set_sn_ir(TELEM_SOCKET, TELEM_SOCKET_INT_MASK)
        .map_err(|_| "Failed to clear CMD socket interruptps")?;
    w5500
        .set_sn_imr(TELEM_SOCKET, TELEM_SOCKET_INT_MASK)
        .map_err(|_| "Failed to unmask TELEM socket interrupts")?;

    Ok(())
}

async fn force_close_cmd(w5500: &mut ExclusiveW5500) -> Result<(), &'static str> {
    w5500
        .set_sn_cr(CMD_SOCKET, SocketCommand::Close)
        .map_err(|_| "Failed to initialize W5500 in closed state")?;
    // wait for socket to close (should only be a couple cycles)
    while w5500
        .sn_sr(CMD_SOCKET)
        .map_err(|_| "Failed to get socket status: bus error")?
        .map_err(|_| "Failed to get socket status: conversion error")?
        != SocketStatus::Closed
    {
        Timer::after_millis(1).await;
    }
    Ok(())
}
