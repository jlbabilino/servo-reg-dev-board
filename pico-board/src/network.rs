use core::cell::Cell;
use core::cell::RefCell;
use core::net::Ipv4Addr;
use core::net::SocketAddrV4;

use embassy_rp::gpio;
use embassy_rp::peripherals::SPI0;
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Instant;
use embedded_hal_bus::spi::ExclusiveDevice;
use w5500_hl::Tcp;
use w5500_hl::Udp;
use w5500_ll::Interrupt;
use w5500_ll::SocketCommand;
use w5500_ll::SocketInterrupt;
use w5500_ll::SocketInterruptMask;
use w5500_ll::SocketMode;
use w5500_ll::SocketStatus;
use w5500_ll::eh1::vdm::W5500;
use w5500_ll::net::Eui48Addr;
use w5500_ll::{Protocol, Registers, Sn};

use crate::LED_COMMAND_CH;
use crate::MOTOR_CUM_ANGLE_MUTEX;

pub type ExclusiveW5500 = W5500<
    ExclusiveDevice<
        embassy_rp::spi::Spi<'static, SPI0, embassy_rp::spi::Async>,
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
///
///

#[embassy_executor::task]
pub async fn network_task(w5500: ExclusiveW5500, mut w5500_int: gpio::Input<'static>) {
    let mut w5500 = w5500;

    configure_w5500(&mut w5500);

    embassy_time::Timer::after_millis(10).await;

    let mut last_hearbeat = Instant::now();

    // let out = test_mutex.lock(|cell| cell);

    w5500
        .udp_bind(TELEM_SOCKET, TELEM_PORT)
        .expect("Failed to bind UDP socket");

    loop {
        // let (status, cmd) = w5500_mutex.lock(|cell| {
        //     let w5500 = cell.borrow_mut();
        //     let status = w5500.sn_sr(CMD_SOCKET).unwrap().unwrap();
        //     let cmd = w5500.sn_cr(CMD_SOCKET).unwrap();
        //     (status, cmd)
        // });

        let status = w5500.sn_sr(CMD_SOCKET).unwrap().unwrap();
        let cmd = w5500.sn_cr(CMD_SOCKET).unwrap();

        // let inner = w5500_mutex.lock(|cell| cell);

        // defmt::info!(
        //     "Socket status: {:?}, command: {:?}, last hearbeat time = {:?}",
        //     status,
        //     cmd,
        //     (Instant::now() - last_hearbeat).as_secs()
        // );

        match status {
            SocketStatus::Closed => {
                defmt::info!("Listening for CMD TCP server on port {}...", CMD_PORT);
                w5500.tcp_listen(CMD_SOCKET, CMD_PORT).unwrap();
            }
            SocketStatus::CloseWait => {
                defmt::info!("CMD TCP server disconnected by client.");
                w5500
                    .set_sn_cr(CMD_SOCKET, w5500_ll::SocketCommand::Disconnect)
                    .unwrap();
                embassy_time::Timer::after_millis(100).await;
                continue;
            }
            SocketStatus::FinWait => {
                defmt::info!("CMD TCP server stuck in weird state.");
                w5500
                    .set_sn_cr(CMD_SOCKET, w5500_ll::SocketCommand::Close)
                    .unwrap();
                embassy_time::Timer::after_millis(100).await;
                continue;
            }
            SocketStatus::Established | SocketStatus::SynRecv | SocketStatus::SynSent => {
                if (Instant::now() - last_hearbeat).as_millis() >= 100 {
                    defmt::error!("Lost connection to client! Restarting");
                    w5500
                        .set_sn_cr(CMD_SOCKET, w5500_ll::SocketCommand::Close)
                        .unwrap();
                    LED_COMMAND_CH.send(DISABLED_ANIM).await;
                    embassy_time::Timer::after_millis(100).await;
                }
                let pc_ip = w5500.sn_dipr(CMD_SOCKET).unwrap();
                let dummy_data = [0xA4];
                w5500.tcp_write(CMD_SOCKET, &dummy_data).unwrap();
                let mut recv_buf: [u8; 1] = [0];
                while let Ok(num_bytes) = w5500.udp_recv_from(TELEM_SOCKET, &mut recv_buf) {
                    defmt::info!("UDP recv: {}", &recv_buf[0]);
                }
                let motor_rot: f32 = MOTOR_CUM_ANGLE_MUTEX.lock(|cell| cell.get()).to_num();
                let packet = motor_rot.to_le_bytes();
                w5500
                    .udp_send_to(TELEM_SOCKET, &packet, &SocketAddrV4::new(pc_ip, TELEM_PORT))
                    .expect("Couldn't send the udp thing");
            }
            _ => {}
        }

        // once interrupt fires, must be connected
        w5500_int.wait_for_low().await;

        while w5500_int.is_low() {
            let ir = w5500.ir().unwrap();
            if ir.unreach() {
                defmt::info!("Client unreachable!");
                let unreach_clr = Interrupt::DEFAULT.clear_unreach();
                w5500.set_ir(unreach_clr).unwrap();
            }

            let cmd_ir = w5500.sn_ir(CMD_SOCKET).unwrap();
            if cmd_ir.con_raised() {
                defmt::info!("CMD TCP Connected!");
                LED_COMMAND_CH.send(ENABLED_ANIM).await;
                last_hearbeat = Instant::now();
                let con_clr = SocketInterrupt::DEFAULT.clear_con();
                w5500.set_sn_ir(CMD_SOCKET, con_clr).unwrap();
            }
            if cmd_ir.discon_raised() {
                defmt::info!("CMD TCP Disconnected!");
                LED_COMMAND_CH.send(DISABLED_ANIM).await;
                let discon_clr = SocketInterrupt::DEFAULT.clear_discon();
                w5500.set_sn_ir(CMD_SOCKET, discon_clr).unwrap();
            }
            if cmd_ir.timeout_raised() {
                defmt::info!("CMD TCP socket timed out!");
                let timeout_clr = SocketInterrupt::DEFAULT.clear_timeout();
                w5500.set_sn_ir(CMD_SOCKET, timeout_clr).unwrap();
            }
            if cmd_ir.recv_raised() {
                let mut buff = [0; 1024];
                let num_bytes = w5500.tcp_read(CMD_SOCKET, &mut buff).unwrap();
                let bytes: &[u8] = &buff[..num_bytes as usize];
                defmt::info!("Received data: {}", bytes);
                const RECV_CLR: SocketInterrupt = SocketInterrupt::DEFAULT.clear_recv();
                w5500.set_sn_ir(CMD_SOCKET, RECV_CLR).unwrap();
            }
            if cmd_ir.sendok_raised() {
                // defmt::info!("TX successful!");
                last_hearbeat = Instant::now();
                const SENDOK_CLR: SocketInterrupt = SocketInterrupt::DEFAULT.clear_sendok();
                w5500.set_sn_ir(CMD_SOCKET, SENDOK_CLR).unwrap();
            }

            let telem_ir = w5500.sn_ir(TELEM_SOCKET).unwrap();
            if telem_ir.recv_raised() {
                const RECV_CLR: SocketInterrupt = SocketInterrupt::DEFAULT.clear_recv();
                w5500.set_sn_ir(TELEM_SOCKET, RECV_CLR).unwrap();
            }
            if telem_ir.sendok_raised() {
                const SENDOK_CLR: SocketInterrupt = SocketInterrupt::DEFAULT.clear_sendok();
                w5500.set_sn_ir(TELEM_SOCKET, SENDOK_CLR).unwrap();
            }
            embassy_time::Timer::after_millis(10).await;
        }

        // let mut rx_buffer = [0; 4096];

        // w5500.udp_recv_from(TELEM_SOCKET, &mut rx_buffer).unwrap();

        // 'tcp_loop: loop {
        //     w5500_int.wait_for_low().await;
        //     let sn_ir = w5500.sn_ir(CMD_SOCKET).unwrap();
        //     if sn_ir.discon_raised() {
        //         defmt::error!("Lost TCP connection! Will attempt to reconnect...");
        //         w5500
        //             .set_sn_ir(CMD_SOCKET, SocketInterrupt::DEFAULT.clear_discon())
        //             .unwrap();
        //         break 'conn_loop; // go back to start
        //     }
        //     if sn_ir.recv_raised() {
        //         w5500
        //             .set_sn_ir(CMD_SOCKET, SocketInterrupt::DEFAULT.clear_recv())
        //             .unwrap();
        //         defmt::info!("Received data");
        //         match w5500.tcp_read(Sn::Sn0, &mut rx_buffer) {
        //             Ok(0) => {
        //                 embassy_time::Timer::after(embassy_time::Duration::from_millis(20)).await;
        //             }
        //             Ok(_) => {
        //                 // defmt::info!("Received: {}", bytes_received);
        //                 let byte_array: [u8; 4] = rx_buffer[..4].try_into().unwrap();
        //                 let hue_degrees: f32 = f32::from_be_bytes(byte_array);

        //                 defmt::info!("Successfully decoded float: {}", hue_degrees);
        //             }
        //             Err(e) => {
        //                 defmt::error!("Error receiving");
        //             }
        //         }
        //     }
        //     // embassy_time::Timer::after(embassy_time::Duration::from_millis(20)).await;
        // }
    }
}

fn configure_w5500(w5500: &mut ExclusiveW5500) {
    w5500.set_sipr(&IP_ADDR).unwrap();
    w5500.set_gar(&GATEWAY).unwrap();
    w5500.set_subr(&SUBNET).unwrap();
    w5500.set_shar(&MAC_ADDR).unwrap();

    // w5500.set_intlevel(0x00FA).unwrap(); // TODO: try to delete this and see if it still works

    // Overall interrupts like ip conflicts,
    // const INT_MASK: w5500_ll::Interrupt = w5500_ll::Interrupt::DEFAULT.set_conflict().set_unreach();
    const INT_MASK: w5500_ll::Interrupt = w5500_ll::Interrupt::DEFAULT;
    w5500.set_imr(INT_MASK).unwrap();

    // Enable interrupts for our two sockets
    const SOCKET_INT_MASK: u8 = CMD_SOCKET.bitmask() | TELEM_SOCKET.bitmask();
    w5500.set_simr(SOCKET_INT_MASK).unwrap();

    // Interrupts for the CMD socket
    const CMD_SOCKET_INT_MASK: SocketInterruptMask = SocketInterruptMask::ALL_MASKED
        .unmask_con()
        .unmask_discon()
        .unmask_recv()
        .unmask_sendok()
        .unmask_timeout();
    w5500.set_sn_imr(CMD_SOCKET, CMD_SOCKET_INT_MASK).unwrap();

    // Interrupts for the TELEM socket
    // const TELEM_SOCKET_INT_MASK: SocketInterruptMask = SocketInterruptMask::ALL_MASKED
    //     .unmask_con()
    //     .unmask_discon()
    //     .unmask_recv()
    //     .unmask_sendok()
    //     .unmask_timeout();
    // w5500
    //     .set_sn_imr(TELEM_SOCKET, TELEM_SOCKET_INT_MASK)
    //     .unwrap();

    // w5500.set_sn_kpalvtr(CMD_SOCKET, 0).unwrap();
    w5500
        .set_sn_cr(CMD_SOCKET, w5500_ll::SocketCommand::Disconnect)
        .unwrap();
}
