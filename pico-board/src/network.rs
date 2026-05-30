use core::net::Ipv4Addr;

use embassy_rp::gpio;
use embassy_rp::peripherals::SPI0;
use embedded_hal_bus::spi::ExclusiveDevice;
use w5500_hl::Tcp;
use w5500_hl::Udp;
use w5500_ll::SocketInterrupt;
use w5500_ll::SocketInterruptMask;
use w5500_ll::SocketStatus;
use w5500_ll::eh1::vdm::W5500;
use w5500_ll::net::Eui48Addr;
use w5500_ll::{Registers, Sn};

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
pub async fn network_task(
    w5500: &'static mut W5500<
        ExclusiveDevice<
            embassy_rp::spi::Spi<'static, SPI0, embassy_rp::spi::Async>,
            gpio::Output<'static>,
            embassy_time::Delay,
        >,
    >,
    w5500_int: &'static mut gpio::Input<'static>,
) {
    // Static IPV4 Config
    const IP_ADDR: Ipv4Addr = Ipv4Addr::new(192, 168, 1, 20); // Pico's static IP
    const GATEWAY: Ipv4Addr = Ipv4Addr::new(192, 168, 1, 1);
    const SUBNET: Ipv4Addr = Ipv4Addr::new(255, 255, 255, 0);
    const MAC_ADDR: Eui48Addr = Eui48Addr::new(0x02, 0x00, 0x11, 0x22, 0x33, 0x44); // arbitrary

    w5500.set_sipr(&IP_ADDR).unwrap();
    w5500.set_gar(&GATEWAY).unwrap();
    w5500.set_subr(&SUBNET).unwrap();
    w5500.set_shar(&MAC_ADDR).unwrap();

    const CMD_SOCKET: Sn = Sn::Sn0;
    const CMD_PORT: u16 = 15397;
    const TELEM_SOCKET: Sn = Sn::Sn1;
    const TELEM_PORT: u16 = 15509;

    // Overall interrupts like ip conflicts,
    const INT_MASK: w5500_ll::Interrupt = w5500_ll::Interrupt::DEFAULT.set_conflict().set_unreach();
    w5500.set_imr(INT_MASK).unwrap();

    // Enable interrupts for our two sockets
    const SOCKET_INT_MASK: u8 = CMD_SOCKET.bitmask() | TELEM_SOCKET.bitmask();
    w5500.set_simr(SOCKET_INT_MASK).unwrap();

    // Interrupts for the CMD socket
    const CMD_SOCKET_INT_MASK: SocketInterruptMask = SocketInterruptMask::DEFAULT
        .mask_con()
        .mask_discon()
        .mask_recv()
        .mask_sendok()
        .mask_timeout();
    w5500.set_sn_imr(CMD_SOCKET, CMD_SOCKET_INT_MASK).unwrap();

    // Interrupts for the TELEM socket
    const TELEM_SOCKET_INT_MASK: SocketInterruptMask = SocketInterruptMask::DEFAULT
        .mask_con()
        .mask_discon()
        .mask_recv()
        .mask_sendok()
        .mask_timeout();
    w5500
        .set_sn_imr(TELEM_SOCKET, TELEM_SOCKET_INT_MASK)
        .unwrap();

    let mut is_connected = false;

    'conn_loop: loop {
        // Outermost loop for disconnecting/reconnecting
        // Set up TCP server
        defmt::info!("Opening CMD TCP server on port {}...", CMD_PORT);

        // Pico is TCP server -> listen on port
        w5500.tcp_listen(CMD_SOCKET, CMD_PORT).unwrap();
        // once interrupt fires, must be connected
        w5500_int.wait_for_low().await;

        let sn_ir = w5500.sn_ir(CMD_SOCKET).unwrap();
        if sn_ir.con_raised() {
            defmt::info!("CMD TCP Connected!");
            let con_clr = SocketInterrupt::DEFAULT.clear_con();
            w5500.set_sn_ir(CMD_SOCKET, con_clr).unwrap();
        } else {
            defmt::panic!("Interrupt wasn't for connect, that's weird");
        }

        let res = w5500.sn_sr(CMD_SOCKET).unwrap().unwrap();

        loop {
            // first check CMD socket status
            let cmd_status = w5500.sn_sr(CMD_SOCKET).unwrap().unwrap();
        }

        let mut rx_buffer = [0; 4096];

        // w5500.udp_recv_from(TELEM_SOCKET, &mut rx_buffer).unwrap();

        'tcp_loop: loop {
            w5500_int.wait_for_low().await;
            let sn_ir = w5500.sn_ir(CMD_SOCKET).unwrap();
            if sn_ir.discon_raised() {
                defmt::error!("Lost TCP connection! Will attempt to reconnect...");
                w5500
                    .set_sn_ir(CMD_SOCKET, SocketInterrupt::DEFAULT.clear_discon())
                    .unwrap();
                break 'conn_loop; // go back to start
            }
            if sn_ir.recv_raised() {
                w5500
                    .set_sn_ir(CMD_SOCKET, SocketInterrupt::DEFAULT.clear_recv())
                    .unwrap();
                defmt::info!("Received data");
                match w5500.tcp_read(Sn::Sn0, &mut rx_buffer) {
                    Ok(0) => {
                        embassy_time::Timer::after(embassy_time::Duration::from_millis(20)).await;
                    }
                    Ok(_) => {
                        // defmt::info!("Received: {}", bytes_received);
                        let byte_array: [u8; 4] = rx_buffer[..4].try_into().unwrap();
                        let hue_degrees: f32 = f32::from_be_bytes(byte_array);

                        defmt::info!("Successfully decoded float: {}", hue_degrees);
                    }
                    Err(e) => {
                        defmt::error!("Error receiving");
                    }
                }
            }
            // embassy_time::Timer::after(embassy_time::Duration::from_millis(20)).await;
        }
    }
}
