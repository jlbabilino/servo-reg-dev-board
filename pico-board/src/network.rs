use core::net::Ipv4Addr;

use embassy_rp::gpio;
use embassy_rp::peripherals::SPI0;
use embedded_hal_bus::spi::ExclusiveDevice;
use w5500_hl::Tcp;
use w5500_hl::Udp;
use w5500_ll::SocketInterrupt;
use w5500_ll::eh1::vdm::W5500;
use w5500_ll::{Registers, Sn};

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
    // Enable Socket 0 to generate interrupts on the INT pin
    w5500.set_simr(0x01).unwrap();

    // Static IPV4 Config
    let ip_addr = Ipv4Addr::new(192, 168, 1, 20); // Pico's static IP
    let gateway = Ipv4Addr::new(192, 168, 1, 1);
    let subnet = Ipv4Addr::new(255, 255, 255, 0);
    let mac_addr = w5500_ll::net::Eui48Addr::new(0x02, 0x00, 0x11, 0x22, 0x33, 0x44); // arbitrary

    w5500.set_sipr(&ip_addr).unwrap();
    w5500.set_gar(&gateway).unwrap();
    w5500.set_subr(&subnet).unwrap();
    w5500.set_shar(&mac_addr).unwrap();


    // Here's how we'll use each hardware socket
    // Sn0: TCP - CMD - high-level signals like enable/disable, watchdog, etc.
    // Sn1: UDP - TELEM - high-frequency signals like motor's position, commanded position, etc.
    const CMD_SOCKET: Sn = Sn::Sn0;
    const CMD_PORT: u16 = 15397;
    const TELEM_SOCKET: Sn = Sn::Sn1;
    const TELEM_PORT: u16 = 15509; 

    loop {
        // Outermost loop for disconnecting/reconnecting
        // Set up TCP server
        defmt::info!("Opening TCP server on port {}...", PORT);
        w5500.tcp_listen(SOCKET, PORT).unwrap();

        w5500.udp_bind()

        // w5500.tcp_connect(sn, port, addr)
        defmt::info!("TCP open");
        w5500_int.wait_for_low().await;
        defmt::info!("Connected");
        loop {
            let sn_ir = w5500.sn_ir(SOCKET).unwrap();
            if sn_ir.con_raised() {
                let con_clr = SocketInterrupt::DEFAULT.clear_con();
                w5500.set_sn_ir(SOCKET, con_clr).unwrap();
                break;
            }
        }

        let mut rx_buffer = [0; 4096];

        loop {
            let sn_ir = w5500.sn_ir(SOCKET).unwrap();
            if sn_ir.recv_raised() {
                w5500.set_sn_ir(SOCKET, sn_ir).unwrap();
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
            embassy_time::Timer::after(embassy_time::Duration::from_millis(20)).await;
        }
    }
}
