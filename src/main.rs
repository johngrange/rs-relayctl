//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico::{self as bsp, hal::{self}};
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::init_clocks_and_plls,
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

// USB device support
use usb_device::{class_prelude::*,prelude::*};

// USB communications device support
use usbd_serial::SerialPort;

// Formatted string support
use core::fmt::Write;
use heapless::String;


use cmdparser::CmdParser;
use cmdparser::Cmd;

type CtlErr = String<10>;

struct Controller<R1: OutputPin, R2: OutputPin, L: OutputPin> {
    relay_1 : R1,
    relay_1_on : bool,
    relay_2 : R2,
    relay_2_on : bool,
    led : L,
    led_on : bool,
}



impl<R1: OutputPin, R2: OutputPin, L: OutputPin> Controller<R1,R2,L> {
    fn new(r1 : R1, r2: R2, l:L) -> Self {
        Controller {
            relay_1 : r1,
            relay_1_on: false,
            relay_2 : r2,
            relay_2_on: false,
            led: l,
            led_on: false,
        }
    }
    fn process_cmd(&mut self, cmd : cmdparser::Cmd) -> Result<String<20>, CtlErr> {
        match cmd {
            Cmd::On(name) =>
            {
                let sname = name.as_str();
                match sname {
                    "R1" => {
                        self.relay_1.set_high().unwrap();
                        self.relay_1_on = true;
                    },
                    "R2" => {
                        self.relay_2.set_high().unwrap();
                        self.relay_2_on = true;
                    },
                    "LED" => {
                        self.led.set_high().unwrap();
                        self.led_on = true;
                    },
                    _default => {
                        let mut e = CtlErr::new();
                        write!(& mut e,"{} not found", name).unwrap();
                        return Err(e)
                    }
                }
                let mut s : String<20> = String::new();
                write!(& mut s, "{} ON",name).unwrap();
                Ok(s)
            },
            Cmd::Off(name) => {
                let sname = name.as_str();
                match sname {
                    "R1" => {
                        self.relay_1.set_low().unwrap();
                        self.relay_1_on = false;
                    },
                    "R2" => {
                        self.relay_2.set_low().unwrap();
                        self.relay_2_on = false;
                    },
                    "LED" => {
                        self.led.set_low().unwrap();
                        self.led_on = false;
                    },
                    _default => {
                        let mut e = CtlErr::new();
                        write!(& mut e,"{} not found", name).unwrap();
                        return Err(e)
                    }
                }
                let mut s : String<20> = String::new();
                write!(& mut s, "{} ON",name).unwrap();
                Ok(s)

            },
            Cmd::Get(name) => {
                let sname = name.as_str();
                let state = match sname {
                    "R1" => {
                        if self.relay_1_on { "ON"} else { "OFF"}
                        
                    },
                    "R2" => {
                        if self.relay_2_on { "ON"} else { "OFF"}
                        
                    },
                    "LED" => {
                        if self.led_on { "ON"} else { "OFF"}
                        
                    },
                    _default => {
                        let mut e = CtlErr::new();
                        write!(& mut e,"{} not found", name).unwrap();
                        return Err(e)
                    }                    
                };
                let mut s : String<20> = String::new();
                write!(& mut s, "{} {}",name, state).unwrap();
                Ok(s)
            },
            Cmd::GetAll => {
                let mut s : String<20> = String::new();
                write!(& mut s, "R1 {}, R2 {}, LED {}",
                    if self.relay_1_on { "ON"} else { "OFF"},
                    if self.relay_2_on { "ON"} else { "OFF"},
                    if self.led_on { "ON"} else { "OFF"}
                ).unwrap();
                Ok(s)
            }
        }
        
    }

}

#[entry]
fn main() -> ! {

    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let mut parser = CmdParser::new();

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // set up the usb driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB communications class driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Pico Relay Corp")
            .product("RS-Relay Controller")
            .serial_number("12344554")])       
        .unwrap()
        .device_class(2)
        .build();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    //
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead.
    // One way to do that is by using [embassy](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/wifi_blinky.rs)
    //
    // If you have a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here. Don't forget adding an appropriate resistor
    // in series with the LED.
    let led_pin = pins.led.into_push_pull_output();

    let relay_1 = pins.gpio6.into_push_pull_output();
    let relay_2 = pins.gpio7.into_push_pull_output();

    let mut controller = Controller::new(relay_1, relay_2, led_pin);
    
    loop {

        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {/*do nothing */}
                Ok(0)  => {/*do nothing */}
                Ok(count) => {
                    
                    match parser.parse_bytes(&buf[..count]) {
                        Ok(res) => {
                            match res {
                                Some(cmd) => {
                                    // Got a command successfully parsed, process it
                                    match controller.process_cmd(cmd) {
                                        Ok(msg) => {
                                            let _ = serial.write(msg.as_bytes());
                                        },
                                        Err(err) => {
                                            let mut text : String<64> = String::new();
                                            writeln!(&mut text, "ERROR: {:?}", err).unwrap();
                                            let _ = serial.write(text.as_bytes());                
                                        }
                                    };
                                }
                                None => {/* no command processed, do nothing */}
                            }                                                        
                        },
                        Err(e) => {
                            let mut text : String<64> = String::new();
                            writeln!(&mut text, "ERROR: {:?}", e).unwrap();
                            let _ = serial.write(text.as_bytes());
                        }
                    }
                }
            }
        }
    }
}
// End of file