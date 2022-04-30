//! # USB HID Mouse Example for RustyKeys
//! # - Based on Pico USB 'Twitchy' Mouse Example -
//!
//! Creates a USB HID Class Poiting device (i.e. a virtual mouse) on a Pico
//! board, with the USB driver running in the main thread.
//!
//! It generates movement reports which will twitch the cursor up and down by a
//! few pixels, several times a second.
//!
//! See the `Cargo.toml` file for Copyright and license details.
//!
//! This is a port of
//! https://github.com/atsamd-rs/atsamd/blob/master/boards/itsybitsy_m0/examples/twitching_usb_mouse.rs

#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

// The macro for marking our interrupt functions
use rp_pico::hal::pac::interrupt;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use embedded_time::fixed_point::FixedPoint;
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;
use rp_pico::hal::gpio::DynPin;

use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Human Interface Device (HID) Class support
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::MouseReport;
use usbd_hid::hid_class::HIDClass;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then submits cursor movement
/// updates periodically.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }

    // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Set up the USB HID Class Device driver, providing Mouse Reports
    let usb_hid = HIDClass::new(bus_ref, MouseReport::desc(), 60);
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet.
        USB_HID = Some(usb_hid);
    }

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27da))
        .manufacturer("Bracket Works")
        .product("Rusty Mousey")
        .serial_number("TEST")
        .device_class(0xEF) // misc
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };
    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure GPIO 25 as an output
    // let mut led_pin = pins.gpio25.into_push_pull_output();

    const ROWS: usize = 2;
    let mut rows: [DynPin; ROWS] = [
        pins.gpio22.into(),
        pins.gpio21.into(),
    ];
    const COLS: usize = 3;
    let mut cols: [DynPin; COLS] = [
        pins.gpio18.into(),
        pins.gpio17.into(),
        pins.gpio16.into(),
    ];

    // Setup pins
    for row in &mut rows {
        row.into_push_pull_output();
        row.set_high().ok();
    }
    for col in &mut cols {
        col.into_pull_up_input();
    }

    let mut prev_matrix: [[bool; COLS]; ROWS] = [[false; COLS]; ROWS];
    loop {
        delay.delay_ms(1);

        let mut matrix: [[bool; COLS]; ROWS] = [[false; COLS]; ROWS];
        // Scan
        for row in 0..ROWS {
            rows[row].set_low().ok();
            delay.delay_us(10);
            for col in 0..COLS {
                matrix[row][col] = cols[col].is_low().unwrap();
            }
            rows[row].set_high().ok();
        }

        if matrix != prev_matrix {
            // Report
            let l_btn = if matrix[0][0] { 1 } else { 0 };
            let r_btn = if matrix[0][2] { 1 } else { 0 };
            let btns = match (l_btn, r_btn) { (1, 0) => 1, (0, 1) => 2, (1, 1) => 3, _ => 0 };
            let (mut x, mut y) = (0, 0);
            x -= if matrix[1][0] { 10 } else { 0 };
            x += if matrix[1][2] { 10 } else { 0 };
            y -= if matrix[0][1] { 10 } else { 0 };
            y += if matrix[1][1] { 10 } else { 0 };

            let report = MouseReport {
                x,
                y,
                buttons: btns,
                wheel: 0,
                pan: 0,
            };
            push_mouse_movement(report).ok().unwrap_or(0);
        }
        prev_matrix = matrix;
    }
}

/// Submit a new mouse movement report to the USB stack.
///
/// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
fn push_mouse_movement(report: MouseReport) -> Result<usize, usb_device::UsbError> {
    cortex_m::interrupt::free(|_| unsafe {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
        USB_HID.as_mut().map(|hid| hid.push_input(&report))
    })
    .unwrap()
}

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_hid = USB_HID.as_mut().unwrap();
    usb_dev.poll(&mut [usb_hid]);
}

// End of file
