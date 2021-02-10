#![no_std]
#![no_main]

use cortex_m::prelude::*;
use cortex_m_rt::entry;
use panic_halt as _;
use rp2040_pac::{Peripherals, CorePeripherals};
use cortex_m::delay::Delay;

#[link_section = ".boot_loader"]
#[used]
pub static BOOT_LOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER;

/// Handle peripheral resets so the chip is usable.
unsafe fn setup_chip(p: &mut rp2040_pac::Peripherals) {
    // Now reset all the peripherals, except QSPI and XIP (we're using those
    // to execute from external flash!)
    p.RESETS.reset.write(|w| {
        w.adc().set_bit();
        w.busctrl().set_bit();
        w.dma().set_bit();
        w.i2c0().set_bit();
        w.i2c1().set_bit();
        w.io_bank0().set_bit();
        w.io_qspi().clear_bit();
        w.jtag().set_bit();
        w.pads_bank0().set_bit();
        w.pads_qspi().clear_bit();
        w.pio0().set_bit();
        w.pio1().set_bit();
        w.pll_sys().clear_bit();
        w.pll_usb().clear_bit();
        w.pwm().set_bit();
        w.rtc().set_bit();
        w.spi0().set_bit();
        w.spi1().set_bit();
        w.syscfg().set_bit();
        w.sysinfo().set_bit();
        w.tbman().set_bit();
        w.timer().set_bit();
        w.uart0().set_bit();
        w.uart1().set_bit();
        w.usbctrl().set_bit();
        w
    });

    const RESETS_RESET_BITS: u32 = 0x01ffffff;
    const RESETS_RESET_USBCTRL_BITS: u32 = 0x01000000;
    const RESETS_RESET_UART1_BITS: u32 = 0x00800000;
    const RESETS_RESET_UART0_BITS: u32 = 0x00400000;
    const RESETS_RESET_SPI1_BITS: u32 = 0x00020000;
    const RESETS_RESET_SPI0_BITS: u32 = 0x00010000;
    const RESETS_RESET_RTC_BITS: u32 = 0x00008000;
    const RESETS_RESET_ADC_BITS: u32 = 0x00000001;

    // We want to take everything out of reset, except these peripherals:
    //
    // * ADC
    // * RTC
    // * SPI0
    // * SPI1
    // * UART0
    // * UART1
    // * USBCTRL
    //
    // These must stay in reset until the clocks are sorted out.
    const PERIPHERALS_TO_UNRESET: u32 = RESETS_RESET_BITS
        & !(RESETS_RESET_ADC_BITS
        | RESETS_RESET_RTC_BITS
        | RESETS_RESET_SPI0_BITS
        | RESETS_RESET_SPI1_BITS
        | RESETS_RESET_UART0_BITS
        | RESETS_RESET_UART1_BITS
        | RESETS_RESET_USBCTRL_BITS);

    // Write 0 to the reset field to take it out of reset
    p.RESETS.reset.modify(|_r, w| {
        w.busctrl().clear_bit();
        w.dma().clear_bit();
        w.i2c0().clear_bit();
        w.i2c1().clear_bit();
        w.io_bank0().clear_bit();
        w.io_qspi().clear_bit();
        w.jtag().clear_bit();
        w.pads_bank0().clear_bit();
        w.pads_qspi().clear_bit();
        w.pio0().clear_bit();
        w.pio1().clear_bit();
        w.pll_sys().clear_bit();
        w.pll_usb().clear_bit();
        w.pwm().clear_bit();
        w.syscfg().clear_bit();
        w.sysinfo().clear_bit();
        w.tbman().clear_bit();
        w.timer().clear_bit();
        w
    });

    while (!p.RESETS.reset_done.read().bits() & PERIPHERALS_TO_UNRESET) != 0 {
        cortex_m::asm::nop();
    }
}

// https://github.com/raspberrypi/pico-sdk/blob/2d5789eca89658a7f0a01e2d6010c0f254605d72/src/rp2_common/hardware_gpio/gpio.c#L154-L158
fn gpio_init(p: &mut Peripherals, pin: u8) {
    p.SIO.gpio_oe_clr.write(|w| unsafe {
        w.bits(1 << pin);
        w
    });
    p.SIO.gpio_out_clr.write(|w| unsafe {
        w.bits(1 << pin);
        w
    });

    // Set input enable on, output disable off
    p.PADS_BANK0.gpio[pin as usize].write(|w| {
        w.od().clear_bit();
        w.ie().set_bit();
        w
    });

    p.IO_BANK0.gpio[pin as usize].gpio_ctrl.write(|w| {
        w.funcsel().sio_0();
        w
    });
}

enum Dir {
    In,
    Out,
}

fn gpio_set_dir(p: &mut Peripherals, dir: Dir, pin: u8) {
    match dir {
        Dir::In => p.SIO.gpio_oe_clr.write(|w| unsafe {
            w.bits(1 << pin);
            w
        }),
        Dir::Out => p.SIO.gpio_oe_set.write(|w| unsafe {
            w.bits(1 << pin);
            w
        }),
    };
}

fn gpio_out_set(p: &mut Peripherals, pin: u8) {
    p.SIO.gpio_out_set.write(|w| unsafe {
       w.bits(1 << pin);
       w
    });
}

fn gpio_out_clr(p: &mut Peripherals, pin: u8) {
    p.SIO.gpio_out_clr.write(|w| unsafe {
        w.bits(1 << pin);
        w
    });
}

fn gpio_in(p: &mut Peripherals, pin: u8) -> bool {
    p.SIO.gpio_in.read().bits() & (1 << pin) == 1
}

struct Dht {
    humidity: f32,
    temp: f32,
}

fn read_from_dht(p: &mut Peripherals, delay: &mut Delay) -> Dht {
    gpio_set_dir(p, Dir::Out, DHT_PIN);
    gpio_out_clr(p, DHT_PIN);
    delay.delay_ms(20);
    gpio_set_dir(p, Dir::In, DHT_PIN);

    gpio_out_set(p, LED_PIN);
    // wait for low
    loop {
        if gpio_in(p, DHT_PIN) == false {
            break;
        }
    }
    // wait for high (ack)
    loop {
        if gpio_in(p, DHT_PIN) == true {
            break;
        }
    }
    // wait for low (start)
    loop {
        if gpio_in(p, DHT_PIN) == false {
            break;
        }
    }
    gpio_out_clr(p, LED_PIN);

    // Every bit starts with th DHT pulling the line low for 50us, then
    //   0: pulled up for ~26-28us
    //   1: pulled up for 70us
    // TODO: read 40 bits

    Dht { humidity: 0.0, temp: 0.0 }
}

static LED_PIN: u8 = 25;
static DHT_PIN: u8 = 15;

#[entry]
fn main() -> ! {
    let mut p = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();

    unsafe { setup_chip(&mut p) };

    let mut delay = Delay::new(cp.SYST, 8_000_000);

    gpio_init(&mut p, LED_PIN);
    gpio_init(&mut p, DHT_PIN);
    gpio_set_dir(&mut p, Dir::Out,LED_PIN);

    loop {
        delay.delay_ms(1000);
        let _ = read_from_dht(&mut p, &mut delay);
    }
}
