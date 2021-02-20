use core::marker::PhantomData;
use rp2040_pac::generic::{Reg, ResetValue, Writable};

pub struct Pin<F> {
    pin: u8,
    _func: PhantomData<F>,
}
pub struct Unspecified;
pub struct Sio<IO> {
    _io: PhantomData<IO>,
}

pub struct Input<M> {
    _mode: PhantomData<M>,
}
pub struct Output;

pub struct SchmittTrigger;
pub struct PullUp;
pub struct PullDown;

impl Pin<()> {
    pub fn new(pin: u8) -> Pin<Unspecified> {
        Pin {
            pin,
            _func: PhantomData,
        }
    }
}

impl<F> Pin<F> {

    /// Returns the PADS_BANK0 register for this pin.
    fn pad_reg(&self) -> &rp2040_pac::pads_bank0::GPIO {
        let pad_block = unsafe { &*rp2040_pac::PADS_BANK0::ptr() };
        &pad_block.gpio[self.pin as usize]
    }

    /// Returns the IO_BANK0 register for this pin.
    fn io_reg(&self) -> &rp2040_pac::io_bank0::GPIO {
        let io_block = unsafe { &*rp2040_pac::IO_BANK0::ptr() };
        &io_block.gpio[self.pin as usize]
    }

    /// Returns the SIO register block.
    fn sio_block(&self) -> &rp2040_pac::sio::RegisterBlock {
        unsafe { &*rp2040_pac::SIO::ptr() }
    }

    /// Utility function to set this Pin's bit in a Writable register.
    fn set_bit<REG>(&self, reg: &Reg<u32, REG>)
    where
        Reg<u32, REG>: Writable + ResetValue<Type = u32>,
    {
        reg.write(|w| unsafe {
            w.bits(1 << self.pin);
            w
        });
    }

    /// Utility function to set this Pin's bit in the Writable register returned by f.
    fn set_sio_bit<'a, Fn, REG: 'a>(&'a self, f: Fn)
        where
            Fn: FnOnce(&'a rp2040_pac::sio::RegisterBlock) -> &'a Reg<u32, REG>,
            Reg<u32, REG>: Writable + ResetValue<Type = u32>,
    {
        self.set_bit(f(self.sio_block()));
    }
}

impl Pin<Unspecified> {
    pub fn func_sio(self) -> Pin<Sio<Unspecified>> {
        let pin: Pin<Sio<Unspecified>> = Pin {
            pin: self.pin,
            _func: PhantomData,
        };

        // pico-sdk gpio_init
        pin.oe_clr();
        pin.out_clr();

        // pico-sdk gpio_set_function
        self.pad_reg().write(|w| {
            w.od().clear_bit();
            w.ie().set_bit();
            w
        });

        self.io_reg().gpio_ctrl.write(|w| {
            w.funcsel().sio_0();
            w
        });

        pin
    }
}

macro_rules! sio_reg {
    ($name:tt, $n:ident) => {
        fn $name(&self) {
            self.sio_block().$n.write(|w| unsafe {
                w.bits(1 << self.pin);
                w
            });
        }
    }
}

impl<IO> Pin<Sio<IO>> {
    pub fn input(self) -> Pin<Sio<Input<Unspecified>>> {
        self.oe_clr();
        Pin {
            pin: self.pin,
            _func: PhantomData,
        }
    }

    pub fn output(self) -> Pin<Sio<Output>> {
        self.oe_set();
        Pin {
            pin: self.pin,
            _func: PhantomData,
        }
    }

    sio_reg!(out_set, gpio_out_set);
    sio_reg!(out_clr, gpio_out_clr);
    sio_reg!(out_xor, gpio_out_xor);
    sio_reg!(oe_set, gpio_oe_set);
    sio_reg!(oe_clr, gpio_oe_clr);
    sio_reg!(oe_xor, gpio_oe_xor);
}

impl Pin<Sio<Output>> {
    pub fn set(&self) {
        self.out_set();
    }
    pub fn clr(&self) {
        self.out_clr();
    }
}

impl<M> Pin<Sio<Input<M>>> {
    pub fn schmitt_trigger(self) -> Pin<Sio<Input<SchmittTrigger>>> {
        self.pad_reg().write(|w| {
            w.schmitt();
            w
        });
        Pin {
            pin: self.pin,
            _func: PhantomData,
        }
    }

    pub fn pull_up(self) -> Pin<Sio<Input<PullUp>>> {
        self.pad_reg().write(|w| {
            w.pue();
            w
        });
        Pin {
            pin: self.pin,
            _func: PhantomData,
        }
    }

    pub fn pull_down(self) -> Pin<Sio<Input<PullDown>>> {
        self.pad_reg().write(|w| {
            w.pde();
            w
        });
        Pin {
            pin: self.pin,
            _func: PhantomData,
        }
    }

    pub fn is_set(&self) -> bool {
        self.sio_block().gpio_in.read().bits() & (1 << self.pin) != 0
    }
}
