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
}

impl Pin<Unspecified> {
    pub fn func_sio(self) -> Pin<Sio<Unspecified>> {
        // pico-sdk gpio_init
        self.set_bit(&self.sio_block().gpio_oe_clr);
        self.set_bit(&self.sio_block().gpio_out_clr);

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

        Pin {
            pin: self.pin,
            _func: PhantomData,
        }
    }
}

impl<IO> Pin<Sio<IO>> {
    pub fn input(self) -> Pin<Sio<Input<Unspecified>>> {
        self.set_bit(&self.sio_block().gpio_oe_clr);
        Pin {
            pin: self.pin,
            _func: PhantomData,
        }
    }

    pub fn output(self) -> Pin<Sio<Output>> {
        self.set_bit(&self.sio_block().gpio_oe_set);
        Pin {
            pin: self.pin,
            _func: PhantomData,
        }
    }

    fn set_sio_bit<'a, F, REG: 'a>(&'a self, f: F)
    where
        F: FnOnce(&'a rp2040_pac::sio::RegisterBlock) -> &'a Reg<u32, REG>,
        Reg<u32, REG>: Writable + ResetValue<Type = u32>,
    {
        self.set_bit(f(self.sio_block()));
    }

    fn set_out_set(&self) {
        self.set_sio_bit(|b| &b.gpio_out_set)
    }

    fn set_out_clr(&self) {
        self.set_sio_bit(|b| &b.gpio_out_clr)
    }
}

impl Pin<Sio<Output>> {
    pub fn set(&self) {
        self.set_bit(&self.sio_block().gpio_out_set);
    }
    pub fn clr(&self) {
        self.set_bit(&self.sio_block().gpio_out_clr);
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
