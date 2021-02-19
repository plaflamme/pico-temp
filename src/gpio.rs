use core::marker::PhantomData;
use rp2040_pac::sio::RegisterBlock;
use rp2040_pac::generic::{Reg, Writable, ResetValue};

pub struct Pin<F> {
    pin: u8,
    _func: PhantomData<F>,
}
pub struct Unspecified;
pub struct Sio<IO> {
    _io: PhantomData<IO>,
}

pub struct Input;
pub struct Output;

impl Pin<()> {

    pub fn new(pin: u8) -> Pin<Unspecified> {
        Pin { pin, _func: PhantomData }
    }

}

impl<F> Pin<F> {

    fn sio_block(&self) -> &rp2040_pac::sio::RegisterBlock {
        unsafe { &* rp2040_pac::SIO::ptr() }
    }

    fn pad_block(&self) -> &rp2040_pac::pads_bank0::GPIO {
        let pad_block = unsafe { &* rp2040_pac::PADS_BANK0::ptr() };
        &pad_block.gpio[self.pin as usize]
    }

    fn io_block(&self) -> &rp2040_pac::io_bank0::GPIO {
        let io_block = unsafe { &* rp2040_pac::IO_BANK0::ptr() };
        &io_block.gpio[self.pin as usize]
    }

    fn set_bit<REG>(&self, reg: &Reg<u32, REG>)
    where
        Reg<u32, REG>: Writable + ResetValue<Type=u32>
    {
        reg.write(|w| unsafe {
            w.bits(1 << self.pin);
            w
        });
    }
}

impl Pin<Unspecified> {

    pub fn func_sio(self) -> Pin<Sio<Unspecified>> {
        self.io_block().gpio_ctrl.write(|w| {
            w.funcsel().sio_0();
            w
        });

        // TODO: not sure why we do this?
        self.pad_block().write(|w| {
            w.od().clear_bit();
            w.ie().set_bit();
            w
        });

        // TODO: not sure why we do this? I think this sets the pin to 0?
        self.set_bit(&self.sio_block().gpio_oe_clr);
        self.set_bit(&self.sio_block().gpio_out_clr);

        Pin { pin: self.pin, _func: PhantomData }
    }
}

impl<IO> Pin<Sio<IO>> {
    pub fn input(self) -> Pin<Sio<Input>> {
        self.set_bit(&self.sio_block().gpio_oe_clr);
        Pin { pin: self.pin, _func: PhantomData }
    }

    pub fn output(self) -> Pin<Sio<Output>> {
        self.set_bit(&self.sio_block().gpio_oe_set);
        Pin { pin: self.pin, _func: PhantomData }
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
