use crate::gpio::{Input, Output, Pin, Sio};
use cortex_m::delay::Delay;
use cortex_m::prelude::*;

macro_rules! wait_for_state {
    ($pin: ident, $delay: ident, $state: expr, $timeout: expr) => {{
        let mut timeout = $timeout;
        loop {
            if $pin.is_set() == $state {
                break Ok(());
            } else {
                $delay.delay_us(1);
                timeout -= 1;
                if timeout <= 0 {
                    break Err(DhtError::Timeout);
                }
            }
        }
    }};
}

fn read_bit<M>(pin: &Pin<Sio<Input<M>>>, delay: &mut Delay) -> Result<bool, DhtError> {
    // Every bit starts with th DHT pulling the line low for 50us, then
    //   0: pulled up for ~26-28us
    //   1: pulled up for 70us
    wait_for_state!(pin, delay, true, 80)?;
    delay.delay_us(35);
    let value = pin.is_set();
    wait_for_state!(pin, delay, false, 50)?;
    Ok(value)
}

fn read_byte<M>(pin: &Pin<Sio<Input<M>>>, delay: &mut Delay) -> Result<u8, DhtError> {
    let mut byte = 0u8;
    for bit in 7..=0 {
        if read_bit(pin, delay)? {
            byte |= 1 << bit;
        }
    }
    Ok(byte)
}

fn read_loop<M>(dht_pin: &Pin<Sio<Input<M>>>, delay: &mut Delay) -> Result<Reading, DhtError> {
    // The DHT keeps it down for ~80us and then pulls up
    wait_for_state!(dht_pin, delay, true, 100)?;
    // The DHT keeps it up for ~80us and then pulls down for the first bit
    wait_for_state!(dht_pin, delay, false, 100)?;

    let mut values = [0u8; 4];
    for byte in values.iter_mut() {
        *byte = read_byte(&dht_pin, delay)?;
    }
    let cksum = read_byte(&dht_pin, delay)?;

    let sum = (values.iter().fold(0u32, |a, &b| a + b as u32) & 0xff) as u8;
    if sum != cksum {
        Err(DhtError::Checksum)
    } else {
        Ok(Reading::from(values))
    }
}

pub struct Dht {
    pin: Option<Pin<Sio<Output>>>,
}

impl Dht {
    pub fn new<M>(pin: Pin<Sio<M>>) -> Self {
        Dht {
            pin: Some(pin.output()),
        }
    }

    pub fn read(
        &mut self,
        led_pin: &Pin<Sio<Output>>,
        delay: &mut Delay,
    ) -> Result<Reading, DhtError> {

        led_pin.set();

        let dht_pin = self.pin.take().unwrap();

        // The start signal is pull down, wait > 18ms, pull up, switch to input mode
        dht_pin.clr();
        delay.delay_ms(20);

        dht_pin.set();
        let dht_pin = dht_pin.input();
        // The DHT should pull down in the next 20-40us
        delay.delay_us(40);

        let result = read_loop(&dht_pin, delay);

        led_pin.clr();

        self.pin = Some(dht_pin.output());

        result
    }
}

pub struct Reading {
    pub relative_humdity: u8,
    pub temperature: f32,
}

impl From<[u8; 4]> for Reading {
    fn from(values: [u8; 4]) -> Self {
        // it seems as though the decimal part of the relative humidity is always 0 on my unit.
        let [rh_i, _, temp_i, temp_d] = values;

        // The way to interpret the 2 values is not really documented.
        // This way is the most common one I've seen in other source files.
        let temperature = temp_i as f32 + (temp_d as f32 / 10.0);
        Reading {
            relative_humdity: rh_i,
            temperature,
        }
    }
}

pub enum DhtError {
    Timeout,
    Checksum,
}
