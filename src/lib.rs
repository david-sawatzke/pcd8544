#![no_std]

extern crate embedded_hal as hal;

use hal::digital::OutputPin;

const WIDTH: u8 = 84;
const HEIGHT: u8 = 48;
const ROWS: u8 = HEIGHT / 8;

#[repr(u8)]
pub enum TemperatureCoefficient {
    TC0 = 0,
    TC1 = 1,
    TC2 = 2,
    TC3 = 3,
}

#[repr(u8)]
pub enum BiasMode {
    Bias1To100 = 0,
    Bias1To80 = 1,
    Bias1To65 = 2,
    Bias1To48 = 3,
    Bias1To40 = 4,
    Bias1To24 = 5,
    Bias1To18 = 6,
    Bias1To10 = 7,
}

#[repr(u8)]
pub enum DisplayMode {
    DisplayBlank = 0b000,
    NormalMode = 0b100,
    AllSegmentsOn = 0b001,
    InverseVideoMode = 0b101,
}

pub struct PCD8544<'a> {
    clk: &'a mut OutputPin,
    din: &'a mut OutputPin,
    dc: &'a mut OutputPin,
    ce: &'a mut OutputPin,
    rst: &'a mut OutputPin,
    light: &'a mut OutputPin,
    power_down_control: bool,
    entry_mode: bool,
    extended_instruction_set: bool,
    data: [[u8; 83]; 6],
}

impl<'a> PCD8544<'a> {
    pub fn new(
        clk: &'a mut OutputPin,
        din: &'a mut OutputPin,
        dc: &'a mut OutputPin,
        ce: &'a mut OutputPin,
        rst: &'a mut OutputPin,
        light: &'a mut OutputPin,
    ) -> PCD8544<'a> {
        clk.set_low();
        rst.set_low();
        ce.set_high();
        let data = [[0; 83]; 6];
        PCD8544 {
            clk,
            din,
            dc,
            ce,
            rst,
            light,
            power_down_control: false,
            entry_mode: false,
            extended_instruction_set: false,
            data,
        }
    }

    pub fn reset(&mut self) {
        self.rst.set_low();
        self.init();
    }

    pub fn init(&mut self) {
        // reset the display
        self.rst.set_low();
        self.rst.set_high();

        // reset state variables
        self.power_down_control = false;
        self.entry_mode = false;
        self.extended_instruction_set = false;

        // write init configuration
        self.enable_extended_commands(true);
        self.set_contrast(56_u8);
        self.set_temperature_coefficient(TemperatureCoefficient::TC3);
        self.set_bias_mode(BiasMode::Bias1To40);
        self.enable_extended_commands(false);
        self.set_display_mode(DisplayMode::NormalMode);

        // clear display data
        self.clear();
    }

    pub fn clear(&mut self) {
        for r in self.data.iter_mut() {
            for c in r.iter_mut() {
                *c = 0;
            }
        }
    }

    pub fn set_power_down(&mut self, power_down: bool) {
        self.power_down_control = power_down;
        self.write_current_function_set();
    }

    pub fn set_entry_mode(&mut self, entry_mode: bool) {
        self.entry_mode = entry_mode;
        self.write_current_function_set();
    }

    pub fn set_light(&mut self, enabled: bool) {
        if enabled {
            self.light.set_low();
        } else {
            self.light.set_high();
        }
    }

    pub fn set_display_mode(&mut self, mode: DisplayMode) {
        self.write_command(0x08 | mode as u8);
    }

    pub fn set_bias_mode(&mut self, bias: BiasMode) {
        self.write_command(0x10 | bias as u8)
    }

    pub fn set_temperature_coefficient(&mut self, coefficient: TemperatureCoefficient) {
        self.write_command(0x04 | coefficient as u8);
    }

    /// contrast in range of 0..128
    pub fn set_contrast(&mut self, contrast: u8) {
        self.write_command(0x80 | contrast);
    }

    pub fn enable_extended_commands(&mut self, enable: bool) {
        self.extended_instruction_set = enable;
        self.write_current_function_set();
    }

    fn write_current_function_set(&mut self) {
        let power = self.power_down_control;
        let entry = self.entry_mode;
        let extended = self.extended_instruction_set;
        self.write_function_set(power, entry, extended);
    }

    fn write_function_set(
        &mut self,
        power_down_control: bool,
        entry_mode: bool,
        extended_instruction_set: bool,
    ) {
        let mut val = 0x20;
        if power_down_control {
            val |= 0x04;
        }
        if entry_mode {
            val |= 0x02;
        }
        if extended_instruction_set {
            val |= 0x01;
        }
        self.write_command(val);
    }

    pub fn write_command(&mut self, value: u8) {
        self.write_byte(false, value);
    }

    pub fn write_data(&mut self, value: u8) {
        self.write_byte(true, value);
    }

    fn write_byte(&mut self, data: bool, value: u8) {
        let mut value = value;
        if data {
            self.dc.set_high();
        } else {
            self.dc.set_low();
        }
        self.ce.set_low();
        for _ in 0..8 {
            self.write_bit((value & 0x80) == 0x80);
            value <<= 1;
        }
        self.ce.set_high();
    }

    fn write_bit(&mut self, high: bool) {
        if high {
            self.din.set_high();
        } else {
            self.din.set_low();
        }
        self.clk.set_high();
        self.clk.set_low();
    }

    fn set_pixel(&mut self, x: u8, y: u8, on: bool) {
        let pixel = &mut self.data[(y / 8) as usize][x as usize];
        if on {
            *pixel |= 1 << (x % 8);
        } else {
            *pixel &= !(1 << (x % 8));
        }
    }

    pub fn flush(&mut self) {
        // Set the x pos to 0
        self.write_command(0x80);
        // Set the y pos to 0
        self.write_command(0x40);
        for r in 0..self.data.len() {
            for c in 0..self.data[r].len() {
                let data = self.data[r][c];
                self.write_data(data);
            }
        }
    }
}

extern crate embedded_graphics;
use self::embedded_graphics::{drawable, pixelcolor::PixelColorU8, Drawing};
impl Drawing<PixelColorU8> for PCD8544<'_> {
    fn draw<T>(&mut self, item_pixels: T)
    where
        T: Iterator<Item = drawable::Pixel<PixelColorU8>>,
    {
        for pixel in item_pixels {
            let x = (pixel.0).0 as u8;
            let y = (pixel.0).1 as u8;
            if x >= WIDTH || y >= HEIGHT {
                continue;
            }
            self.set_pixel(
                x,
                y,
                if pixel.1.into_inner() > 128 {
                    true
                } else {
                    false
                },
            );
        }
    }
}
