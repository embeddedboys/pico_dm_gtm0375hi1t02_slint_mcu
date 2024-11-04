#![no_std]
#![no_main]

#[cfg(not(feature = "simulator"))]
use cortex_m::delay::Delay;
// use defmt::info;
#[cfg(not(feature = "simulator"))]
use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
#[cfg(not(feature = "simulator"))]
use embedded_graphics::{pixelcolor::Rgb565, prelude::IntoStorage};
#[cfg(not(feature = "simulator"))]
use embedded_hal::digital::OutputPin;
#[cfg(not(feature = "simulator"))]
use rp2040_hal::pio::{Tx, ValidStateMachine};

mod graphics;
pub mod overclock;

#[cfg(not(feature = "simulator"))]
type Result<T = ()> = core::result::Result<T, DisplayError>;

#[cfg(not(feature = "simulator"))]
pub struct Pio8BitBus<SM: ValidStateMachine, DC> {
    tx: Tx<SM>,
    dc: DC,
}

#[cfg(not(feature = "simulator"))]
impl<TX, DC> Pio8BitBus<TX, DC>
where
    TX: ValidStateMachine,
    DC: OutputPin,
{
    pub fn new(tx: Tx<TX>, dc: DC) -> Self {
        Self { tx, dc }
    }

    fn write_iter(&mut self, iter: impl Iterator<Item = u8>) -> Result {
        for value in iter {
            self.tx.write(value as u32);
            while !self.tx.is_empty() {}
        }
        Ok(())
    }

    fn write_pairs(&mut self, iter: impl Iterator<Item = [u8; 2]>) -> Result {
        use core::iter::once;
        self.write_iter(iter.flat_map(|[first, second]| once(first).chain(once(second))))
    }

    pub fn write_data(&mut self, data: DataFormat<'_>) -> Result {
        match data {
            DataFormat::U8(slice) => self.write_iter(slice.iter().copied()),
            DataFormat::U16LEIter(iter) => self.write_pairs(iter.map(u16::to_be_bytes)),
            _ => Err(DisplayError::DataFormatNotImplemented),
        }
    }
}

#[cfg(not(feature = "simulator"))]
impl<TX, DC> WriteOnlyDataCommand for Pio8BitBus<TX, DC>
where
    TX: ValidStateMachine,
    DC: OutputPin,
{
    fn send_commands(&mut self, cmd: DataFormat<'_>) -> Result {
        self.dc.set_low().map_err(|_| DisplayError::DCError)?;
        self.write_data(cmd)?;
        Ok(())
    }
    fn send_data(&mut self, buf: DataFormat<'_>) -> Result {
        self.dc.set_high().map_err(|_| DisplayError::DCError)?;
        self.write_data(buf)?;
        Ok(())
    }
}

#[cfg(not(feature = "simulator"))]
pub struct ILI9488<DI, RST, BL>
where
    DI: WriteOnlyDataCommand,
    RST: OutputPin,
    BL: OutputPin,
{
    di: DI,
    rst: Option<RST>,
    bl: Option<BL>,

    size_x: u16,
    size_y: u16,
}

#[cfg(not(feature = "simulator"))]
impl<DI, RST, BL> ILI9488<DI, RST, BL>
where
    DI: WriteOnlyDataCommand,
    RST: OutputPin,
    BL: OutputPin,
{
    pub fn new(di: DI, rst: Option<RST>, bl: Option<BL>, size_x: u16, size_y: u16) -> Self {
        Self {
            di,
            rst,
            bl,
            size_x,
            size_y,
        }
    }

    pub fn init_test(&mut self) -> Result {
        self.write_command(0x55)?;
        Ok(())
    }

    pub fn init(&mut self, delay_source: &mut Delay) -> Result {
        self.hard_reset(delay_source);

        if let Some(bl) = self.bl.as_mut() {
            bl.set_high().unwrap();
        }

        // Positive Gamma Control
        self.write_reg(&[0xE0, 0x00, 0x03, 0x09, 0x08, 0x16, 0x0A, 0x3F, 0x78, 0x4C, 0x09, 0x0A, 0x08, 0x16, 0x1A, 0x0F])?;

        // Negative Gamma Control
        self.write_reg(&[0xE1, 0x00, 0x16, 0x19, 0x03, 0x0F, 0x05, 0x32, 0x45, 0x46, 0x04, 0x0E, 0x0D, 0x35, 0x37, 0x0F])?;

        self.write_reg(&[0xC0, 0x17, 0x15])?;          // Power Control 1
        self.write_reg(&[0xC1, 0x41])?;                // Power Control 2
        self.write_reg(&[0xC5, 0x00, 0x12, 0x80])?;    // VCOM Control
        self.write_reg(&[0x36, 0x28])?;                // Memory Access Control
        self.write_reg(&[0x3A, 0x55])?;                // Pixel Interface Format RGB565 8080 16-bit
        self.write_reg(&[0xB0, 0x00])?;                // Interface Mode Control

        // Frame Rate Control
        // self.write_reg(&[0xB1, 0xD0, 0x11);              // 60Hz
        self.write_reg(&[0xB1, 0xD0, 0x14])?;          // 90Hz

        self.write_reg(&[0xB4, 0x02])?;                // Display Inversion Control
        self.write_reg(&[0xB6, 0x02, 0x02, 0x3B])?;    // Display Function Control
        self.write_reg(&[0xB7, 0xC6])?;                // Entry Mode Set
        self.write_reg(&[0xF7, 0xA9, 0x51, 0x2C, 0x82])?;  // Adjust Control 3
        self.write_reg(&[0x11])?;                      // Exit Sleep
        delay_source.delay_ms(60);
        self.write_reg(&[0x29])?;                      // Display on

        Ok(())
    }

    pub fn hard_reset(&mut self, delay_source: &mut Delay) {
        if let Some(rst) = self.rst.as_mut() {
            rst.set_high().unwrap();
            delay_source.delay_ms(10);
            rst.set_low().unwrap();
            delay_source.delay_ms(10);
            rst.set_high().unwrap();
            delay_source.delay_ms(10);
        }
    }

    pub fn set_addr_win(&mut self, xs: u16, ys: u16, xe: u16, ye: u16) -> Result {
        self.write_reg(&[
            0x2A,
            (xs >> 8) as u8,
            (xs) as u8,
            (xe >> 8) as u8,
            (xe) as u8,
        ])?;

        self.write_reg(&[
            0x2B,
            (ys >> 8) as u8,
            (ys) as u8,
            (ye >> 8) as u8,
            (ye) as u8,
        ])?;

        self.write_reg(&[0x2C])?;
        Ok(())
    }

    pub fn write_pixels<I>(&mut self, colors: I) -> Result
    where
        I: IntoIterator<Item = Rgb565>,
    {
        let mut iter = colors.into_iter().map(|c| c.into_storage());
        let buf = DataFormat::U16LEIter(&mut iter);
        self.di.send_data(buf)?;
        Ok(())
    }

    pub fn write_command(&mut self, cmd: u8) -> Result {
        self.di.send_commands(DataFormat::U8(&[cmd]))?;
        Ok(())
    }

    pub fn write_data(&mut self, buf: u8) -> Result {
        self.di.send_data(DataFormat::U8(&[buf]))?;
        Ok(())
    }

    pub fn write_reg(&mut self, seq: &[u8]) -> Result {
        self.write_command(seq[0])?;
        for val in &seq[1..] {
            self.write_data(*val)?;
        }
        Ok(())
    }
}
