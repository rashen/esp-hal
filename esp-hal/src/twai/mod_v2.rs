#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # Two-wire Automotive Interface (TWAI)
//!
//! ## Overview
//!
//! The TWAI is a multi-master, multi-cast communication protocol with error
//! detection and signaling and inbuilt message priorities and arbitration. The
//! TWAI protocol is suited for automotive and industrial applications.
//!
//! See ESP-IDF's
#![doc = concat!("[TWAI documentation](https://docs.espressif.com/projects/esp-idf/en/latest/", chip!(), "/api-reference/peripherals/twai.html#twai-protocol-summary)")]
//! for a summary on the protocol.
//!
//! ## Configuration
//! The driver  offers functions for initializing the TWAI peripheral, setting
//! up the timing parameters, configuring acceptance filters, handling
//! interrupts, and transmitting/receiving messages on the TWAI bus.
//!
//! This driver manages the ISO 11898-1 compatible TWAI
//! controllers. It supports Standard Frame Format (11-bit) and Extended Frame
//! Format (29-bit) frame identifiers.
//!
//! ## Examples
//!
//! ### Transmitting and Receiving Messages
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::twai;
//! # use esp_hal::twai::filter;
//! # use esp_hal::twai::filter::SingleStandardFilter;
//! # use esp_hal::twai::TwaiConfiguration;
//! # use esp_hal::twai::BaudRate;
//! # use esp_hal::twai::TwaiMode;
//! # use nb::block;
//! // Use GPIO pins 2 and 3 to connect to the respective pins on the TWAI
//! // transceiver.
//! let twai_rx_pin = peripherals.GPIO3;
//! let twai_tx_pin = peripherals.GPIO2;
//!
//! // The speed of the TWAI bus.
//! const TWAI_BAUDRATE: twai::BaudRate = BaudRate::B1000K;
//!
//! // Begin configuring the TWAI peripheral. The peripheral is in a reset like
//! // state that prevents transmission but allows configuration.
//! let mut twai_config = twai::TwaiConfiguration::new(
//!     peripherals.TWAI0,
//!     twai_rx_pin,
//!     twai_tx_pin,
//!     TWAI_BAUDRATE,
//!     TwaiMode::Normal,
//! );
//!
//! // Partially filter the incoming messages to reduce overhead of receiving
//! // undesired messages
//! twai_config.set_filter(
//!     const { SingleStandardFilter::new(b"xxxxxxxxxx0", b"x", [b"xxxxxxxx", b"xxxxxxxx"]) },
//! );
//!
//! // Start the peripheral. This locks the configuration settings of the
//! // peripheral and puts it into operation mode, allowing packets to be sent
//! // and received.
//! let mut twai = twai_config.start();
//!
//! loop {
//!     // Wait for a frame to be received.
//!     let frame = block!(twai.receive())?;
//!
//!     // Transmit the frame back.
//!     let _result = block!(twai.transmit(&frame))?;
//! }
//! # }
//! ```
//!
//! ### Self-testing (self reception of transmitted messages)
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::twai;
//! # use esp_hal::twai::filter;
//! # use esp_hal::twai::filter::SingleStandardFilter;
//! # use esp_hal::twai::TwaiConfiguration;
//! # use esp_hal::twai::BaudRate;
//! # use esp_hal::twai::EspTwaiFrame;
//! # use esp_hal::twai::StandardId;
//! # use esp_hal::twai::TwaiMode;
//! # use nb::block;
//! // Use GPIO pins 2 and 3 to connect to the respective pins on the TWAI
//! // transceiver.
//! let can_rx_pin = peripherals.GPIO3;
//! let can_tx_pin = peripherals.GPIO2;
//!
//! // The speed of the TWAI bus.
//! const TWAI_BAUDRATE: twai::BaudRate = BaudRate::B1000K;
//!
//! // Begin configuring the TWAI peripheral.
//! let mut can_config = twai::TwaiConfiguration::new(
//!     peripherals.TWAI0,
//!     can_rx_pin,
//!     can_tx_pin,
//!     TWAI_BAUDRATE,
//!     TwaiMode::SelfTest
//! );
//!
//! // Partially filter the incoming messages to reduce overhead of receiving
//! // undesired messages
//! can_config.set_filter(const { SingleStandardFilter::new(b"xxxxxxxxxx0",
//! b"x", [b"xxxxxxxx", b"xxxxxxxx"]) });
//!
//! // Start the peripheral. This locks the configuration settings of the
//! // peripheral and puts it into operation mode, allowing packets to be sent
//! // and received.
//! let mut can = can_config.start();
//!
//! # // TODO: `new_*` should return Result not Option
//! let frame = EspTwaiFrame::new_self_reception(StandardId::ZERO,
//!     &[1, 2, 3]).unwrap(); // Wait for a frame to be received.
//! let frame = block!(can.receive())?;
//!
//! # loop {}
//! # }
//! ```

use core::marker::PhantomData;
#[cfg(esp32c5)]
use core::ptr;

use enumset::{EnumSet, EnumSetType};
use procmacros::handler;

use self::filter::{Filter, FilterType};
use crate::{
    Async, Blocking, DriverMode,
    gpio::{
        DriveMode, InputConfig, InputSignal, OutputConfig, OutputSignal, Pull,
        interconnect::{PeripheralInput, PeripheralOutput},
    },
    interrupt::InterruptHandler,
    pac::twai0::RegisterBlock,
    peripherals::PCR,
    system::{Cpu, PeripheralGuard},
};
pub mod filter;

mod reg {
    pub const TWAI0_FD_BASE: *mut u8 = esp32c5::TWAI0::ptr() as *mut u8;
    pub const TWAI_TX_BUF1: isize = 0x100;
    pub const TWAI_TX_BUF2: isize = 0x200;
    pub const TWAI_TX_BUF3: isize = 0x300;
    pub const TWAI_TX_BUF4: isize = 0x400;
}

/// TWAI error kind
///
/// This represents a common set of TWAI operation errors. HAL implementations
/// are free to define more specific or additional error types. However, by
/// providing a mapping to these common TWAI errors, generic code can still
/// react to them.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
#[non_exhaustive]
pub enum ErrorKind {
    /// The peripheral receive buffer was overrun.
    Overrun,
    // MAC sublayer errors
    /// A bit error is detected at that bit time when the bit value that is
    /// monitored differs from the bit value sent.
    Bit,
    /// A stuff error is detected at the bit time of the sixth consecutive
    /// equal bit level in a frame field that shall be coded by the method
    /// of bit stuffing.
    Stuff,
    /// Calculated CRC sequence does not equal the received one.
    Crc,
    /// A form error shall be detected when a fixed-form bit field contains
    /// one or more illegal bits.
    Form,
    /// An ACK  error shall be detected by a transmitter whenever it does not
    /// monitor a dominant bit during the ACK slot.
    Acknowledge,
    /// A different error occurred. The original error may contain more
    /// information.
    Other,
}

macro_rules! impl_display {
    ($($kind:ident => $msg:expr),* $(,)?) => {
        impl core::fmt::Display for ErrorKind {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                match self {
                    $(Self::$kind => write!(f, $msg)),*
                }
            }
        }

        #[cfg(feature = "defmt")]
        impl defmt::Format for ErrorKind {
            fn format(&self, f: defmt::Formatter<'_>) {
                match self {
                    $(Self::$kind => defmt::write!(f, $msg)),*
                }
            }
        }
    };
}

impl_display! {
    Overrun => "The peripheral receive buffer was overrun",
    Bit => "Bit value that is monitored differs from the bit value sent",
    Stuff => "Sixth consecutive equal bits detected",
    Crc => "Calculated CRC sequence does not equal the received one",
    Form => "A fixed-form bit field contains one or more illegal bits",
    Acknowledge => "Transmitted frame was not acknowledged",
    Other => "A different error occurred. The original error may contain more information",
}

#[instability::unstable]
impl From<ErrorKind> for embedded_can::ErrorKind {
    fn from(value: ErrorKind) -> Self {
        match value {
            ErrorKind::Overrun => embedded_can::ErrorKind::Overrun,
            ErrorKind::Bit => embedded_can::ErrorKind::Bit,
            ErrorKind::Stuff => embedded_can::ErrorKind::Stuff,
            ErrorKind::Crc => embedded_can::ErrorKind::Crc,
            ErrorKind::Form => embedded_can::ErrorKind::Form,
            ErrorKind::Acknowledge => embedded_can::ErrorKind::Acknowledge,
            ErrorKind::Other => embedded_can::ErrorKind::Other,
        }
    }
}

#[instability::unstable]
impl embedded_can::Error for ErrorKind {
    fn kind(&self) -> embedded_can::ErrorKind {
        (*self).into()
    }
}

/// Specifies in which mode the TWAI controller will operate.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TwaiMode {
    /// Self-test mode (no acknowledgement required for a successful message
    pub self_test: bool,
    /// Listen only operating mode
    pub listen_only: bool,
    /// Write from memory to memory
    pub loopback: bool,
}

impl TwaiMode {
    const fn normal() -> Self {
        Self {
            self_test: false,
            listen_only: false,
            loopback: false,
        }
    }
    const fn self_test() -> Self {
        Self {
            self_test: true,
            listen_only: false,
            loopback: false,
        }
    }
    const fn listen_only() -> Self {
        Self {
            self_test: false,
            listen_only: true,
            loopback: false,
        }
    }
}

/// Standard 11-bit TWAI Identifier (`0..=0x7FF`).
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StandardId(u16);

impl StandardId {
    /// TWAI ID `0`, the highest priority.
    pub const ZERO: Self = StandardId(0);

    /// TWAI ID `0x7FF`, the lowest priority.
    pub const MAX: Self = StandardId(0x7FF);

    /// Tries to create a `StandardId` from a raw 16-bit integer.
    ///
    /// This will return `None` if `raw` is out of range of an 11-bit integer
    /// (`> 0x7FF`).
    #[inline]
    pub fn new(raw: u16) -> Option<Self> {
        if raw <= 0x7FF {
            Some(StandardId(raw))
        } else {
            None
        }
    }

    /// Creates a new `StandardId` without checking if it is inside the valid
    /// range.
    ///
    /// # Safety
    /// Using this method can create an invalid ID and is thus marked as unsafe.
    #[inline]
    pub const unsafe fn new_unchecked(raw: u16) -> Self {
        StandardId(raw)
    }

    /// Returns TWAI Identifier as a raw 16-bit integer.
    #[inline]
    pub fn as_raw(&self) -> u16 {
        self.0
    }
}

#[instability::unstable]
impl From<StandardId> for embedded_can::StandardId {
    fn from(value: StandardId) -> Self {
        embedded_can::StandardId::new(value.as_raw()).unwrap()
    }
}

#[instability::unstable]
impl From<embedded_can::StandardId> for StandardId {
    fn from(value: embedded_can::StandardId) -> Self {
        StandardId::new(value.as_raw()).unwrap()
    }
}

/// Extended 29-bit TWAI Identifier (`0..=1FFF_FFFF`).
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ExtendedId(u32);

impl ExtendedId {
    /// TWAI ID `0`, the highest priority.
    pub const ZERO: Self = ExtendedId(0);

    /// TWAI ID `0x1FFFFFFF`, the lowest priority.
    pub const MAX: Self = ExtendedId(0x1FFF_FFFF);

    /// Tries to create a `ExtendedId` from a raw 32-bit integer.
    ///
    /// This will return `None` if `raw` is out of range of an 29-bit integer
    /// (`> 0x1FFF_FFFF`).
    #[inline]
    pub fn new(raw: u32) -> Option<Self> {
        if raw <= 0x1FFF_FFFF {
            Some(ExtendedId(raw))
        } else {
            None
        }
    }

    /// Creates a new `ExtendedId` without checking if it is inside the valid
    /// range.
    ///
    /// # Safety
    /// Using this method can create an invalid ID and is thus marked as unsafe.
    #[inline]
    pub const unsafe fn new_unchecked(raw: u32) -> Self {
        ExtendedId(raw)
    }

    /// Returns TWAI Identifier as a raw 32-bit integer.
    #[inline]
    pub fn as_raw(&self) -> u32 {
        self.0
    }

    /// Returns the Base ID part of this extended identifier.
    pub fn standard_id(&self) -> StandardId {
        // ID-28 to ID-18
        StandardId((self.0 >> 18) as u16)
    }
}

#[instability::unstable]
impl From<ExtendedId> for embedded_can::ExtendedId {
    fn from(value: ExtendedId) -> Self {
        embedded_can::ExtendedId::new(value.0).unwrap()
    }
}

#[instability::unstable]
impl From<embedded_can::ExtendedId> for ExtendedId {
    fn from(value: embedded_can::ExtendedId) -> Self {
        ExtendedId::new(value.as_raw()).unwrap()
    }
}

/// A TWAI Identifier (standard or extended).
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Id {
    /// Standard 11-bit Identifier (`0..=0x7FF`).
    Standard(StandardId),
    /// Extended 29-bit Identifier (`0..=0x1FFF_FFFF`).
    Extended(ExtendedId),
}

impl From<StandardId> for Id {
    #[inline]
    fn from(id: StandardId) -> Self {
        Id::Standard(id)
    }
}

impl From<ExtendedId> for Id {
    #[inline]
    fn from(id: ExtendedId) -> Self {
        Id::Extended(id)
    }
}

#[instability::unstable]
impl From<Id> for embedded_can::Id {
    fn from(value: Id) -> Self {
        match value {
            Id::Standard(id) => embedded_can::Id::Standard(id.into()),
            Id::Extended(id) => embedded_can::Id::Extended(id.into()),
        }
    }
}

#[instability::unstable]
impl From<embedded_can::Id> for Id {
    fn from(value: embedded_can::Id) -> Self {
        match value {
            embedded_can::Id::Standard(id) => Id::Standard(id.into()),
            embedded_can::Id::Extended(id) => Id::Extended(id.into()),
        }
    }
}

const MAX_DATA_LEN: usize = 64;

/// A TWAI Frame.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EspTwaiFrame {
    id: Id,
    dlc: usize,
    data: [u8; MAX_DATA_LEN],
    is_remote: bool,
    self_reception: bool,
}

impl EspTwaiFrame {
    /// Creates a new `EspTwaiFrame` with the specified ID and data payload.
    pub fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        // TWAI frames cannot contain more than 8 bytes of data.
        if data.len() > MAX_DATA_LEN {
            return None;
        }

        let mut d: [u8; MAX_DATA_LEN] = [0; _];
        d[..data.len()].copy_from_slice(data);

        Some(EspTwaiFrame {
            id: id.into(),
            data: d,
            dlc: data.len(),
            is_remote: false,
            self_reception: false,
        })
    }

    /// Creates a new `EspTwaiFrame` for a transmission request with the
    /// specified ID and data length (DLC).
    pub fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self> {
        // TWAI frames cannot have more than 8 bytes.
        if dlc > MAX_DATA_LEN {
            return None;
        }

        Some(EspTwaiFrame {
            id: id.into(),
            data: [0; MAX_DATA_LEN],
            dlc,
            is_remote: true,
            self_reception: false,
        })
    }

    /// Creates a new `EspTwaiFrame` ready for self-reception with the specified
    /// ID and data payload.
    pub fn new_self_reception(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        if data.len() > 64 {
            return None;
        }

        let mut d: [u8; 64] = [0; _];
        d[..data.len()].copy_from_slice(data);

        Some(EspTwaiFrame {
            id: id.into(),
            data: d,
            dlc: data.len(),
            is_remote: false,
            self_reception: true,
        })
    }

    /// Extracts sender ID from `EspTwaiFrame`
    pub fn id(&self) -> Id {
        self.id
    }

    /// Extracts the data from the `EspTwaiFrame`
    pub fn data(&self) -> &[u8] {
        &self.data[..self.dlc]
    }
}

#[instability::unstable]
impl embedded_can::Frame for EspTwaiFrame {
    fn new(id: impl Into<embedded_can::Id>, data: &[u8]) -> Option<Self> {
        Self::new(id.into(), data)
    }

    fn new_remote(id: impl Into<embedded_can::Id>, dlc: usize) -> Option<Self> {
        Self::new_remote(id.into(), dlc)
    }

    fn is_extended(&self) -> bool {
        matches!(self.id, Id::Extended(_))
    }

    fn is_remote_frame(&self) -> bool {
        self.is_remote
    }

    fn id(&self) -> embedded_can::Id {
        self.id.into()
    }

    fn dlc(&self) -> usize {
        self.dlc
    }

    fn data(&self) -> &[u8] {
        // Remote frames do not contain data, yet have a value for the dlc so return
        // an empty slice for remote frames.
        match self.is_remote {
            true => &[],
            false => &self.data[0..self.dlc],
        }
    }
}

/// The underlying timings for the TWAI peripheral.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TimingConfig {
    /// The baudrate prescaler is used to determine the period of each time
    /// quantum by dividing the TWAI controller's source clock.
    pub baud_rate_prescaler: u8,

    /// The synchronization jump width is used to determine the maximum number
    /// of time quanta a single bit time can be lengthened/shortened for
    /// synchronization purposes.
    pub sync_jump_width: u8,

    /// Timing segment 1 consists of 1 to 16 time quanta before sample point.
    pub tseg_1: u8,

    /// Timing Segment 2 consists of 1 to 8 time quanta after sample point.
    pub tseg_2: u8,

    /// Propagation segment
    pub prop: u8,
}

/// A selection of pre-determined baudrates for the TWAI driver.
/// Currently these timings are sourced from the ESP IDF C driver which assumes
/// an APB clock of 80MHz.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BaudRate {
    /// A baud rate of 125 Kbps.
    B125K,
    /// A baud rate of 250 Kbps.
    B250K,
    /// A baud rate of 500 Kbps.
    B500K,
    /// A baud rate of 1 Mbps.
    B1000K,
    /// A custom baud rate defined by the user.
    ///
    /// This variant allows users to specify their own timing configuration
    /// using a `TimingConfig` struct.
    Custom(TimingConfig),
}

impl BaudRate {
    /// Convert the BaudRate into the timings that the peripheral needs.
    // See: https://github.com/espressif/esp-idf/tree/ab4200e/components/esp_hal_twai/include/hal/twai_types.h
    const fn timing(self) -> TimingConfig {
        // Clock source is assumed to be 80 MHz
        let timing = match self {
            Self::B125K => TimingConfig {
                baud_rate_prescaler: 8,
                sync_jump_width: 8,
                tseg_1: 48,
                tseg_2: 16,
                prop: 15,
            },
            Self::B250K => TimingConfig {
                baud_rate_prescaler: 4,
                sync_jump_width: 8,
                tseg_1: 48,
                tseg_2: 16,
                prop: 15,
            },
            Self::B500K => TimingConfig {
                baud_rate_prescaler: 2,
                sync_jump_width: 8,
                tseg_1: 48,
                tseg_2: 16,
                prop: 15,
            },
            Self::B1000K => TimingConfig {
                baud_rate_prescaler: 1,
                sync_jump_width: 10,
                tseg_1: 45,
                tseg_2: 20,
                prop: 14,
            },
            Self::Custom(timing_config) => timing_config,
        };

        timing
    }
}

/// An inactive TWAI peripheral in the "Reset"/configuration state.
pub struct TwaiConfiguration<'d, Dm: DriverMode> {
    twai: AnyTwai<'d>,
    filter: Option<(FilterType, [u8; 8])>,
    phantom: PhantomData<Dm>,
    mode: TwaiMode,
    initialized: bool,
    _guard: PeripheralGuard,
}

impl<'d, Dm> TwaiConfiguration<'d, Dm>
where
    Dm: DriverMode,
{
    fn new_internal(
        twai: AnyTwai<'d>,
        rx_pin: impl PeripheralInput<'d>,
        tx_pin: impl PeripheralOutput<'d>,
        baud_rate: BaudRate,
        no_transceiver: bool,
        mode: TwaiMode,
    ) -> Self {
        // Set clock source to PLL_F80M
        let _ = PCR::regs()
            .twai0_func_clk_conf()
            .modify(|_, w| w.twai0_func_clk_sel().set_bit());

        // Enable twai clock
        let _ = PCR::regs()
            .twai0_conf()
            .modify(|_, w| w.twai0_clk_en().set_bit());

        // Reset twai clock
        let _ = PCR::regs()
            .twai0_conf()
            .modify(|_, w| w.twai0_rst_en().set_bit());
        let _ = PCR::regs()
            .twai0_conf()
            .modify(|_, w| w.twai0_rst_en().clear_bit());

        // Enable function clock
        let _ = PCR::regs()
            .twai0_func_clk_conf()
            .modify(|_, w| w.twai0_func_clk_en().set_bit());

        // Wait for ready
        while PCR::regs().twai0_conf().read().twai0_ready().bit_is_clear() {
            core::hint::spin_loop();
        }

        let rx_pin = rx_pin.into();
        let tx_pin = tx_pin.into();

        let guard = PeripheralGuard::new(twai.peripheral());

        let mut this = TwaiConfiguration {
            twai,
            filter: None, // We'll immediately call `set_filter`
            phantom: PhantomData,
            mode,
            initialized: false,
            _guard: guard,
        };

        let retransmission_limit = 0;
        let drop_rtr_frames = true;

        this.regs().mode_settings().write(|w| {
            w.ena().clear_bit(); // Disable
            w.rst().set_bit(); // Reset
            w.rtrle().bit(retransmission_limit > 0); // Retransmission
            unsafe { w.rtrth().bits(retransmission_limit) };
            w.fdrf().bit(drop_rtr_frames); // Drop RTR frames
            w.afm().set_bit(); // Enable filter mode
            w.fde().set_bit(); // Flexible bit rate between nominal and data field
            w.tbfbo().set_bit(); // Treat bus-off as TX failure
            w.rxbam().set_bit(); // Enable RX FIFO automatic increase
            w
        });

        this.regs().rx_status_rx_settings().write(|w| {
            w.rtsop().clear_bit(); // Set sample point for timestamp
            w
        });

        let int_mask = this.regs().int_ena_set().write(|w| {
            w.rxi_int_ena_mask().set_bit();
            w.txi_int_ena_mask().set_bit();
            w.ewli_int_ena_mask().set_bit();
            w.ali_int_ena_mask().set_bit();
            w.doi_int_ena_mask().set_bit();
            w.bsi_int_ena_mask().set_bit();
            w.txbhci_int_ena_mask().set_bit();
            w
        });
        this.regs()
            .int_ena_clr()
            .write(|w| unsafe { w.bits(!int_mask) });

        // Set up the GPIO pins.
        let tx_config = if no_transceiver {
            OutputConfig::default()
                .with_drive_mode(DriveMode::OpenDrain)
                .with_pull(Pull::Up)
        } else {
            OutputConfig::default()
        };

        tx_pin.apply_output_config(&tx_config);
        rx_pin.apply_input_config(&InputConfig::default().with_pull(if no_transceiver {
            Pull::Up
        } else {
            Pull::None
        }));

        tx_pin.set_output_enable(true);
        rx_pin.set_input_enable(true);

        this.twai.output_signal().connect_to(&tx_pin);
        this.twai.input_signal().connect_to(&rx_pin);

        this.set_baud_rate(baud_rate);
        this
    }

    fn regs(&self) -> &RegisterBlock {
        self.twai.register_block()
    }

    fn internal_set_interrupt_handler(&mut self, handler: InterruptHandler) {
        for core in Cpu::other() {
            crate::interrupt::disable(core, self.twai.interrupt());
        }
        crate::interrupt::bind_handler(self.twai.interrupt(), handler);
    }

    /// Set the bitrate of the bus.
    fn set_baud_rate(&mut self, baud_rate: BaudRate) {
        let timing = baud_rate.timing();
        self.regs().btr().write(|w| unsafe {
            w.brp().bits(timing.baud_rate_prescaler);
            w.sjw().bits(timing.sync_jump_width);
            w.ph1().bits(timing.tseg_1);
            w.ph2().bits(timing.tseg_2);
            w.prop().bits(timing.prop);
            w
        });
    }

    /// Set up the acceptance filter on the device.
    ///
    /// NOTE: On a bus with mixed 11-bit and 29-bit packet id's, you may
    /// experience an 11-bit filter match against a 29-bit frame and vice
    /// versa. Your application should check the id again once a frame has
    /// been received to make sure it is the expected value.
    ///
    /// You may use a `const {}` block to ensure that the filter is parsed
    /// during program compilation.
    ///
    /// The filter is not applied to the peripheral until [`Self::start`] is
    /// called.
    ///
    /// [ESP32C3 Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf#subsubsection.29.4.6)
    pub fn set_filter(&mut self, filter: impl Filter) {
        // Convert the filter into values for the registers and store them for later
        // use.
        self.filter = Some((filter.filter_type(), filter.to_registers()));
    }

    fn apply_filter(&self) {
        let Some((_filter_type, _registers)) = self.filter.as_ref() else {
            todo!()
        };
    }

    /// Set the error warning threshold.
    ///
    /// In the case when any of an error counter value exceeds the threshold, or
    /// all the error counter values are below the threshold, an error
    /// warning interrupt will be triggered (given the enable signal is
    /// valid).
    pub fn set_error_warning_limit(&mut self, _limit: u8) {
        // TODO: Implement
        todo!()
    }

    /// Set the operating mode based on provided option
    fn set_mode(&self, mode: TwaiMode) {
        assert!(self.regs().mode_settings().read().ena().bit_is_clear());
        self.regs().mode_settings().modify(|_, w| {
            w.stm().bit(mode.self_test);
            w.bmm().bit(mode.listen_only);
            w.rom().bit(mode.listen_only);
            w.acf().bit(mode.listen_only);
            w.ilbp().bit(mode.loopback);
            w
        });
    }

    /// Put the peripheral into Operation Mode, allowing the transmission and
    /// reception of packets using the new object.
    pub fn start(self) -> Twai<'d, Dm> {
        // self.apply_filter();
        self.set_mode(self.mode);

        // Put the peripheral into operation mode by setting the ena register
        self.regs().mode_settings().modify(|_, w| {
            let _ = w.ena().set_bit();
            w
        });

        // Wait for state change
        while self.regs().int_stat().read().fcsi_int_st().bit_is_clear() {
            core::hint::spin_loop();
        }

        Twai {
            rx: TwaiRx {
                twai: unsafe { self.twai.clone_unchecked() },
                phantom: PhantomData,
                _guard: PeripheralGuard::new(self.twai.peripheral()),
            },
            tx: TwaiTx {
                twai: unsafe { self.twai.clone_unchecked() },
                phantom: PhantomData,
                _guard: PeripheralGuard::new(self.twai.peripheral()),
            },
            twai: unsafe { self.twai.clone_unchecked() },
            phantom: PhantomData,
        }
    }
}

impl<'d> TwaiConfiguration<'d, Blocking> {
    /// Create a new instance of [TwaiConfiguration]
    ///
    /// You will need to use a transceiver to connect to the TWAI bus
    pub fn new(
        peripheral: impl Instance + 'd,
        rx_pin: impl PeripheralInput<'d>,
        tx_pin: impl PeripheralOutput<'d>,
        baud_rate: BaudRate,
        mode: TwaiMode,
    ) -> Self {
        Self::new_internal(peripheral.degrade(), rx_pin, tx_pin, baud_rate, false, mode)
    }

    /// Create a new instance of [TwaiConfiguration] meant to connect two ESP32s
    /// directly
    ///
    /// You don't need a transceiver by following the description in the
    /// `twai.rs` example
    pub fn new_no_transceiver(
        peripheral: impl Instance + 'd,
        rx_pin: impl PeripheralInput<'d>,
        tx_pin: impl PeripheralOutput<'d>,
        baud_rate: BaudRate,
        mode: TwaiMode,
    ) -> Self {
        Self::new_internal(peripheral.degrade(), rx_pin, tx_pin, baud_rate, true, mode)
    }

    /// Convert the configuration into an async configuration.
    pub fn into_async(mut self) -> TwaiConfiguration<'d, Async> {
        self.set_interrupt_handler(self.twai.async_handler());
        TwaiConfiguration {
            twai: self.twai,
            filter: self.filter,
            phantom: PhantomData,
            mode: self.mode,
            initialized: self.initialized,
            _guard: self._guard,
        }
    }

    /// Registers an interrupt handler for the TWAI peripheral.
    ///
    /// Note that this will replace any previously registered interrupt
    /// handlers.
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        self.internal_set_interrupt_handler(handler);
    }
}

impl<'d> TwaiConfiguration<'d, Async> {
    /// Convert the configuration into a blocking configuration.
    pub fn into_blocking(self) -> TwaiConfiguration<'d, Blocking> {
        use crate::{interrupt, system::Cpu};

        interrupt::disable(Cpu::current(), self.twai.interrupt());

        // Re-create in  blocking mode
        TwaiConfiguration {
            twai: self.twai,
            filter: self.filter,
            phantom: PhantomData,
            mode: self.mode,
            initialized: self.initialized,
            _guard: self._guard,
        }
    }
}

impl crate::private::Sealed for TwaiConfiguration<'_, Blocking> {}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for TwaiConfiguration<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        self.internal_set_interrupt_handler(handler);
    }
}

/// An active TWAI peripheral in Normal Mode.
///
/// In this mode, the TWAI controller can transmit and receive messages
/// including error signals (such as error and overload frames).
pub struct Twai<'d, Dm: DriverMode> {
    twai: AnyTwai<'d>,
    tx: TwaiTx<'d, Dm>,
    rx: TwaiRx<'d, Dm>,
    phantom: PhantomData<Dm>,
}

impl<'d, Dm> Twai<'d, Dm>
where
    Dm: DriverMode,
{
    fn regs(&self) -> &RegisterBlock {
        self.twai.register_block()
    }

    fn mode(&self) -> TwaiMode {
        let mode = self.regs().mode_settings().read();
        TwaiMode {
            self_test: mode.stm().bit_is_set(),
            listen_only: mode.bmm().bit_is_set(),
            loopback: mode.ilbp().bit_is_set(),
        }
    }

    /// Stop the peripheral, putting it into reset mode and enabling
    /// reconfiguration.
    pub fn stop(self) -> TwaiConfiguration<'d, Dm> {
        // Put the peripheral into reset/configuration mode by setting the reset mode
        // bit.

        self.regs()
            .mode_settings()
            .modify(|_, w| w.ena().bit(false));

        let mode = self.mode();

        let guard = PeripheralGuard::new(self.twai.peripheral());
        TwaiConfiguration {
            twai: self.twai,
            filter: None, // filter already applied, no need to restore it
            phantom: PhantomData,
            mode,
            initialized: false,
            _guard: guard,
        }
    }

    /// Returns the value of the receive error counter.
    pub fn receive_error_count(&self) -> u8 {
        self.regs().rec_tec().read().rec_val().bits() as u8
    }

    /// Returns the value of the transmit error counter.
    pub fn transmit_error_count(&self) -> u8 {
        self.regs().rec_tec().read().tec_val().bits() as u8
    }

    /// Check if the controller is in a bus off state.
    pub fn is_bus_off(&self) -> bool {
        self.regs().mode_settings().read().ena().bit_is_clear()
    }

    /// Get the number of messages that the peripheral has available in the
    /// receive FIFO.
    ///
    /// Note that this may not be the number of valid messages in the receive
    /// FIFO due to fifo overflow/overrun.
    pub fn num_available_messages(&self) -> u8 {
        self.regs().rx_status_rx_settings().read().rxfrc().bits() as u8
    }

    /// Clear the receive FIFO, discarding any valid, partial, or invalid
    /// packets.
    pub fn clear_receive_fifo(&self) {
        while self.num_available_messages() > 0 {
            release_receive_fifo(self.regs());
        }
    }

    /// Sends the specified `EspTwaiFrame` over the TWAI bus.
    pub fn transmit(&mut self, frame: &EspTwaiFrame) -> nb::Result<(), EspTwaiError> {
        self.tx.transmit(frame)
    }

    /// Receives a TWAI frame from the TWAI bus.
    pub fn receive(&mut self) -> nb::Result<EspTwaiFrame, EspTwaiError> {
        self.rx.receive()
    }

    /// Consumes this `Twai` instance and splits it into transmitting and
    /// receiving halves.
    pub fn split(self) -> (TwaiRx<'d, Dm>, TwaiTx<'d, Dm>) {
        (self.rx, self.tx)
    }
}

/// Interface to the TWAI transmitter part.
pub struct TwaiTx<'d, Dm: DriverMode> {
    twai: AnyTwai<'d>,
    phantom: PhantomData<Dm>,
    _guard: PeripheralGuard,
}

impl<Dm> TwaiTx<'_, Dm>
where
    Dm: DriverMode,
{
    #[allow(unused)] // Unused while transmit is not implemented
    fn regs(&self) -> &RegisterBlock {
        self.twai.register_block()
    }

    /// Transmit a frame.
    ///
    /// Because of how the TWAI registers are set up, we have to do some
    /// assembly of bytes. Note that these registers serve a filter
    /// configuration role when the device is in configuration mode so
    /// patching the svd files to improve this may be non-trivial.
    ///
    /// [ESP32C3 Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf#subsubsection.29.4.4.2)
    ///
    /// NOTE: TODO: This may not work if using the self reception/self test
    /// functionality. See notes 1 and 2 in the "Frame Identifier" section
    /// of the reference manual.
    pub fn transmit(&mut self, _frame: &EspTwaiFrame) -> nb::Result<(), EspTwaiError> {
        todo!();
    }
}

/// Interface to the TWAI receiver part.
pub struct TwaiRx<'d, Dm: DriverMode> {
    twai: AnyTwai<'d>,
    phantom: PhantomData<Dm>,
    _guard: PeripheralGuard,
}

impl<Dm> TwaiRx<'_, Dm>
where
    Dm: DriverMode,
{
    fn regs(&self) -> &RegisterBlock {
        self.twai.register_block()
    }

    /// Receive a frame
    pub fn receive(&mut self) -> nb::Result<EspTwaiFrame, EspTwaiError> {
        // Not implemented, use the async version instead
        todo!();
    }
}

/// List of TWAI events.
#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum TwaiInterrupt {
    /// A frame has been received.
    Receive,
    /// A frame has been transmitted.
    Transmit,
    /// An error has occurred on the bus.
    BusError,
    /// An arbitration lost event has occurred.
    ArbitrationLost,
    /// The controller has entered an error passive state.
    ErrorPassive,
}

/// Represents errors that can occur in the TWAI driver.
/// This enum defines the possible errors that can be encountered when
/// interacting with the TWAI peripheral.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EspTwaiError {
    /// TWAI peripheral has entered a bus-off state.
    BusOff,
    /// The received frame contains an invalid DLC.
    NonCompliantDlc(u8),
    /// Encapsulates errors defined by the embedded-hal crate.
    EmbeddedHAL(ErrorKind),
}

#[instability::unstable]
impl embedded_can::Error for EspTwaiError {
    fn kind(&self) -> embedded_can::ErrorKind {
        if let Self::EmbeddedHAL(kind) = self {
            (*kind).into()
        } else {
            embedded_can::ErrorKind::Other
        }
    }
}

#[instability::unstable]
impl<Dm> embedded_can::nb::Can for Twai<'_, Dm>
where
    Dm: DriverMode,
{
    type Frame = EspTwaiFrame;
    type Error = EspTwaiError;

    /// Transmit a frame.
    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        self.tx.transmit(frame)?;

        // Success in readying packet for transmit. No packets can be replaced in the
        // transmit buffer so return None in accordance with the
        // embedded-can/embedded-hal trait.
        nb::Result::Ok(None)
    }

    /// Return a received frame if there are any available.
    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        self.rx.receive()
    }
}

/// TWAI peripheral instance.
#[doc(hidden)]
pub trait PrivateInstance: crate::private::Sealed {
    /// Returns the system peripheral marker for this instance.
    fn peripheral(&self) -> crate::system::Peripheral;

    /// Input signal.
    fn input_signal(&self) -> InputSignal;
    /// Output signal.
    fn output_signal(&self) -> OutputSignal;
    /// The interrupt associated with this TWAI instance.
    fn interrupt(&self) -> crate::peripherals::Interrupt;

    /// Provides an asynchronous interrupt handler for TWAI instance.
    fn async_handler(&self) -> InterruptHandler;

    /// Returns a reference to the register block for TWAI instance.
    fn register_block(&self) -> &RegisterBlock;

    /// Enables/disables interrupts for the TWAI peripheral based on the `enable` flag.
    fn enable_interrupts(&self, interrupts: EnumSet<TwaiInterrupt>, enable: bool) {
        self.register_block().int_ena_set().modify(|_, w| {
            for interrupt in interrupts {
                match interrupt {
                    TwaiInterrupt::Receive => w.rxi_int_ena_mask().bit(enable),
                    TwaiInterrupt::Transmit => w.txi_int_ena_mask().bit(enable),
                    TwaiInterrupt::BusError => w.bsi_int_ena_mask().bit(enable),
                    TwaiInterrupt::ArbitrationLost => w.ali_int_ena_mask().bit(enable),
                    TwaiInterrupt::ErrorPassive => w.ewli_int_ena_mask().bit(enable),
                };
            }
            w
        });
    }

    /// Listen for given interrupts.
    fn listen(&mut self, interrupts: impl Into<EnumSet<TwaiInterrupt>>) {
        self.enable_interrupts(interrupts.into(), true);
    }

    /// Unlisten the given interrupts.
    fn unlisten(&mut self, interrupts: impl Into<EnumSet<TwaiInterrupt>>) {
        self.enable_interrupts(interrupts.into(), false);
    }
    /// Returns a reference to the asynchronous state for this TWAI instance.
    fn async_state(&self) -> &asynch::TwaiAsyncState;
}

fn read_frame(register_block: &RegisterBlock) -> Result<EspTwaiFrame, EspTwaiError> {
    // Ensure that we are at the start of a frame. If not there has likely been an overrun.
    if register_block
        .rx_status_rx_settings()
        .read()
        .rxmof()
        .bit_is_clear()
    {
        release_receive_fifo(register_block);
        if register_block
            .rx_status_rx_settings()
            .read()
            .rxe()
            .bit_is_set()
        {
            return Err(EspTwaiError::EmbeddedHAL(ErrorKind::Overrun));
        }
    }

    let frame_format = register_block.rx_data().read().rx_data().bits();

    let is_standard_format = frame_format & (0b1 << 6) == 0;
    let dlc = (frame_format & 0b1111) as u8;

    if dlc > 64 {
        release_receive_fifo(register_block);
        return Err(EspTwaiError::NonCompliantDlc(dlc));
    }

    let id = register_block.rx_data().read().bits();
    let id = if is_standard_format {
        let id = (id >> 18) & 0x7FF;
        Id::Standard(StandardId::new(id as u16).unwrap())
    } else {
        let id = id & 0x1FFFFFFF;
        Id::Extended(ExtendedId::new(id).unwrap())
    };

    let _timestamp_low = register_block.rx_data().read();
    let _timestamp_high = register_block.rx_data().read();
    let mut buffer: [u8; 64] = [0; _];
    // 4 because u32 is 4 u8s
    for i in 0..(dlc.div_ceil(4)) as usize {
        let start = i * 4;
        let end = start + 4;
        buffer[start..end].copy_from_slice(&register_block.rx_data().read().bits().to_le_bytes());
    }

    let mut frame = EspTwaiFrame::new(id, &buffer[..dlc as usize]).expect("Data length exceeded");

    frame.self_reception = false;

    Ok(frame)
}

/// Release the message in the buffer. This will decrement the received
/// message counter and prepare the next message in the FIFO for
/// reading.
fn release_receive_fifo(register_block: &RegisterBlock) {
    while register_block.rx_status_rx_settings().read().rxmof().bit() {
        // Flush the incoming message buffer until we reach the start of a new frame
        let _ = register_block.rx_data().read();
    }
}

/// Write a frame to the peripheral.
fn write_frame(register_block: &RegisterBlock, frame: &EspTwaiFrame, buffer_idx: u8) {
    // Assemble the frame information into the data_0 byte.
    let ide: u32 = matches!(frame.id, Id::Extended(_)) as u32;
    let rtr_bit: u32 = frame.is_remote as u32;
    let fdf = (frame.dlc > 8) as u32;
    let dlc_bits: u32 = (frame.dlc & 0xFF) as u32;
    // let rwcnt = dlc_bits + 4;

    let frame_format: u32 = (fdf << 7) | (ide << 6) | (rtr_bit << 5) | dlc_bits;

    let id = match frame.id {
        Id::Standard(id) => (id.as_raw() as u32 & 0x7FF) << 18,
        Id::Extended(id) => id.as_raw() & 0x1FFF_FFFF,
    };

    unsafe {
        let buffer_ptr = (reg::TWAI0_FD_BASE as *mut u8).offset(match buffer_idx {
            0 => reg::TWAI_TX_BUF1,
            1 => reg::TWAI_TX_BUF2,
            2 => reg::TWAI_TX_BUF3,
            3 => reg::TWAI_TX_BUF4,
            _ => panic!("Illegal buffer"),
        });

        let frame_format_ptr = buffer_ptr;
        let id_ptr = buffer_ptr.offset(4);
        let time_low = buffer_ptr.offset(8);
        let time_high = buffer_ptr.offset(12);
        let data_ptr = buffer_ptr.offset(16);

        assert!(id_ptr != buffer_ptr);

        ptr::write_volatile(frame_format_ptr as *mut u32, frame_format);
        ptr::write_volatile(id_ptr as *mut u32, id);
        ptr::write_volatile(time_low as *mut u32, 0u32);
        ptr::write_volatile(time_high as *mut u32, 0u32);
        ptr::copy_nonoverlapping(frame.data.as_ptr(), data_ptr as *mut u8, frame.dlc);
    }

    register_block.tx_command_txtb_info().modify(|_, w| {
        match buffer_idx {
            0 => w.txb1().set_bit(),
            1 => w.txb2().set_bit(),
            2 => w.txb3().set_bit(),
            3 => w.txb4().set_bit(),
            _ => panic!("Illegal buffer"),
        };
        w.txcr().set_bit()
    });
}

impl PrivateInstance for crate::peripherals::TWAI0<'_> {
    fn peripheral(&self) -> crate::system::Peripheral {
        crate::system::Peripheral::Twai0
    }

    fn input_signal(&self) -> InputSignal {
        InputSignal::TWAI0_RX
    }

    fn output_signal(&self) -> OutputSignal {
        OutputSignal::TWAI0_TX
    }

    fn interrupt(&self) -> crate::peripherals::Interrupt {
        crate::peripherals::Interrupt::TWAI0
    }

    fn async_handler(&self) -> InterruptHandler {
        #[handler]
        fn twai0() {
            debug!("twai0 interrupt handler");
            let twai = unsafe { crate::peripherals::TWAI0::steal() };
            asynch::handle_interrupt(twai.register_block(), twai.async_state());
        }

        twai0
    }

    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        crate::peripherals::TWAI0::regs()
    }

    fn async_state(&self) -> &asynch::TwaiAsyncState {
        static STATE: asynch::TwaiAsyncState = asynch::TwaiAsyncState::new();
        &STATE
    }
}

#[cfg(soc_has_twai1)]
impl PrivateInstance for crate::peripherals::TWAI1<'_> {
    fn peripheral(&self) -> crate::system::Peripheral {
        crate::system::Peripheral::Twai1
    }

    fn input_signal(&self) -> InputSignal {
        InputSignal::TWAI1_RX
    }

    fn output_signal(&self) -> OutputSignal {
        OutputSignal::TWAI1_TX
    }

    fn interrupt(&self) -> crate::peripherals::Interrupt {
        crate::peripherals::Interrupt::TWAI1
    }

    fn async_handler(&self) -> InterruptHandler {
        #[handler]
        fn twai1() {
            let twai = unsafe { crate::peripherals::TWAI1::steal() };
            asynch::handle_interrupt(twai.register_block(), twai.async_state());
        }

        twai1
    }

    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        crate::peripherals::TWAI1::regs()
    }

    fn async_state(&self) -> &asynch::TwaiAsyncState {
        static STATE: asynch::TwaiAsyncState = asynch::TwaiAsyncState::new();
        &STATE
    }
}

crate::any_peripheral! {
    /// Any TWAI peripheral.
    pub peripheral AnyTwai<'d> {
        #[cfg(soc_has_twai0)]
        Twai0(crate::peripherals::TWAI0<'d>),
        #[cfg(soc_has_twai1)]
        Twai1(crate::peripherals::TWAI1<'d>),
    }
}

impl PrivateInstance for AnyTwai<'_> {
    delegate::delegate! {
        to match &self.0 {
            #[cfg(soc_has_twai0)]
            any::Inner::Twai0(twai) => twai,
            #[cfg(soc_has_twai1)]
            any::Inner::Twai1(twai) => twai,
        } {
            fn peripheral(&self) -> crate::system::Peripheral;
            fn input_signal(&self) -> InputSignal;
            fn output_signal(&self) -> OutputSignal;
            fn interrupt(&self) -> crate::peripherals::Interrupt;
            fn async_handler(&self) -> InterruptHandler;
            fn register_block(&self) -> &RegisterBlock;
            fn async_state(&self) -> &asynch::TwaiAsyncState;
        }
    }
}

/// A peripheral singleton compatible with the TWAI driver.
pub trait Instance: PrivateInstance + any::Degrade {}

#[cfg(soc_has_twai0)]
impl Instance for crate::peripherals::TWAI0<'_> {}
#[cfg(soc_has_twai1)]
impl Instance for crate::peripherals::TWAI1<'_> {}
impl Instance for AnyTwai<'_> {}

mod asynch {
    use core::{future::poll_fn, task::Poll};

    use embassy_sync::{channel::Channel, waitqueue::AtomicWaker};
    use esp_sync::RawMutex;

    use super::*;

    pub struct TwaiAsyncState {
        pub tx_waker: AtomicWaker,
        pub err_waker: AtomicWaker,
        pub rx_waker: AtomicWaker,
        pub rx_queue: Channel<RawMutex, Result<EspTwaiFrame, EspTwaiError>, 32>,
    }

    impl Default for TwaiAsyncState {
        fn default() -> Self {
            Self::new()
        }
    }

    impl TwaiAsyncState {
        pub const fn new() -> Self {
            Self {
                tx_waker: AtomicWaker::new(),
                err_waker: AtomicWaker::new(),
                rx_waker: AtomicWaker::new(),
                rx_queue: Channel::new(),
            }
        }
    }

    impl Twai<'_, Async> {
        /// Transmits an `EspTwaiFrame` asynchronously over the TWAI bus.
        ///
        /// The transmission is aborted if the future is dropped. The technical
        /// reference manual does not specifiy if aborting the transmission also
        /// stops it, in case it is activly transmitting. Therefor it could be
        /// the case that even though the future is dropped, the frame was sent
        /// anyways.
        pub async fn transmit_async(&mut self, frame: &EspTwaiFrame) -> Result<(), EspTwaiError> {
            self.tx.transmit_async(frame).await
        }
        /// Receives an `EspTwaiFrame` asynchronously over the TWAI bus.
        pub async fn receive_async(&mut self) -> Result<EspTwaiFrame, EspTwaiError> {
            self.rx.receive_async().await
        }
    }

    #[must_use = "futures do nothing unless you `.await` or poll them"]
    pub struct TransmitFuture<'d, 'f> {
        twai: AnyTwai<'d>,
        frame: &'f EspTwaiFrame,
        in_flight: Option<u8>,
    }

    impl<'d, 'f> TransmitFuture<'d, 'f> {
        pub fn new(twai: AnyTwai<'d>, frame: &'f EspTwaiFrame) -> Self {
            Self {
                twai,
                frame,
                in_flight: None,
            }
        }
    }

    impl core::future::Future for TransmitFuture<'_, '_> {
        type Output = Result<(), EspTwaiError>;

        fn poll(
            mut self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> Poll<Self::Output> {
            const NOT_EXIST: u8 = 0;
            const READY: u8 = 1;
            const TRANSMITTING: u8 = 2;
            const ABORT_IN_PROGRESS: u8 = 3;
            const OK: u8 = 4;
            const ERROR: u8 = 6;
            const ABORTED: u8 = 7;
            const EMPTY: u8 = 8;

            self.twai.async_state().tx_waker.register(cx.waker());

            if self
                .twai
                .register_block()
                .mode_settings()
                .read()
                .ena()
                .bit_is_clear()
            {
                return Poll::Ready(Err(EspTwaiError::BusOff));
            }

            if self.in_flight.is_none() {
                // Pick an empty buffer
                let tx_status = self.twai.register_block().tx_status().read();
                let buffer_idx = if tx_status.txtb0_state().bits() >= OK {
                    0
                } else if tx_status.tx2s().bits() >= OK {
                    1
                } else if tx_status.tx2s().bits() >= OK {
                    2
                } else if tx_status.tx2s().bits() >= OK {
                    3
                } else {
                    // No buffer is empty
                    return Poll::Ready(Err(EspTwaiError::BusOff));
                };

                write_frame(self.twai.register_block(), self.frame, buffer_idx);
                self.in_flight = Some(buffer_idx);
            }

            if let Some(buffer_idx) = &self.in_flight {
                let tx_status = self.twai.register_block().tx_status().read();
                let tx_buffer_status = match buffer_idx {
                    0 => tx_status.txtb0_state().bits(),
                    1 => tx_status.tx2s().bits(),
                    2 => tx_status.tx3s().bits(),
                    3 => tx_status.tx4s().bits(),
                    _ => panic!("Illegal buffer"),
                };

                return match tx_buffer_status {
                    ERROR | ABORTED | EMPTY | NOT_EXIST => Poll::Ready(Err(EspTwaiError::BusOff)),
                    READY | TRANSMITTING | ABORT_IN_PROGRESS => Poll::Pending,
                    OK => Poll::Ready(Ok(())),
                    _ => unreachable!(),
                };
            }

            Poll::Pending
        }
    }

    impl Drop for TransmitFuture<'_, '_> {
        fn drop(&mut self) {
            // Attempt to abort outgoing send
            if let Some(buffer_idx) = self.in_flight {
                self.twai
                    .register_block()
                    .tx_command_txtb_info()
                    .write(|w| {
                        match buffer_idx {
                            0 => w.txb1().set_bit(),
                            1 => w.txb2().set_bit(),
                            2 => w.txb3().set_bit(),
                            3 => w.txb4().set_bit(),
                            _ => panic!("Illegal buffer"),
                        };
                        w.txca().set_bit()
                    });
            }
        }
    }

    impl TwaiTx<'_, Async> {
        /// Transmits an `EspTwaiFrame` asynchronously over the TWAI bus.
        ///
        /// The transmission is aborted if the future is dropped. The technical
        /// reference manual does not specifiy if aborting the transmission also
        /// stops it, in case it is actively transmitting. Therefor it could be
        /// the case that even though the future is dropped, the frame was sent
        /// anyways.
        pub async fn transmit_async(&mut self, frame: &EspTwaiFrame) -> Result<(), EspTwaiError> {
            self.twai.listen(EnumSet::all());
            TransmitFuture::new(self.twai.reborrow(), frame).await
        }
    }

    impl TwaiRx<'_, Async> {
        /// Receives an `EspTwaiFrame` asynchronously over the TWAI bus.
        pub async fn receive_async(&mut self) -> Result<EspTwaiFrame, EspTwaiError> {
            self.twai.listen(EnumSet::all());
            poll_fn(|cx| {
                self.twai.async_state().err_waker.register(cx.waker());
                self.twai.async_state().rx_waker.register(cx.waker());

                if let Ok(result) = self.twai.async_state().rx_queue.try_receive() {
                    return Poll::Ready(result);
                }

                // Check that the peripheral is not in a bus off state.
                if self.regs().mode_settings().read().ena().bit_is_clear() {
                    return Poll::Ready(Err(EspTwaiError::BusOff));
                }

                Poll::Pending
            })
            .await
        }
    }

    pub(super) fn handle_interrupt(register_block: &RegisterBlock, async_state: &TwaiAsyncState) {
        let intr_status = register_block.int_stat().read();

        if intr_status.ewli_int_st().bit_is_set() | intr_status.ali_int_st().bit_is_set() {
            async_state.err_waker.wake();
        }

        if intr_status.doi_int_st().bit_is_set() {
            // DOI_INT_ST must be manually cleared to avoid being set again
            async_state.err_waker.wake();
            register_block
                .int_stat()
                .modify(|_, w| w.doi_int_st().clear_bit());
        }

        if intr_status.bsi_int_st().bit_is_set() {
            // TODO: Switch data rate
        }

        if intr_status.rxi_int_st().bit_is_set() {
            if register_block.mode_settings().read().ena().bit_is_clear() {
                let _ = async_state.rx_queue.try_send(Err(EspTwaiError::BusOff));
            }

            match read_frame(register_block) {
                Ok(frame) => {
                    let _ = async_state.rx_queue.try_send(Ok(frame));
                }
                Err(e) => {}
            };
            async_state.rx_waker.wake();
        }

        // async_state.tx_waker.wake();
        if intr_status.txi_int_st().bit_is_set() || intr_status.txbhci_int_st().bit_is_set() {
            async_state.tx_waker.wake();
        }

        register_block
            .int_ena_clr()
            .write(|w| unsafe { w.bits(intr_status.bits()) });
    }
}
