pub(crate) mod descriptors;
pub(crate) mod regs;

cfg_if::cfg_if! {
if #[cfg(not(prusti))] {

/*** Hardware Device Parameters of the Intel 82599 NIC (taken from the datasheet) ***/

/// The maximum number of receive descriptors per queue.
/// This is the maximum value that has been tested for the 82599 device.
pub(crate) const IXGBE_MAX_RX_DESC: u16            = 8192;
/// The maximum number of transmit descriptors per queue.
/// This is the maximum value that has been tested for the 82599 device.
pub(crate) const IXGBE_MAX_TX_DESC: u16            = 8192;
/// The maximum number of rx queues available on this NIC. 
pub(crate) const IXGBE_MAX_RX_QUEUES: u8           = 128;
/// The maximum number of tx queues available on this NIC.
pub(crate) const IXGBE_MAX_TX_QUEUES: u8           = 128;
/// The number of l34 5-tuple filters.
pub(crate) const NUM_L34_5_TUPLE_FILTERS: usize    = 128; 



/// Possible link speeds of the 82599 NIC
#[derive(PartialEq)]
pub enum LinkSpeedMbps {
    LS100 = 100,
    LS1000 = 1000,
    LS10000 = 10000, 
    LSUnknown = 0,
}

impl LinkSpeedMbps {
    /// Converts between a u32 and a LinkSpeedMbps enum.
    /// The u32 number is the value in the links register that represents the link speed.
    pub(crate) fn from_links_register_value(value: u32) -> LinkSpeedMbps {
        if value == (1 << 28) {
            Self::LS100
        } else if value == (2 << 28) {
            Self::LS1000
        } else if value == (3 << 28) {
            Self::LS10000
        } else {
            Self::LSUnknown
        }
    }
}

/// The set of receive buffer sizes that are accepted by the 82599 device.
#[derive(Copy, Clone)]
pub enum RxBufferSizeKiB {
    Buffer1KiB = 1,
    Buffer2KiB = 2,
    Buffer3KiB = 3,
    Buffer4KiB = 4,
    Buffer5KiB = 5,
    Buffer6KiB = 6,
    Buffer7KiB = 7,
    Buffer8KiB = 8,
    Buffer9KiB = 9,
    Buffer10KiB = 10,
    Buffer11KiB = 11,
    Buffer12KiB = 12,
    Buffer13KiB = 13,
    Buffer14KiB = 14,
    Buffer15KiB = 15,
    Buffer16KiB = 16
}

/// Options for the filter protocol used in the 5-tuple filters.
pub enum FilterProtocol {
    Tcp = 0,
    Udp = 1,
    Sctp = 2,
    Other = 3
}

#[derive(Copy, Clone)]
pub enum NumDesc {
    Descs16 = 16,
    Descs512 = 512,
    Descs1k = 1024,
    Descs2k = 2048,
    Descs4k = 4096,
    Descs8k = 8192
}

}}