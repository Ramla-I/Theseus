use zerocopy::FromBytes;
use static_assertions::*;
use volatile::Volatile;

/// All buffers are created with 2KiB so that the max ethernet frame can fit in one packet buffer
pub const DEFAULT_RX_BUFFER_SIZE_IN_BYTES:              usize   = 2 * 1024;
/// Size of ether type field in ethernet frame header
pub const ETHER_TYPE_LEN_IN_BYTES:                      u16 = 2;
/// Size of CRC checksum field in ethernet frame header
pub const CRC_CHECKSUM_LEN_IN_BYTES:                    u16 = 4;
pub const MAC_ADDR_LEN_IN_BYTES:                        u16 = 6;
/// Size of ethernet frame header which contains a source and destination mac address, and the ether type field.
pub const ETHERNET_HEADER_LEN_IN_BYTES:                 u16 = MAC_ADDR_LEN_IN_BYTES * 2 + ETHER_TYPE_LEN_IN_BYTES;
/// Minimum size of the payload of an ethernet frame (doesn't include the ethernet header or offloaded CRC)
pub const MIN_PAYLOAD_LEN_IN_BYTES:                     u16 = 46;
/// Maximum size of the payload of an ethernet frame (doesn't include the ethernet header or offloaded CRC)
pub const MAX_STANDARD_PAYLOAD_LEN_IN_BYTES:            u16 = 1500;
/// Minimum size of of an ethernet frame with an offloaded CRC
pub const MIN_ETHERNET_FRAME_LEN_IN_BYTES:              u16 = MIN_PAYLOAD_LEN_IN_BYTES + ETHERNET_HEADER_LEN_IN_BYTES;
/// Maximum size of of an ethernet frame with an offloaded CRC
pub const MAX_STANDARD_ETHERNET_FRAME_LEN_IN_BYTES:     u16 = MAX_STANDARD_PAYLOAD_LEN_IN_BYTES + ETHERNET_HEADER_LEN_IN_BYTES;



/// A struct that makes it easy to access different fields of an ethernet frame
#[derive(FromBytes)]
#[repr(C)]
pub struct EthernetFrame {
    pub dest_addr:  [u8; MAC_ADDR_LEN_IN_BYTES as usize],
    pub src_addr:   [u8; MAC_ADDR_LEN_IN_BYTES as usize],
    pub length:     u16,
    pub payload:    [u8; MAX_STANDARD_PAYLOAD_LEN_IN_BYTES as usize],
    _padding:       [u8; DEFAULT_RX_BUFFER_SIZE_IN_BYTES - 1514]
}

const_assert_eq!(core::mem::size_of::<EthernetFrame>(), DEFAULT_RX_BUFFER_SIZE_IN_BYTES);
