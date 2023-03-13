use prusti_contracts::*;
use core::ops::{Deref, DerefMut};

cfg_if::cfg_if! {
if #[cfg(prusti)] {

use crate::spec::memory_spec::*;

} else {

use memory::{MappedPages, PhysicalAddress, create_contiguous_mapping};
use zerocopy::FromBytes;
use crate::{DEFAULT_RX_BUFFER_SIZE_2KB, allocator::NIC_MAPPING_FLAGS_CACHED};


/// Size of ether type field in ethernet frame header
pub const ETHER_TYPE_LEN_IN_BYTES:                      u16 = 2;
/// Size of CRC checksum field in ethernet frame header
pub const CRC_CHECKSUM_LEN_IN_BYTES:                    u16 = 4;
pub const MAC_ADDR_LEN_IN_BYTES:                        u16 = 6;
/// Size of ethernet frame header which contains a source and destination mac address, and the ether type field.
pub const ETHERNET_HEADER_LEN_IN_BYTES:                 u16 = MAC_ADDR_LEN_IN_BYTES * 2 + ETHER_TYPE_LEN_IN_BYTES;
/// Minimum size of the payload of an ethernet frame (doesn't include the ethernet header)
pub const MIN_PAYLOAD_LEN_IN_BYTES:                     u16 = 46;
/// Maximum size of the payload of an ethernet frame (doesn't include the ethernet header)
pub const MAX_STANDARD_PAYLOAD_LEN_IN_BYTES:            u16 = 1500;
pub const MAX_STANDARD_ETHERNET_FRAME_LEN_IN_BYTES:     u16 = MAX_STANDARD_PAYLOAD_LEN_IN_BYTES + ETHERNET_HEADER_LEN_IN_BYTES + CRC_CHECKSUM_LEN_IN_BYTES;
pub const MIN_ETHERNET_FRAME_LEN_IN_BYTES:              u16 = MIN_PAYLOAD_LEN_IN_BYTES + ETHERNET_HEADER_LEN_IN_BYTES + CRC_CHECKSUM_LEN_IN_BYTES;

}}

/// The different payload sizes supported by the NIC.
#[derive(PartialEq, Eq)]
pub enum MTU {
    Standard    = 1500,
    Jumbo       = 9000
}

pub type PacketBufferS = PacketBuffer<{MTU::Standard}>;
pub type PacketBufferJ = PacketBuffer<{MTU::Jumbo}>;


/// A buffer that stores a packet to be transmitted or received through the NIC
/// and is guaranteed to be contiguous in physical memory. 
/// Auto-dereferences into a `MappedPages` object that represents its underlying memory. 
/// 
/// This is a combined packet buffer without any drop handler to make it easy to use buffers in a network function loop.
/// Network functions receive a packet, process it, and then transmit it.
pub struct PacketBuffer<const N: MTU> {
    pub(crate) mp: MappedPages,
    pub(crate) phys_addr: PhysicalAddress,
    pub(crate) length: u16,
    // pub buffer: *mut EthernetFrame //look into ouborous or pinned. should be able to store reference to MappedPages
}


impl core::cmp::PartialEq for PacketBufferS {
    #[pure]
    fn eq(&self, other: &Self) -> bool {
        self.phys_addr.value() == other.phys_addr.value()
    }
}

cfg_if::cfg_if! {
if #[cfg(not(prusti))] {

impl<const N: MTU> PacketBuffer<N> {
    /// Creates a new `PacketBuffer` of the standard 2 KiB size.
    /// The actual size of the buffer is always >= `length_in_bytes`
    /// The `length_in_bytes` is a `u16` because that is the maximum size of a NIC buffer. 
    /// # Note: The `length_in_bytes` should include the total length, payload + headers + checksum. 
    /// # Warning: Since the checksum is added by the NIC using a hardware offload, the last 4 bytes of the buffer should never be used.
    pub fn new(mut length_in_bytes: u16) -> Result<PacketBuffer<{MTU::Standard}>, &'static str> {
        if length_in_bytes >  MAX_STANDARD_ETHERNET_FRAME_LEN_IN_BYTES {
            return Err("Size of packet buffer is larger than MTU");
        } else if length_in_bytes < MIN_ETHERNET_FRAME_LEN_IN_BYTES{
            length_in_bytes = MIN_ETHERNET_FRAME_LEN_IN_BYTES;
        }

        let (mut mp, starting_phys_addr) = create_contiguous_mapping(
            DEFAULT_RX_BUFFER_SIZE_2KB as usize,
            NIC_MAPPING_FLAGS_CACHED,
        )?;
        
        // let buffer = mp.as_type_mut::<EthernetFrame>(0)? as *mut EthernetFrame;

        Ok(PacketBuffer {
            mp: mp,
            phys_addr: starting_phys_addr,
            length: length_in_bytes,
            // buffer
        })
    }

    pub fn len(&self) -> u16 {
        self.length
    }

    /// Returns the size of the buffer without the bytes used for the ethernet header and checksum
    pub fn ethernet_payload_len(&self) -> u16 {
        self.length - ETHERNET_HEADER_LEN_IN_BYTES - CRC_CHECKSUM_LEN_IN_BYTES
    }
}

impl<const N: MTU> Deref for PacketBuffer<N> {
    type Target = MappedPages;
    fn deref(&self) -> &MappedPages {
        &self.mp
    }
}

impl<const N: MTU> DerefMut for PacketBuffer<N> {
    fn deref_mut(&mut self) -> &mut MappedPages {
        &mut self.mp
    }
}


/// A struct that makes it easy to access different fields of an ethernet frame
/// Note: Tried to use const generics for the payload, but it fails when trying to derive FromBytes, works otherwise.
#[derive(FromBytes)]
pub struct EthernetFrame {
    pub dest_addr:  [u8; MAC_ADDR_LEN_IN_BYTES as usize],
    pub src_addr:   [u8; MAC_ADDR_LEN_IN_BYTES as usize],
    pub length:     u16,
    pub payload:    [u8; MAX_STANDARD_PAYLOAD_LEN_IN_BYTES as usize],
    crc:            [u8; CRC_CHECKSUM_LEN_IN_BYTES as usize]
}

}}