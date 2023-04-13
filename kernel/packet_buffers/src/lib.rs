#![no_std]
#![feature(adt_const_params)]
#![allow(incomplete_features)]

extern crate memory;
extern crate zerocopy;
extern crate owning_ref;
extern crate alloc;
#[macro_use] extern crate static_assertions;

use core::ops::{Deref, DerefMut};
use memory::{MappedPages, PhysicalAddress, create_contiguous_mapping, EntryFlags, BorrowedMappedPages, Mutable};
use zerocopy::FromBytes;
use owning_ref::BoxRefMut;
use alloc::boxed::Box;

/// All buffers are created with 2KiB so that the max ethernet frame can fit in one packet buffer
pub const DEFAULT_RX_BUFFER_SIZE_IN_BYTES_2KB: usize   = 2 * 1024;

/// The mapping flags used for descriptors and packet buffers
const NIC_MAPPING_FLAGS_CACHED: EntryFlags = EntryFlags::from_bits_truncate(
    EntryFlags::PRESENT.bits() |
    EntryFlags::WRITABLE.bits() |
    EntryFlags::NO_EXECUTE.bits()
);


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
    phys_addr: PhysicalAddress,
    pub length: u16,
    pub buffer: BorrowedMappedPages<EthernetFrame, Mutable> //look into ouborous or pinned. should be able to store reference to MappedPages
}
const_assert_eq!(core::mem::size_of::<PacketBufferS>(), 64);
const_assert_eq!(core::mem::size_of::<BorrowedMappedPages<EthernetFrame, Mutable>>(), 48);



impl<const N: MTU> PacketBuffer<N> {
    /// Creates a new `PacketBuffer` of the standard 2 KiB size.
    /// The actual size of the buffer is always >= `length_in_bytes`
    /// The `length_in_bytes` is a `u16` because that is the maximum size of a NIC buffer. 
    pub fn new(mut length_in_bytes: u16) -> Result<PacketBuffer<{MTU::Standard}>, &'static str> {
        if length_in_bytes >  MAX_STANDARD_ETHERNET_FRAME_LEN_IN_BYTES {
            return Err("Size of packet buffer is larger than MTU");
        } else if length_in_bytes < MIN_ETHERNET_FRAME_LEN_IN_BYTES{
            length_in_bytes = MIN_ETHERNET_FRAME_LEN_IN_BYTES;
        }

        let (mp, starting_phys_addr) = create_contiguous_mapping(
            DEFAULT_RX_BUFFER_SIZE_IN_BYTES_2KB,
            NIC_MAPPING_FLAGS_CACHED,
        )?;
        
        let buffer = mp.into_borrowed_mut(0).map_err(|(_mp, err)| err)?;
        Ok(PacketBuffer {
            phys_addr: starting_phys_addr,
            length: length_in_bytes,
            buffer
        })
    }

    #[inline(always)]
    pub fn phys_addr(&self) -> PhysicalAddress {
        self.phys_addr
    }
}

impl<const N: MTU> Deref for PacketBuffer<N> {
    type Target = EthernetFrame;
    #[inline(always)]
    fn deref(&self) -> &EthernetFrame {
        &self.buffer
    }
}

impl<const N: MTU> DerefMut for PacketBuffer<N> {
    #[inline(always)]
    fn deref_mut(&mut self) -> &mut EthernetFrame {
        &mut self.buffer
    }
}


/// A struct that makes it easy to access different fields of an ethernet frame
/// Note: Tried to use const generics for the payload, but it fails when trying to derive FromBytes, works otherwise.
#[derive(FromBytes)]
#[repr(C)]
pub struct EthernetFrame {
    pub dest_addr:  [u8; MAC_ADDR_LEN_IN_BYTES as usize],
    pub src_addr:   [u8; MAC_ADDR_LEN_IN_BYTES as usize],
    pub length:     u16,
    pub payload:    [u8; MAX_STANDARD_PAYLOAD_LEN_IN_BYTES as usize],
    _padding: [u8; 2048 - 1514]
}

const_assert_eq!(core::mem::size_of::<EthernetFrame>(), 2048);
