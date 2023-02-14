//! Defines buffers that are used to send and receive packets.

#![no_std]

extern crate alloc;
#[macro_use] extern crate log;
extern crate memory;
extern crate mpmc;

use core::ops::{Deref, DerefMut};
use alloc::vec::Vec;
use memory::{PhysicalAddress, MappedPages, EntryFlags, create_contiguous_mapping};


/// A buffer that stores a packet to be transmitted through the NIC
/// and is guaranteed to be contiguous in physical memory. 
/// Auto-dereferences into a `MappedPages` object that represents its underlying memory. 
pub struct PacketBuffer {
    pub mp: MappedPages,
    pub phys_addr: PhysicalAddress,
    pub length: u16,
}
impl PacketBuffer {
    /// Creates a new PacketBuffer with the specified size in bytes.
    /// The size is a `u16` because that is the maximum size of an NIC transmit buffer. 
    pub fn new(size_in_bytes: u16) -> Result<PacketBuffer, &'static str> {
        let (mp, starting_phys_addr) = create_contiguous_mapping(
            size_in_bytes as usize,
            EntryFlags::WRITABLE | EntryFlags::NO_CACHE | EntryFlags::NO_EXECUTE,
        )?;
        Ok(PacketBuffer {
            mp: mp,
            phys_addr: starting_phys_addr,
            length: size_in_bytes,
        })
    }
}

impl Deref for PacketBuffer {
    type Target = MappedPages;
    fn deref(&self) -> &MappedPages {
        &self.mp
    }
}
impl DerefMut for PacketBuffer {
    fn deref_mut(&mut self) -> &mut MappedPages {
        &mut self.mp
    }
}

/// A network (e.g., Ethernet) frame that has been received by the NIC.
pub struct ReceivedFrame(pub Vec<PacketBuffer>);
