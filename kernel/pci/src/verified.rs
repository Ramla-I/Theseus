//! This file is not supposed to be compiled.
//! It is just and easy way to measure the lines of verified code.
//! it's everything that doesn't lie in the not(prusti) cfg flag

#![no_std]
#![allow(dead_code)]
#![feature(rustc_private)]

extern crate alloc;

use prusti_contracts::*;
use core::mem::size_of;
use prusti_representation_creator::resource_identifier::ResourceIdentifier;

/// The bus, slot, and function number of a given PCI device.
/// This offers methods for reading and writing the PCI config space. 
#[derive(Copy, Clone, PartialEq, Eq, Hash)]
#[invariant(self.slot < MAX_SLOTS_PER_BUS && self.func < MAX_FUNCTIONS_PER_SLOT)]
pub struct PciLocation {
    bus:  u8, // 8 bits
    slot: u8, // 5 bits (aka device)
    func: u8, // 3 bits
}

impl ResourceIdentifier for PciLocation {
    #[pure]
    fn overlaps(&self,other: &Self) -> bool {
        self.bus == other.bus && self.slot == other.slot && self.func == other.func
    }
}

enum PciLocationError {
    InvalidSlot(u8),
    InvalidFunction(u8),
}

impl PciLocation {
    fn new(bus: u8, slot: u8, func: u8) -> Result<PciLocation, PciLocationError> {
        if slot >= MAX_SLOTS_PER_BUS {
            Err(PciLocationError::InvalidSlot(slot))
        } else if func >= MAX_FUNCTIONS_PER_SLOT {
            Err(PciLocationError::InvalidFunction(func))
        } else {
            Ok(PciLocation { bus, slot, func})
        }
    }
}