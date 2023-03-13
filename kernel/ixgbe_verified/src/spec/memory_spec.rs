use prusti_contracts::*;

use spec::range_inclusive_spec::*;
use core::ops::{Deref, DerefMut};
use alloc::boxed::Box;

#[derive(Clone, Copy)]
pub struct PhysicalAddress(usize);
impl PhysicalAddress {
    #[pure]
    pub fn value(&self) -> usize {
        self.0
    }
}

impl core::cmp::PartialEq for PhysicalAddress {
    #[pure]
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

pub struct Frame(usize);

pub struct AllocatedPages(RangeInclusive<usize>);

pub struct EntryFlags(u64);

pub struct MappedPages {
    page_table_p4: Frame,
    pages: AllocatedPages,
    flags: EntryFlags
}


pub struct Fragment<T> {
    pub(crate) ptr: Box<T>,
}

impl<T> Deref for Fragment<T> {
    type Target = Box<T>;

    fn deref(&self) -> &Self::Target {
        &self.ptr
    }
}

impl<T> DerefMut for Fragment<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.ptr
    }
}

pub struct PacketBufferS {
    pub(crate) mp: MappedPages,
    pub(crate) phys_addr: PhysicalAddress,
    pub(crate) length: u16,
}

impl core::cmp::PartialEq for PacketBufferS {
    #[pure]
    fn eq(&self, other: &Self) -> bool {
        self.phys_addr.0 == other.phys_addr.0
    }
}