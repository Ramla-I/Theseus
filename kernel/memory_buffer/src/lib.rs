#![no_std]
#![feature(ptr_internals)]

extern crate alloc;

use alloc::collections::VecDeque;
use log::warn;
use memory::{MappedPages, VirtualAddress};
use prusti_representation_creator::*;
use resource_identifier::ResourceIdentifier;
use core::ptr::Unique;
use core::{cmp::{max, min}, ops::{DerefMut, Deref}};

// pub mod verified;

pub struct Buffer<T>(Unique<T>);
impl<T> Deref for Buffer<T> {
    type Target = T;
    fn deref(&self) -> &Self::Target {
        unsafe{ self.0.as_ref() }
    }
}

impl<T> DerefMut for Buffer<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe{ self.0.as_mut() }
    }
}

#[derive(PartialEq, Copy, Clone)]
struct BufferInfo {
    start_addr: VirtualAddress,
    end_addr:   VirtualAddress, // inclusive
    min_addr:   VirtualAddress, 
    max_addr:   VirtualAddress, // inclusive
}

impl BufferInfo {
    fn new(start_addr: VirtualAddress, end_addr: VirtualAddress, mp: &MappedPages) -> Option<Self> {
        if start_addr < mp.start_address() || end_addr >= mp.start_address() + mp.size_in_bytes() {
            None
        } else {
            Some(BufferInfo { 
                start_addr, 
                end_addr, 
                min_addr: mp.start_address(), 
                max_addr: mp.start_address() + mp.size_in_bytes() - 1
            })
        }
    }
}

impl ResourceIdentifier for BufferInfo {
    fn overlaps(&self, other: &Self) -> bool {
        self.min_addr == other.min_addr && self.max_addr == other.max_addr
        &&  max(self.start_addr, other.start_addr) <= min(self.end_addr,   other.end_addr)
    }
}

/// Will ensure single ownership but not lifetime guarantee
/// The backing MP should never be freed until the buffers are also freed
pub struct BufferCreator<R> {
    rep_creator: RepresentationCreator<BufferInfo, Buffer<R>>,
    backing_mp: MappedPages,
    pub buffers: VecDeque<Buffer<R>>
}

impl<R> BufferCreator<R> {
    // No way to verify this with Prusti because of unsafe code
    // could use kani?
    fn create_buffer(id: &BufferInfo) -> Buffer<R> {
        unsafe{ Buffer(Unique::new_unchecked(id.start_addr.value() as *mut R)) }
    }
    
    // pub fn new(backing_mp: MappedPages) -> Self {
    //     BufferCreator {
    //         rep_creator: RepresentationCreator::new(Self::create_buffer, false),
    //         backing_mp,
    //         buffers: Vec::new()
    //     }
    // }

    pub fn create_buffers_from_mp(backing_mp: MappedPages, num_buffers: usize, buffer_size: usize) -> Result<BufferCreator<R>, (MappedPages, &'static str)> {
        let mut rep_creator = RepresentationCreator::new(Self::create_buffer, false);

        let mut start_addr = backing_mp.start_address();
        let mut buffers = VecDeque::with_capacity(num_buffers);

        for _ in 0..num_buffers {
            let buffer_info = BufferInfo::new(start_addr, start_addr + buffer_size - 1, &backing_mp);
            if buffer_info.is_none() { 
                return Err((backing_mp, "Buffer would be out of bounds of the backing memory"));
            }

            let buffer = rep_creator.create_unique_representation(buffer_info.unwrap()).map(|(buffer, _)| buffer);
            match buffer {
                Ok(x) => {
                    buffers.push_back(x);
                    start_addr += buffer_size;
                },
                Err(_) => return Err((backing_mp, "Failed to create buffer"))
            }
        }

        Ok( BufferCreator {
            rep_creator,
            backing_mp,
            buffers
        })
    }

}

impl<R> Drop for BufferCreator<R> {
    fn drop(&mut self) {
        let mut all_buffers_present = true;
        for i in 0..self.rep_creator.list_len() {
            let buffer_info = self.rep_creator.list_lookup(i);
            all_buffers_present &= self.buffers.iter().find(|buffer| buffer_info.start_addr.value() == buffer.0.as_ptr() as usize).is_some();
        }

        if !all_buffers_present {
            warn!("Not all buffers were present in the list so dropping the backing mp would lead to a use-after-free");
            let backing_mp = core::mem::replace(&mut self.backing_mp, MappedPages::empty());
            core::mem::forget(backing_mp); // we will have a memory leak rather than a use-after-free
        }
    }
}