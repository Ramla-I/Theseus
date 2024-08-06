use memory::MappedPages;
use memory_structs::VirtualAddress;
use core::ptr::Unique;
use core::ops::{DerefMut, Deref};
use prusti_contracts::*;
use prusti_external_spec::{vecdeque_wrapper::*, trusted_result::*};
use log::warn;


#[derive(Clone, Copy)]
pub enum BufferCreationError {
    SizeOverflow,
    StartVaddrOverflow,
    EndBoundOverflow,
    MappingSizeOverflow,
    AlignmentError,
}

impl BufferCreationError {
    pub fn into_str(self) -> &'static str {
        match self {
            BufferCreationError::SizeOverflow => "Buffer Creation Error: Overflow of size of type * length or size > isize::MAX",
            BufferCreationError::StartVaddrOverflow => "Buffer Creation Error: Overflow of mp start address + byte offset",
            BufferCreationError::EndBoundOverflow => "Buffer Creation Error: Overflow of byte offset + size of type",
            BufferCreationError::MappingSizeOverflow => "Buffer Creation Error: The byte offset + size exceeds the size of the mapping",
            BufferCreationError::AlignmentError => "Buffer Creation Error: The byte offset is unaligned with type alignment",
        }
    }
}



#[verified]
#[ensures(result.is_ok() ==> peek_result_ref(&result).num_buffers == num_buffers)]
#[ensures(result.is_ok() ==> {
    let buffers = &peek_result_ref(&result).buffers;
    let start_addr = mp.start_address_trusted().value();
    buffers.len() == num_buffers 
    && forall (|i: usize| i < buffers.len() ==> buffers.index(i).addr() === start_addr + i * core::mem::size_of::<T>() )
})]
pub fn create_buffers_from_mp<T>(mp: MappedPages, num_buffers: usize) -> Result<BufferBackingStore<T>, (MappedPages, BufferCreationError)> {
    let size_in_bytes = num_buffers.checked_mul(core::mem::size_of::<T>());
    if size_in_bytes.is_none() { return Err((mp, BufferCreationError::SizeOverflow)); }
    
    let size_in_bytes = size_in_bytes.unwrap();
    if size_in_bytes > mp.size_in_bytes_trusted() { return Err((mp, BufferCreationError::EndBoundOverflow)); }
    
    let mut buffers: VecDequeWrapper<Buffer<T>> = VecDequeWrapper::with_capacity(num_buffers);
    
    let mut i = 0;
    let mut start_addr = mp.start_address_trusted().value() + i * core::mem::size_of::<T>();
    while i < num_buffers {
        body_invariant!(i < num_buffers);
        body_invariant!(buffers.len() == i);
        body_invariant!(start_addr == mp.start_address_trusted().value() + i * core::mem::size_of::<T>());
        body_invariant!(forall (|j: usize| j < buffers.len() ==> buffers.index(j).addr() === mp.start_address_trusted().value() + j * core::mem::size_of::<T>()));
        
        // addrs.push_back(start_addr);
        buffers.push_back(create_buffer(start_addr));
        i += 1;
        start_addr = mp.start_address_trusted().value() + i * core::mem::size_of::<T>();
    }

    Ok( BufferBackingStore {
        mp,
        num_buffers,
        buffers: buffers,
    })
}
pub struct BufferBackingStore<T> {
    mp: MappedPages,
    num_buffers: usize,
    pub buffers: VecDequeWrapper<Buffer<T>>,
}

impl<T> Drop for BufferBackingStore<T> {
    fn drop(&mut self) {
        let mut all_buffers_present = true;
        for i in 0..self.num_buffers {
            let buffer_start_addr = self.mp.start_address().value() + i * core::mem::size_of::<T>();
            all_buffers_present &= self.buffers.0.iter().find(|buffer| buffer.0.as_ptr() as usize == buffer_start_addr).is_some();
        }

        if !all_buffers_present {
            warn!("Not all buffers were present in the list so dropping the backing mp would lead to a use-after-free");
            let mp = core::mem::replace(&mut self.mp, MappedPages::empty());
            core::mem::forget(mp); // we will have a memory leak rather than a use-after-free
        }
    }
}

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

impl<T> Buffer<T> {
    #[trusted]
    #[pure]
    fn addr(&self) -> usize {
        Unique::as_ptr(self.0) as usize
    }
}

#[verified]
#[trusted]
#[ensures(result.addr() == start_addr)]
fn create_buffer<T>(start_addr: usize) -> Buffer<T> {
    unsafe{ Buffer(Unique::new_unchecked(start_addr as *mut T)) }
}


