//! To Do: Check that MP is mutable or add mutability requirement

use crate::BorrowedSharedMappedPages;
use memory::MappedPages;
use core::ptr::Unique;
use alloc::sync::Arc;
use zerocopy::FromBytes;
use prusti_contracts::*;
#[allow(unused_imports)]
use prusti_external_spec::{vecdeque_wrapper::*, trusted_result::*};
#[allow(unused_imports)]
use memory_structs::VirtualAddress; // we need this line so that prusti knows that the value() fn is pure

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
#[ensures(result.is_ok() ==> {
    let buffers = &peek_result_ref(&result);
    let start_addr = mp.start_address_trusted().value();
    buffers.len() == num_buffers 
    && forall (|i: usize| i < buffers.len() ==> addr(buffers.index(i)) === start_addr + i * core::mem::size_of::<T>() )
})]
pub fn create_buffers_from_mp<T: FromBytes>(mp: MappedPages, num_buffers: usize) -> Result<VecDequeWrapper<BorrowedSharedMappedPages<T>>, (MappedPages, BufferCreationError)> {
    let size_in_bytes = num_buffers.checked_mul(core::mem::size_of::<T>());
    if size_in_bytes.is_none() { return Err((mp, BufferCreationError::SizeOverflow)); }
    
    let size_in_bytes = size_in_bytes.unwrap();
    if size_in_bytes > mp.size_in_bytes_trusted() { return Err((mp, BufferCreationError::EndBoundOverflow)); }
    
    let mut buffers: VecDequeWrapper<BorrowedSharedMappedPages<T>> = VecDequeWrapper::with_capacity(num_buffers);

    // store the start address here for verification purposes. Otherwise we have to declare Arc::deref() as pure
    let mp_start_addr = mp.start_address_trusted().value();
    
    let mut i = 0;
    let mut start_addr = mp.start_address_trusted().value() + i * core::mem::size_of::<T>();
    let mp = Arc::new(mp);
    
    while i < num_buffers {
        body_invariant!(i < num_buffers);
        body_invariant!(buffers.len() == i);
        body_invariant!(start_addr == mp_start_addr + i * core::mem::size_of::<T>());
        body_invariant!(forall (|j: usize| j < buffers.len() ==> addr(buffers.index(j)) === mp_start_addr + j * core::mem::size_of::<T>()));
        
        buffers.push_back(create_buffer(start_addr, mp.clone()));
        i += 1;
        start_addr = mp_start_addr + i * core::mem::size_of::<T>();
    }

    Ok(buffers)
}



#[trusted]
#[pure]
fn addr<T:FromBytes>(buffer: &BorrowedSharedMappedPages<T>) -> usize {
    Unique::as_ptr(buffer.ptr) as usize
}

#[verified]
#[trusted]
#[ensures(addr(&result) == start_addr)]
fn create_buffer<T: FromBytes>(start_addr: usize, mp: Arc<MappedPages>) -> BorrowedSharedMappedPages<T> {
    BorrowedSharedMappedPages {
        ptr: unsafe{ Unique::new_unchecked(start_addr as *mut T) },
        mp
    }
}


