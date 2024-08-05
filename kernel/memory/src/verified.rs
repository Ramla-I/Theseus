use prusti_contracts::*;
#[allow(unused_imports)]
use prusti_external_spec::{trusted_option::*,trusted_result::*, mem::*, num::*};
use crate::MappedPages;
use core::slice;

#[derive(Clone, Copy)]
pub enum MPCastError {
    SizeOverflow,
    StartVaddrOverflow,
    EndBoundOverflow,
    MappingSizeOverflow,
    AlignmentError,
}

impl MPCastError {
    pub fn into_str(self) -> &'static str {
        match self {
            MPCastError::SizeOverflow => "MappedPages Cast Error: Overflow of size of type * length or size > isize::MAX",
            MPCastError::StartVaddrOverflow => "MappedPages Cast Error: Overflow of mp start address + byte offset",
            MPCastError::EndBoundOverflow => "MappedPages Cast Error: Overflow of byte offset + size of type",
            MPCastError::MappingSizeOverflow => "MappedPages Cast Error: The byte offset + size exceeds the size of the mapping",
            MPCastError::AlignmentError => "MappedPages Cast Error: The byte offset is unaligned with type alignment",
        }
    }
}

struct MPCastVAddr(usize);

pub(crate) fn cast_mp_as_ref<'a, T>(mp: &'a MappedPages, byte_offset: usize, req_alignment: Option<usize>) -> Result<&'a T, MPCastError>  {
    let vaddr = cast_mp_as::<T>(mp, byte_offset, core::mem::size_of::<T>(), req_alignment)?;
    Ok(cast_addr_as_const_ptr(mp, vaddr))
}

pub(crate) fn cast_mp_as_mut_ref<'a, T>(mp: &'a MappedPages, byte_offset: usize, req_alignment: Option<usize>) -> Result<&'a mut T, MPCastError>  {
    let vaddr = cast_mp_as::<T>(mp, byte_offset, core::mem::size_of::<T>(), req_alignment)?;
    Ok(cast_addr_as_mut_ptr(mp, vaddr))
}

pub(crate) fn cast_mp_as_slice<T>(mp: &MappedPages, byte_offset: usize, length: usize, req_alignment: Option<usize>) -> Result<&[T], MPCastError>  {
    let vaddr = cast_mp_as_slice_helper::<T>(mp, byte_offset, length, req_alignment)?;
    Ok(cast_addr_as_slice(mp, vaddr, length))
}

pub(crate) fn cast_mp_as_slice_mut<T>(mp: &MappedPages, byte_offset: usize, length: usize, req_alignment: Option<usize>) -> Result<&mut [T], MPCastError>  {
    let vaddr = cast_mp_as_slice_helper::<T>(mp, byte_offset, length, req_alignment)?;
    Ok(cast_addr_as_slice_mut(mp, vaddr, length))
}

#[verified]
#[requires(mp.start_address_trusted().value() % core::mem::align_of::<T>() == 0)]
fn cast_mp_as_slice_helper<T>(mp: &MappedPages, byte_offset: usize, length: usize, req_alignment: Option<usize>) -> Result<MPCastVAddr, MPCastError> {
    let size_in_bytes = length.checked_mul(core::mem::size_of::<T>());
    if size_in_bytes.is_none() {return Err(MPCastError::SizeOverflow);}
    
    let size_in_bytes = size_in_bytes.unwrap();
    if size_in_bytes > isize::MAX as usize { return Err(MPCastError::SizeOverflow); }

    cast_mp_as::<T>(mp, byte_offset, size_in_bytes, req_alignment)
}


#[verified]
#[requires(mp.start_address_trusted().value() % core::mem::align_of::<T>() == 0)]
#[ensures(result.is_ok() ==> {
    let start_vaddr = peek_result_ref(&result).0;
    start_vaddr >= mp.start_address_trusted().value()
        && start_vaddr + size_in_bytes <= mp.start_address_trusted().value() + mp.size_in_bytes_trusted()
        && start_vaddr % core::mem::align_of::<T>() == 0
})]
#[ensures(result.is_ok() && req_alignment.is_some() ==> peek_result_ref(&result).0 % peek_option(&req_alignment) == 0)]
fn cast_mp_as<T>(mp: &MappedPages, byte_offset: usize, size_in_bytes: usize, req_alignment: Option<usize>) -> Result<MPCastVAddr, MPCastError>  {
    let mp_addr = mp.start_address_trusted().value();
    let mp_size_in_bytes = mp.size_in_bytes_trusted();

    if byte_offset % core::mem::align_of::<T>() != 0 { return Err(MPCastError::AlignmentError); }

    let start_vaddr = mp_addr.checked_add(byte_offset);
    if start_vaddr.is_none() { return Err(MPCastError::StartVaddrOverflow); }
    let start_vaddr = start_vaddr.unwrap();
    if let Some(req_alignment) = req_alignment {
        if start_vaddr % req_alignment != 0 { return Err(MPCastError::AlignmentError); }
    }

    // check that size of slice fits within the size of the mapping
    let end_bound = byte_offset.checked_add(size_in_bytes);
    if end_bound.is_none() { return Err(MPCastError::EndBoundOverflow); }

    if end_bound.unwrap() > mp_size_in_bytes { return Err(MPCastError::MappingSizeOverflow) }

    Ok(MPCastVAddr(start_vaddr))
}

#[trusted]
#[ensures(result as *const T as usize === start_vaddr.0)]
fn cast_addr_as_const_ptr<'a, T>(_mp: &'a MappedPages, start_vaddr: MPCastVAddr) -> &'a T {
    unsafe {
        &*(start_vaddr.0 as *const T)
    }
}

#[trusted]
#[ensures(result as *const T as usize === start_vaddr.0)]
fn cast_addr_as_mut_ptr<'a, T>(_mp: &'a MappedPages, start_vaddr: MPCastVAddr) -> &'a mut T {
    unsafe {
        &mut *(start_vaddr.0 as *mut T)
    }
}

#[trusted]
#[ensures(&result[0] as *const T as usize === start_vaddr.0)]
#[ensures(result.len() == length)]
fn cast_addr_as_slice<'a, T>(_mp: &'a MappedPages, start_vaddr: MPCastVAddr, length: usize) -> &'a [T] {
    unsafe {
        slice::from_raw_parts(start_vaddr.0 as *const T, length)
    }
}

#[trusted]
#[ensures(&result[0] as *const T as usize === start_vaddr.0)]
#[ensures(result.len() == length)]
fn cast_addr_as_slice_mut<'a, T>(_mp: &'a MappedPages, start_vaddr: MPCastVAddr, length: usize) -> &'a mut [T] {
    unsafe {
        slice::from_raw_parts_mut(start_vaddr.0 as *mut T, length)
    }
}
