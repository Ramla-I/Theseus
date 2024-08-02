use prusti_contracts::*;
use prusti_external_spec::{trusted_option::*,trusted_result::*, mem::*, num::*};

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
            MPCastError::SizeOverflow => "MP Cast Error: Overflow of size of type * length or size > isize::MAX",
            MPCastError::StartVaddrOverflow => "MP Cast Error: Overflow of mp start address + byte offset",
            MPCastError::EndBoundOverflow => "MP Cast Error: Overflow of byte offset + size of type",
            MPCastError::MappingSizeOverflow => "MP Cast Error: The byte offset + size exceeds the size of the mapping",
            MPCastError::AlignmentError => "MP Cast Error: The byte offset is unaligned with type alignment",
        }
    }
}

pub struct MPCastStartVaddr(usize);
impl core::ops::Deref for MPCastStartVaddr {
    type Target = usize;
    fn deref(&self) -> &usize {
        &self.0
    }
}

#[verified]
#[requires(mp_addr % core::mem::align_of::<T>() == 0)]
#[ensures(result.is_ok() ==> {
    let start_vaddr = peek_result_ref(&result).0;
    (start_vaddr >= mp_addr) && (start_vaddr + length * core::mem::size_of::<T>() <= mp_addr + mp_size_in_bytes)
})]
pub(crate) fn cast_mp_as_slice<T>(mp_addr: usize, mp_size_in_bytes: usize, byte_offset: usize, length: usize) -> Result<MPCastStartVaddr, MPCastError>  {
    let size_in_bytes = length.checked_mul(core::mem::size_of::<T>());
    if size_in_bytes.is_none() {return Err(MPCastError::SizeOverflow);}
    
    let size_in_bytes = size_in_bytes.unwrap();
    if size_in_bytes > isize::MAX as usize { return Err(MPCastError::SizeOverflow); }

    cast_mp_as::<T>(mp_addr, mp_size_in_bytes, byte_offset, size_in_bytes)
}

#[verified]
#[requires(mp_addr % core::mem::align_of::<T>() == 0)]
#[ensures(result.is_ok() ==> {
    let start_vaddr = peek_result_ref(&result).0;
    (start_vaddr >= mp_addr) && (start_vaddr + core::mem::size_of::<T>() <= mp_addr + mp_size_in_bytes)
})]
pub(crate) fn cast_mp_as_type<T>(mp_addr: usize, mp_size_in_bytes: usize, byte_offset: usize) -> Result<MPCastStartVaddr, MPCastError>  {
    let size_in_bytes = core::mem::size_of::<T>();
    cast_mp_as::<T>(mp_addr, mp_size_in_bytes, byte_offset, size_in_bytes)
}


#[verified]
#[requires(mp_addr % core::mem::align_of::<T>() == 0)]
#[ensures(result.is_ok() ==> {
    let start_vaddr = peek_result_ref(&result).0;
    (start_vaddr >= mp_addr) && (start_vaddr + size_in_bytes <= mp_addr + mp_size_in_bytes)
})]
fn cast_mp_as<T>(mp_addr: usize, mp_size_in_bytes: usize, byte_offset: usize, size_in_bytes: usize) -> Result<MPCastStartVaddr, MPCastError>  {
    if byte_offset % core::mem::align_of::<T>() != 0 { return Err(MPCastError::AlignmentError); }

    let start_vaddr = mp_addr.checked_add(byte_offset);
    if start_vaddr.is_none() { return Err(MPCastError::StartVaddrOverflow); }

    // check that size of slice fits within the size of the mapping
    let end_bound = byte_offset.checked_add(size_in_bytes);
    if end_bound.is_none() { return Err(MPCastError::EndBoundOverflow); }

    if end_bound.unwrap() > mp_size_in_bytes { return Err(MPCastError::MappingSizeOverflow) }

    Ok(MPCastStartVaddr(start_vaddr.unwrap()))
}

#[verified]
#[requires(mp_addr % core::mem::align_of::<T>() == 0)]
#[ensures(result.is_ok() ==> {
    let start_vaddr = peek_result_ref(&result).0;
    (start_vaddr >= mp_addr) && (start_vaddr + size_in_bytes <= mp_addr + mp_size_in_bytes)
})]
fn cast_mp_as2<T>(mp: crate::MappedPages, mp_addr: usize, mp_size_in_bytes: usize, byte_offset: usize, size_in_bytes: usize) -> Result<MPCastStartVaddr, MPCastError>  {
    if byte_offset % core::mem::align_of::<T>() != 0 { return Err(MPCastError::AlignmentError); }

    let start_vaddr = mp_addr.checked_add(byte_offset);
    if start_vaddr.is_none() { return Err(MPCastError::StartVaddrOverflow); }

    // check that size of slice fits within the size of the mapping
    let end_bound = byte_offset.checked_add(size_in_bytes);
    if end_bound.is_none() { return Err(MPCastError::EndBoundOverflow); }

    if end_bound.unwrap() > mp_size_in_bytes { return Err(MPCastError::MappingSizeOverflow) }

    Ok(MPCastStartVaddr(start_vaddr.unwrap()))
}