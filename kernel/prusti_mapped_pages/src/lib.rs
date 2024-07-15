//! To verify this crate, in the directory with the Cargo.toml file, run the cargo-prusti executable:
//! "../../../prusti-release-2023-08-22/cargo-prusti"
//! The Prusti.toml file in this directory contains the configuration for the verification.

#![no_std]
#![allow(dead_code)]
#![allow(unused_imports)]

extern crate prusti_contracts;
extern crate prusti_external_spec;
extern crate alloc;

use prusti_contracts::*;
use prusti_external_spec::{trusted_option::*,trusted_result::*, mem::*, num::*};

pub enum MPCastError {
    SizeOverflow,
    StartVaddrOverflow,
    EndBoundOverflow,
    MappingSizeOverflow,
    AlignmentError,
}

struct MPCastStartVaddr(usize);

#[requires(core::mem::align_of::<T>() != 0)]
#[ensures(result.is_ok() ==> {
    let ptr = peek_result_ref(&result);
    (ptr.0 >= mp_addr) && (ptr.0 + length * core::mem::size_of::<T>() <= mp_addr + mp_size_in_bytes)
})]
fn cast_mp_as_slice<T>(mp_addr: usize, mp_size_in_bytes: usize, byte_offset: usize, length: usize) -> Result<MPCastStartVaddr, MPCastError>  {
    let size_in_bytes = length.checked_mul(core::mem::size_of::<T>());
    if size_in_bytes.is_none() {return Err(MPCastError::SizeOverflow);}
    let size_in_bytes = size_in_bytes.unwrap();

    if size_in_bytes > isize::MAX as usize {
        return Err(MPCastError::SizeOverflow);
    }

    if byte_offset % core::mem::align_of::<T>() != 0 {
        return Err(MPCastError::AlignmentError);
    }

    let start_vaddr = mp_addr.checked_add(byte_offset);

    if start_vaddr.is_none() {
        return Err(MPCastError::StartVaddrOverflow);
    }

    // check that size of slice fits within the size of the mapping
    let end_bound = byte_offset.checked_add(size_in_bytes);

    if end_bound.is_none() {
        return Err(MPCastError::EndBoundOverflow);
    }

    if end_bound.unwrap() > mp_size_in_bytes { return Err(MPCastError::MappingSizeOverflow) }

    Ok(MPCastStartVaddr(start_vaddr.unwrap()))

}
