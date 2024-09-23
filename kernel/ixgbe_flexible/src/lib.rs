//! A restricted driver for a 82599 10GbE Network Interface Card.

#![no_std]
// #![allow(unaligned_references)] // temporary, just to suppress unsafe packed borrows 
#![allow(incomplete_features)] // to allow adt_const_params without a warning
#![feature(rustc_private)]
#![feature(ptr_internals)]

extern crate alloc;

mod hal;
mod ethernet_frame;
mod mempool;