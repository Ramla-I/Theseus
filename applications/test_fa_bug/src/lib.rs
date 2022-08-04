#![no_std]
// #![feature(plugin)]
// #![plugin(application_main_fn)]


extern crate alloc;
// #[macro_use] extern crate log;
#[macro_use] extern crate terminal_print;
extern crate memory;
extern crate nic_initialization;

use alloc::vec::Vec;
use alloc::string::String;
use memory::{create_contiguous_mapping, PhysicalAddress};
use nic_initialization::{NIC_MAPPING_FLAGS, allocate_memory};

pub fn main(_args: Vec<String>) -> isize {
        
    // Allocate pages for RQ and SQ
    let (rq_mp, rq_pa) = create_contiguous_mapping(8192, NIC_MAPPING_FLAGS).unwrap();
    let sq_pa = PhysicalAddress::new(rq_pa.value() + 8192).ok_or("Could not create starting address for SQ").unwrap(); 
    let sq_mp = allocate_memory(sq_pa, 524288).unwrap();
            
    // Allocate page for SQ/RQ doorbell
    let (db_page, db_pa) = create_contiguous_mapping(4096, NIC_MAPPING_FLAGS).unwrap();
    println!("rq = {:#X}, sq = {:#X}, db = {:#X}", rq_pa, sq_pa, db_pa);

    assert_eq!(sq_pa, db_pa);
    0
}
