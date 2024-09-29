#![no_std]
#![allow(dead_code)]
#![allow(unused_imports)]
#![feature(ptr_internals)]

extern crate prusti_contracts;
extern crate alloc;

pub mod trusted_option;
pub mod trusted_result;
pub mod partial_ord;
pub mod cmp;
pub mod mem;
pub mod num;
pub mod ptr;
pub mod vecdeque_wrapper;
pub mod vec_wrapper;
