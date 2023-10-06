#![no_std]
use core::cmp::PartialEq;
use core::marker::Copy;

pub trait UniqueCheck: Copy + PartialEq {
    fn overlaps(&self, other: &Self) -> bool;
}
