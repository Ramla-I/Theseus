#![no_std]
#![feature(ptr_internals)]
#![allow(dead_code)]

#[macro_use(private_fields)]
extern crate proc_static_assertions;
extern crate alloc;
pub mod verified;

use memory::MappedPages;
use core::ptr::Unique;
use core::ops::{DerefMut, Deref};
use core::cmp::Ordering;
use prusti_contracts::*;
use zerocopy::FromBytes;
use alloc::sync::Arc;

/// A borrowed [`MappedPages`] object that is shared, and a unique address range derefs 
/// to `&T` and `&mut T`.
/// The [`MappedPages`] needs to be mutable.
///
/// ## Type parameters
/// 1. `T: FromBytes`: the same parameter used in [`MappedPages::as_type()`] functions.
///
/// ## Drop behavior
/// * When dropped, the borrow ends and the contained `Arc<MappedPages>` is dropped.

#[private_fields("ptr", "mp")]
pub struct BorrowedSharedMappedPages<T>
where
    T: FromBytes,
{
    ptr: Unique<T>,
    mp: Arc<MappedPages>,
}

impl<T: FromBytes> BorrowedSharedMappedPages<T> {
    /// Consumes this object and returns the inner `MappedPages` value
    /// (more specifically, the `Borrow`-able container holding the `MappedPages`).
    pub fn into_inner(self) -> Arc<MappedPages> {
        self.mp
    }

    /// Returns a reference to the inner `MappedPages` value
    /// (more specifically, the `Borrow`-able container holding the `MappedPages`).
    pub fn inner_ref(&self) -> &Arc<MappedPages> {
        &self.mp
    }
}

impl<T: FromBytes> Deref for BorrowedSharedMappedPages<T> {
    type Target = T;
    fn deref(&self) -> &T {
        // SAFETY:
        // ✅ The pointer is properly aligned; its alignment has been checked in `MappedPages::as_type()`.
        // ✅ The pointer is dereferenceable; it has been bounds checked by `MappedPages::as_type()`.
        // ✅ The pointer has been initialized in the constructor `from()`.
        // ✅ The lifetime of the returned reference `&T` is tied to the lifetime of the `MappedPages`,
        //     ensuring that the `MappedPages` object will persist at least as long as the reference.
        unsafe { self.ptr.as_ref() }
    }
}

impl<T: FromBytes> DerefMut for BorrowedSharedMappedPages<T> {
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY:
        // ✅ Same as the above `Deref` block, plus:
        // ✅ The underlying `MappedPages` is guaranteed to be writable by `MappedPages::as_type_mut()`.
        unsafe { self.ptr.as_mut() }
    }
}

impl<T: FromBytes> AsRef<T> for BorrowedSharedMappedPages<T> {
    fn as_ref(&self) -> &T { self.deref() }
}

impl<T: FromBytes> AsMut<T> for BorrowedSharedMappedPages<T> {
    fn as_mut(&mut self) -> &mut T { self.deref_mut() }
}

// Forward the impls of `PartialEq`, `Eq`, `PartialOrd`, `Ord`, and `Hash`.
impl<T: FromBytes + PartialEq> PartialEq for BorrowedSharedMappedPages<T> {
    fn eq(&self, other: &Self) -> bool { self.deref().eq(other.deref()) }
}
impl<T: FromBytes + Eq> Eq for BorrowedSharedMappedPages<T> { }
impl<T: FromBytes + PartialOrd> PartialOrd for BorrowedSharedMappedPages<T> {
    #[pure] // with opt_in_verification enabled I get a complaint if this isn't here
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> { self.deref().partial_cmp(other.deref()) }
}
impl<T: FromBytes + Ord> Ord for BorrowedSharedMappedPages<T> {
    fn cmp(&self, other: &Self) -> Ordering { self.deref().cmp(other.deref()) }
}

