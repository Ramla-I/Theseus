use prusti_contracts::*;
use core::ptr::Unique;

#[extern_spec]
impl<T> Unique<T>{
    #[pure]
    unsafe fn as_ref(&self) -> &T;
}