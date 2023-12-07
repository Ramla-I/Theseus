//! The specification for the `Option` type and helper functions to peek the inner value.

use prusti_contracts::*;

#[extern_spec]
impl<T: PartialEq + Copy> core::option::Option<T> {
    #[requires(self.is_some())]
    #[ensures(old(self) === Some(result))]
    pub fn unwrap(self) -> T;
    
    #[pure]
    #[ensures(result == matches!(self, None))]
    #[ensures(self.is_some() == !result)]
    pub const fn is_none(&self) -> bool;

    #[pure]
    #[ensures(result == matches!(self, Some(_)))]
    #[ensures(self.is_none() == !result)]
    pub const fn is_some(&self) -> bool;

    #[ensures(result === old(snap(self)))]
    #[ensures(self.is_none())]
    pub fn take(&mut self) -> Option<T>;
}

#[extern_spec]
impl<T: Copy + PartialEq> PartialEq for core::option::Option<T> {
    #[pure]
    #[ensures(self.is_none() && other.is_none() ==> result == true)]
    #[ensures(self.is_none() && other.is_some() ==> result == false)]
    #[ensures(self.is_some() && other.is_none() ==> result == false)]
    #[ensures(self.is_some() && other.is_some() ==> {
        let val1 = peek_option(&self);
        let val2 = peek_option(&other);
        result == (val1 == val2)
    })]
    fn eq(&self, other:&Self) -> bool;
}

#[pure]
#[requires(val.is_some())]
pub fn peek_option<T: Copy>(val: &Option<T>) -> T {
    match val {
        Some(val) => *val,
        None => unreachable!(),
    }
}

#[pure]
#[requires(val.is_some())]
pub fn peek_option_ref<T>(val: &Option<T>) -> &T {
    match val {
        Some(val) => val,
        None => unreachable!(),
    }
}