use prusti_contracts::*;
use alloc::vec::Vec;
use crate::trusted_option::*;

pub struct VecWrapper<T>(pub Vec<T>);

impl<T> VecWrapper<T> {

    #[trusted]
    #[ensures(result.len() == 0)]
    pub fn new() -> Self {
        VecWrapper( Vec::new() ) 
    }

    #[trusted]
    #[ensures(result.len() == 0)]
    pub fn with_capacity(capacity: usize) -> Self {
        VecWrapper( Vec::with_capacity(capacity) )
    }

    #[pure]
    #[trusted]
    pub fn len(&self) -> usize {
        self.0.len()
    }

    #[pure]
    #[trusted]
    #[requires(index < self.len())]
    #[inline(always)]
    pub fn index(&self, index: usize) -> &T {
        &self.0[index]
    }

    #[trusted]
    #[requires(index < self.len())]
    #[after_expiry(
        self.len() == old(self.len()) &&
        self.index(index) === before_expiry(result) &&
        forall(
            |i: usize| (i < self.len() && i != index) ==>
            self.index(i) === old(self.index(i))
        )
    )]
    #[inline(always)]
    pub fn index_mut(&mut self, index: usize) -> &mut T {
        &mut self.0[index]
    }

    #[trusted]
    #[ensures(self.len() == old(self.len()) + 1)]
    #[ensures(forall (|i: usize| i < old(self.len()) ==> { self.index(i) === old(self.index(i))}))]
    #[ensures({
        let idx = self.len() - 1;
        *self.index(idx) === value
    })]
    #[inline(always)]
    pub fn push(&mut self, value: T) {
        self.0.push(value);
    }

    #[trusted]
    #[ensures(result.is_some() ==> self.len() == old(self.len()) - 1)]
    #[ensures(result.is_none() ==> self.len() == old(self.len()))]
    #[ensures(forall (|i: usize| i < self.len() ==> { self.index(i) === old(self.index(i))}))]
    #[ensures(result.is_some() ==> peek_option_ref(&result) === old(self.index(self.len() - 1)))]
    #[inline(always)]
    pub fn pop(&mut self) -> Option<T> {
        self.0.pop()
    }
}
