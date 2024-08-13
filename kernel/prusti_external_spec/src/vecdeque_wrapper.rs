use prusti_contracts::*;
use alloc::collections::VecDeque;

pub struct VecDequeWrapper<T>( pub VecDeque<T> );

impl<T> VecDequeWrapper<T> {

    #[trusted]
    #[ensures(result.len() == 0)]
    pub fn new() -> Self {
        VecDequeWrapper( VecDeque::new() )
    }

    #[trusted]
    #[ensures(result.len() == 0)]
    pub fn with_capacity(capacity: usize) -> Self {
        VecDequeWrapper( VecDeque::with_capacity(capacity) )
    }

    #[trusted]
    #[pure]
    pub fn len(&self) -> usize {
        self.0.len()
    }

    #[trusted]
    #[pure]
    #[requires(index < self.len())]
    pub fn index(&self, index: usize) -> &T {
        &self.0[index]
    }

    #[trusted]
    #[ensures(self.len() == old(self.len()) + 1)]
    #[ensures(forall (|i: usize| i < old(self.len()) ==> {
        self.index(i) === old(self.index(i))
    }))]
    #[ensures({
        let idx = self.len() - 1;
        *self.index(idx) === value
    })]
    pub fn push_back(&mut self, value: T) {
        self.0.push_back(value);
    }


    #[trusted]
    pub fn pop_front(&mut self) -> Option<T> {
        self.0.pop_front()
    }
}


#[cfg(not(prusti))]
use core::ops::{Deref, DerefMut};

#[cfg(not(prusti))]
impl<T> Deref for VecDequeWrapper<T> {
    type Target = VecDeque<T>;

    fn deref(&self) -> &VecDeque<T> {
        &self.0
    }
}

#[cfg(not(prusti))]
impl<T> DerefMut for VecDequeWrapper<T> {
    fn deref_mut(&mut self) -> &mut VecDeque<T> {
        &mut self.0
    }
}