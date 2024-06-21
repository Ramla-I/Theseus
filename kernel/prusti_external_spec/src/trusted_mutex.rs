//! A simple mutex implementation that can be used for verification by Prusti when the actual working of the mutex does not matter.
//! I added this for verification of portions of the PCI crate, and the actual implementation of the mutex is not important for the verification.

pub struct Mutex<T> {
    value: T,
}

impl<T> Mutex<T> {
    pub const fn new(value: T) -> Self {
        Mutex { value }
    }

    pub fn lock(&self) -> &T {
        &self.value
    }
}