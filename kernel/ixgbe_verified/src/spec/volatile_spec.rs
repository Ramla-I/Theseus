pub struct Volatile<T: Copy>{
    inner: T
}

impl<T: Copy> Volatile<T> {
    pub fn write(&mut self, val: T) {
        self.inner = val;
    }

    pub fn read(&self) -> T {
        self.inner
    }
}