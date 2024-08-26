use volatile::Volatile;
use zerocopy::FromBytes;
use core::ops::Deref;

#[derive(FromBytes)]
#[repr(C)]
pub struct TransmitHead( Volatile<u32> );
impl Deref for TransmitHead {
    type Target = Volatile<u32>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl TransmitHead {
    pub fn clear(&mut self) {
        self.0.write(0);
    }
}