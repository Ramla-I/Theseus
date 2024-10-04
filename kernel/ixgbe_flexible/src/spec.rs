use prusti_contracts::*;
use memory_structs::PhysicalAddress;
use volatile::Volatile;
use crate::mempool::PktBuff;

/// I should not have to write this...
/// need to complain to prusti developers
#[pure]
#[trusted]
pub fn value(addr: PhysicalAddress) -> usize {
    addr.value()
}


#[extern_spec]
impl<T: Copy> Volatile<T>{
    #[pure]
    pub fn read(&self) -> T;

    #[ensures(self.read() === val)]
    pub fn write(&mut self, val: T);
}


#[pure]
#[trusted]
pub fn pktbuff_addr(buff: &PktBuff) -> PhysicalAddress {
    buff.paddr
}