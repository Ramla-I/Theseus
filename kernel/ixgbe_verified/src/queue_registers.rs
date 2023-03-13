//! Structs which provide access to the ixgbe device queue registers and store their backing memory pages.
//! 
//! They implement the `RxQueueRegisters` and `TxQueueRegisters` traits which allows 
//! the registers to be accessed through virtual NICs

use super::hal::regs::{RegistersRx, RegistersTx};
use core::ops::{Deref, DerefMut};

cfg_if::cfg_if! {
if #[cfg(prusti)] {

use crate::spec::memory_spec::*;

} else {

use crate::mapped_pages_fragments::{MappedPagesFragments, Fragment};

}}

/// Struct that stores a pointer to registers for one ixgbe transmit queue
/// as well as a shared reference to the backing `MappedPages` where these registers are located.
pub(crate) struct TxQueueRegisters {
    /// the ID of the tx queue that these registers control
    id: usize,
    /// We prevent the drop handler from dropping the `regs` because the backing memory is not in the heap,
    /// but in the stored mapped pages. The memory will be deallocated when the `backing_pages` are dropped.
    regs: Fragment<RegistersTx>
}

impl Deref for TxQueueRegisters {
    type Target = Fragment<RegistersTx>;
    fn deref(&self) -> &Fragment<RegistersTx> {
        &self.regs
    }
}

impl DerefMut for TxQueueRegisters {
    fn deref_mut(&mut self) -> &mut Fragment<RegistersTx> {
        &mut self.regs
    }
}

/// Struct that stores a pointer to registers for one ixgbe receive queue
/// as well as a shared reference to the backing `MappedPages` where these registers are located.
pub struct RxQueueRegisters {
    /// the ID of the rx queue that these registers control
    id: usize,
    /// We prevent the drop handler from dropping the `regs` because the backing memory is not in the heap,
    /// but in the stored mapped pages. The memory will be deallocated when the `backing_pages` are dropped.
    regs: Fragment<RegistersRx>
}

impl Deref for RxQueueRegisters {
    type Target = Fragment<RegistersRx>;
    fn deref(&self) -> &Fragment<RegistersRx> {
        &self.regs
    }
}

impl DerefMut for RxQueueRegisters {
    fn deref_mut(&mut self) -> &mut Fragment<RegistersRx> {
        &mut self.regs
    }
}

cfg_if::cfg_if! {
if #[cfg(not(prusti))] {

impl TxQueueRegisters {
    pub fn new(queue_id: usize, mp: &mut MappedPagesFragments) -> Result<TxQueueRegisters, &'static str> {
        let fragment = mp.fragment(queue_id * core::mem::size_of::<RegistersTx>())?;

        Ok(TxQueueRegisters {
            id: queue_id,
            regs: fragment
        })
    }

    pub fn id(&self) -> usize {
        self.id
    }
}

impl RxQueueRegisters {
    pub fn new(queue_id: usize, mp: &mut MappedPagesFragments) -> Result<RxQueueRegisters, &'static str> {
        let fragment = mp.fragment(queue_id * core::mem::size_of::<RegistersRx>())?;

        Ok(RxQueueRegisters {
            id: queue_id,
            regs: fragment
        })
    }

    pub fn id(&self) -> usize {
        self.id
    }
}

}}