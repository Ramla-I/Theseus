//! Structs which provide access to the ixgbe device queue registers.
//! We separate the queue registers from the rest of the register structs so that we can place
//! the registers as a field of the RxQueue/ TxQueue type.
//! This way the RxQueue/ TxQueue contains all the required resources to receive/ send a packet on that queue.
//! Each queue can be used independently of each other.
//! 
//! We also prefer this struct as a way to keep the queue ID together with the registers as the queue ID
//! is used in many places.

use super::hal::regs::{RegistersRx, RegistersTx};
use prusti_borrowed_shared_mp::BorrowedSharedMappedPages;
use core::ops::{Deref, DerefMut};

/// Struct that stores a pointer to registers for one ixgbe transmit queue
/// as well as a shared reference to the backing `MappedPages` where these registers are located.
pub(crate) struct TxQueueRegisters {
    /// the ID of the tx queue that these registers control
    /// To Do: use queue ID enum?
    pub(crate) id: usize,
    pub(crate) regs: BorrowedSharedMappedPages<RegistersTx>
}

impl TxQueueRegisters {
    pub fn id(&self) -> usize {
        self.id
    }
}

impl Deref for TxQueueRegisters {
    type Target = RegistersTx;
    fn deref(&self) -> &Self::Target {
        self.regs.deref()
    }
}

impl DerefMut for TxQueueRegisters {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.regs.deref_mut()
    }
}


/// Struct that stores a pointer to registers for one ixgbe receive queue
/// as well as a shared reference to the backing `MappedPages` where these registers are located.
pub struct RxQueueRegisters {
    /// the ID of the rx queue that these registers control
    pub(crate) id: usize,
    pub(crate) regs: BorrowedSharedMappedPages<RegistersRx>
}

impl RxQueueRegisters {
    pub fn id(&self) -> usize {
        self.id
    }
}

impl Deref for RxQueueRegisters {
    type Target = RegistersRx;
    fn deref(&self) -> &Self::Target {
        self.regs.deref()
    }
}

impl DerefMut for RxQueueRegisters {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.regs.deref_mut()
    }
}
