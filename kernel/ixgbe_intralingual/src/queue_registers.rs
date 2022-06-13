//! Structs which provide access to the ixgbe device queue registers and store their backing memory pages.
//! 
//! They implement the `RxQueueRegisters` and `TxQueueRegisters` traits which allows 
//! the registers to be accessed through virtual NICs

use crate::mapped_pages_fragments::{MappedPagesFragments, Fragment};
use super::regs::{RegistersRx, RegistersTx};
use alloc::{
    sync::Arc,
    boxed::Box
};
use core::ops::{Deref, DerefMut};
use core::mem::ManuallyDrop;
use nic_queues::{TxQueueRegisters};
use memory::MappedPages;


/// Struct that stores a pointer to registers for one ixgbe receive queue
/// as well as a shared reference to the backing `MappedPages` where these registers are located.
pub struct IxgbeRxQueueRegisters {
    /// We prevent the drop handler from dropping the `regs` because the backing memory is not in the heap,
    /// but in the stored mapped pages. The memory will be deallocated when the `backing_pages` are dropped.
    pub regs: ManuallyDrop<Box<RegistersRx>>,
    pub backing_pages: Arc<MappedPages>
}

// impl RxQueueRegisters for IxgbeRxQueueRegisters {
//     fn set_rdbal(&mut self, value: u32) {
//         self.regs.rdbal.write(value)
//     }    
//     fn set_rdbah(&mut self, value: u32) {
//         self.regs.rdbah.write(value)
//     }
//     fn set_rdlen(&mut self, value: u32) {
//         self.regs.rdlen.write(value)
//     }
//     fn set_rdh(&mut self, value: u32) {
//         self.regs.rdh.write(value)
//     }
//     fn set_rdt(&mut self, value: u32) {
//         self.regs.rdt.write(value)
//     }
// }
impl Deref for IxgbeRxQueueRegisters {
    type Target = Box<RegistersRx>;
    fn deref(&self) -> &Box<RegistersRx> {
        &self.regs
    }
}
impl DerefMut for IxgbeRxQueueRegisters {
    fn deref_mut(&mut self) -> &mut Box<RegistersRx> {
        &mut self.regs
    }
}

/// Struct that stores a pointer to registers for one ixgbe transmit queue
/// as well as a shared reference to the backing `MappedPages` where these registers are located.
pub struct IxgbeTxQueueRegisters {
    /// the ID of the tx queue that these registers control
    id: usize,
    /// We prevent the drop handler from dropping the `regs` because the backing memory is not in the heap,
    /// but in the stored mapped pages. The memory will be deallocated when the `backing_pages` are dropped.
    regs: Fragment<RegistersTx>
}

impl IxgbeTxQueueRegisters {
    pub fn new(queue_id: usize, mp: &mut MappedPagesFragments) -> Result<IxgbeTxQueueRegisters, &'static str> {
        let fragment = mp.fragment(queue_id * core::mem::size_of::<RegistersTx>())?;

        Ok(IxgbeTxQueueRegisters {
            id: queue_id,
            regs: fragment
        })
    }

    pub fn id(&self) -> usize {
        self.id
    }
}

impl TxQueueRegisters for IxgbeTxQueueRegisters {
    fn set_tdbal(&mut self, value: u32) {
        self.regs.tdbal.write(value)
    }  
    fn set_tdbah(&mut self, value: u32) {
        self.regs.tdbah.write(value)
    }
    fn set_tdlen(&mut self, value: u32) {
        self.regs.tdlen.write(value)
    }
    fn set_tdh(&mut self, value: u32) {
        self.regs.tdh.write(value)
    }
    fn set_tdt(&mut self, value: u32) {
        self.regs.tdt.write(value)
    }
}

impl Deref for IxgbeTxQueueRegisters {
    type Target = Fragment<RegistersTx>;
    fn deref(&self) -> &Fragment<RegistersTx> {
        &self.regs
    }
}

impl DerefMut for IxgbeTxQueueRegisters {
    fn deref_mut(&mut self) -> &mut Fragment<RegistersTx> {
        &mut self.regs
    }
}

