use memory::{MappedPages};
use crate::{hal::descriptors::AdvancedTxDescriptor, packet_buffers::PacketBufferS};
use crate::queue_registers::TxQueueRegisters;
use crate::NumDesc;
use crate::allocator::*;
use owning_ref::BoxRefMut;
use core::{ops::{DerefMut, Deref}};
use alloc::{
    vec::Vec, collections::VecDeque,
};

pub type TxQueueE = TxQueue<{TxState::Enabled}>;
pub type TxQueueD = TxQueue<{TxState::Disabled}>;


/// A struct that holds all information for a transmit queue. 
/// There should be one such object per queue.
pub struct TxQueue<const S: TxState> {
    /// The number of the queue, stored here for our convenience.
    id: u8,
    /// Registers for this transmit queue
    pub(crate) regs: TxQueueRegisters,
    /// Transmit descriptors 
    pub(crate) tx_descs: BoxRefMut<MappedPages, [AdvancedTxDescriptor]>,
    /// The number of transmit descriptors in the descriptor ring
    num_tx_descs: u16,
    /// Current transmit descriptor index
    tx_cur: u16,
    /// The packet buffers that descriptors have stored information of
    tx_bufs_in_use: VecDeque<PacketBufferS>,
    /// first descriptor that has been used but not checked for transmit completion
    tx_clean: u16,
    /// The cpu which this queue is mapped to. 
    /// This in itself doesn't guarantee anything but we use this value when setting the cpu id for interrupts and DCA.
    cpu_id : Option<u8>
}

impl TxQueue<{TxState::Enabled}> {
    pub(crate) fn new(mut regs: TxQueueRegisters, num_desc: NumDesc, cpu_id: Option<u8>) -> Result<TxQueue<{TxState::Enabled}>, &'static str> {
        let (tx_descs, paddr) = create_desc_ring(num_desc)?;
        let num_tx_descs = tx_descs.len();

        // write the physical address of the tx descs array
        regs.tdbal.write(paddr.value() as u32); 
        regs.tdbah.write((paddr.value() >> 32) as u32); 

        // write the length (in total bytes) of the tx descs array
        regs.tdlen.write((num_tx_descs * core::mem::size_of::<AdvancedTxDescriptor>()) as u32);               
        
        // write the head index and the tail index (both 0 initially because there are no tx requests yet)
        regs.tdh.write(0);
        regs.tdt.write(0);

        Ok(TxQueue { id: regs.id() as u8, regs, tx_descs, num_tx_descs: num_tx_descs as u16, tx_cur: 0, tx_bufs_in_use: VecDeque::with_capacity(num_tx_descs), tx_clean: 0, cpu_id })
    }

    /// Sends a maximum of `batch_size` number of packets from the stored `buffers`.
    /// The number of packets sent are returned.
    pub fn tx_batch(&mut self, batch_size: usize,  buffers: &mut Vec<PacketBufferS>, used_buffers: &mut Vec<PacketBufferS>) -> Result<usize, &'static str> {
        let mut pkts_sent = 0;
        let mut tx_cur = self.tx_cur;

        self.tx_clean(used_buffers);
        let tx_clean = self.tx_clean;
        // debug!("tx_cur = {}, tx_clean ={}", tx_cur, tx_clean);

        for _ in 0.. batch_size {
            if let Some(packet) = buffers.pop() {
                let tx_next = (tx_cur + 1) % self.num_tx_descs;
    
                if tx_clean == tx_next {
                    // tx queue of device is full, push packet back onto the
                    // queue of to-be-sent packets
                    buffers.push(packet);
                    break;
                }
    
                self.tx_descs[tx_cur as usize].send(packet.phys_addr, packet.length);
                self.tx_bufs_in_use.push_back(packet);
    
                tx_cur = tx_next;
                pkts_sent += 1;
            } else {
                break;
            }
        }


        self.tx_cur = tx_cur;
        self.regs.tdt.write(tx_cur as u32);

        Ok(pkts_sent)
    }

    /// Removes multiples of `TX_CLEAN_BATCH` packets from `queue`.    
    /// (code taken from https://github.com/ixy-languages/ixy.rs/blob/master/src/ixgbe.rs#L1016)
    fn tx_clean(&mut self, used_buffers: &mut Vec<PacketBufferS>)  {
        const TX_CLEAN_BATCH: usize = 32;

        let mut tx_clean = self.tx_clean as usize;
        let tx_cur = self.tx_cur;

        loop {
            let mut cleanable = tx_cur as i32 - tx_clean as i32;

            if cleanable < 0 {
                cleanable += self.num_tx_descs as i32;
            }
    
            if cleanable < TX_CLEAN_BATCH as i32 {
                break;
            }
    
            let mut cleanup_to = tx_clean + TX_CLEAN_BATCH - 1;

            if cleanup_to >= self.num_tx_descs as usize {
                cleanup_to -= self.num_tx_descs as usize;
            }

            if self.tx_descs[cleanup_to].desc_done() {
                if TX_CLEAN_BATCH >= self.tx_bufs_in_use.len() {
                    used_buffers.extend(self.tx_bufs_in_use.drain(..))
                } else {
                    used_buffers.extend(self.tx_bufs_in_use.drain(..TX_CLEAN_BATCH))
                };

                tx_clean = (cleanup_to + 1) % self.num_tx_descs as usize;
            } else {
                break;
            }
        }

        self.tx_clean = tx_clean as u16;
    }

    /// To Do: disable queue according to data sheet (set registers)
    /// policy descisions: do we empty out all packets waiting to be transmitted?
    pub fn disable(self) -> TxQueue<{TxState::Disabled}> {
        panic!("Not fully implemented");
        TxQueue {
            id: self.id,
            regs: self.regs,
            tx_descs: self.tx_descs,
            num_tx_descs: self.num_tx_descs,
            tx_cur: self.tx_cur,
            tx_bufs_in_use: self.tx_bufs_in_use,
            tx_clean: self.tx_clean,
            cpu_id : self.cpu_id
        }
    }
}


impl Deref for TxQueue<{TxState::Enabled}> {
    type Target = BoxRefMut<MappedPages, [AdvancedTxDescriptor]>;

    fn deref(&self) -> &Self::Target {
        &self.tx_descs
    }
}

impl DerefMut for TxQueue<{TxState::Enabled}> {    
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.tx_descs
    }
}

impl TxQueue<{TxState::Disabled}> {
    /// To Do: enable queue according to data sheet (set registers)
    pub fn enable(self) -> TxQueue<{TxState::Enabled}> {
        panic!("Not fully implemented");
        TxQueue {
            id: self.id,
            regs: self.regs,
            tx_descs: self.tx_descs,
            num_tx_descs: self.num_tx_descs,
            tx_cur: self.tx_cur,
            tx_bufs_in_use: self.tx_bufs_in_use,
            tx_clean: self.tx_clean,
            cpu_id : self.cpu_id
        }
    }
}


// implementation of pseudo functions that should only be used for testing
impl TxQueue<{TxState::Enabled}> {
    /// Sets all the descriptors in the tx queue with a valid packet buffer but doesn't update the TDT.
    /// Requires that the length of `buffers` is equal to the number of descriptors in the queue
    pub fn tx_populate(&mut self, buffers: &mut Vec<PacketBufferS>) {
        // assert!(buffers.len() == self.tx_descs.len());

        for desc in self.tx_descs.iter_mut() {
            let mut packet = buffers.pop().unwrap();
            packet.length = 64;
            desc.send(packet.phys_addr, packet.length);
            self.tx_bufs_in_use.push_back(packet);
        }

        assert!(self.tx_bufs_in_use.len() == self.tx_descs.len());
    }


    /// Send a max `batch_size` number of packets.
    /// There is no buffer management, and this function simply reuses the packet buffer that's already stored.
    /// The number of packets sent is returned.
    pub fn tx_batch_pseudo(&mut self, batch_size: usize) -> usize {
        let mut pkts_sent = 0;
        let mut tx_cur = self.tx_cur;

        self.tx_clean_pseudo();
        let tx_clean = self.tx_clean;

        for _ in 0..batch_size {
            let tx_next = (tx_cur + 1) % self.num_tx_descs;

            if tx_clean == tx_next {
                // tx queue of device is full
                break;
            }

            self.tx_descs[tx_cur as usize].send(self.tx_bufs_in_use[tx_cur as usize].phys_addr, self.tx_bufs_in_use[tx_cur as usize].length);

            tx_cur = tx_next;
            pkts_sent += 1;
        }

        self.tx_cur = tx_cur;
        self.regs.tdt.write(tx_cur as u32);

        pkts_sent
    }

    fn tx_clean_pseudo(&mut self) {
        const TX_CLEAN_BATCH: usize = 32;
        let mut tx_clean = self.tx_clean as usize;
        let tx_cur = self.tx_cur;

        loop {
            let mut cleanable = tx_cur as i32 - tx_clean as i32;

            if cleanable < 0 {
                cleanable += self.num_tx_descs as i32;
            }
    
            if cleanable < TX_CLEAN_BATCH as i32 {
                break;
            }
    
            let mut cleanup_to = tx_clean + TX_CLEAN_BATCH - 1;

            if cleanup_to >= self.num_tx_descs as usize {
                cleanup_to -= self.num_tx_descs as usize;
            }

            if self.tx_descs[cleanup_to].desc_done() {
                tx_clean = (cleanup_to + 1) % self.num_tx_descs as usize;
            } else {
                break;
            }
        }
        self.tx_clean = tx_clean as u16;
    }
}

#[derive(PartialEq, Eq)]
pub enum TxState {
    Disabled,
    Enabled,
}