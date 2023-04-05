use memory::{MappedPages, create_contiguous_mapping, BorrowedSliceMappedPages, Mutable};
use zerocopy::FromBytes;
use crate::regs::ReportStatusBit;
use crate::{hal::descriptors::LegacyTxDescriptor};
use crate::queue_registers::TxQueueRegisters;
use crate::{NumDesc};
use crate::allocator::*;
use owning_ref::{BoxRefMut, BoxRef};
use core::{panic, num};
use core::{ops::{DerefMut, Deref}};
use alloc::{
    vec::Vec,
};
use hal::regs::TDHSet;
use volatile::Volatile;
use mempool::*;

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
    pub(crate) tx_descs: BorrowedSliceMappedPages<LegacyTxDescriptor, Mutable>,
    /// The number of transmit descriptors in the descriptor ring
    num_tx_descs: u16,
    /// Current transmit descriptor index (first desc that can be used)
    tx_cur: u16,
    /// The packet buffers that descriptors have stored information of
    tx_bufs_in_use: Vec<PacketBuffer>,
    /// first descriptor that has been used but not checked for transmit completion
    /// or in some cases, it hasn't been used like when we start out and right after clean
    tx_clean: u16,
    /// The cpu which this queue is mapped to. 
    /// This in itself doesn't guarantee anything but we use this value when setting the cpu id for interrupts and DCA.
    cpu_id : Option<u8>,
    /// value of Report Status flag in the descriptors
    rs_bit: u8,
    /// descriptor write back address
    head_wb: BoxRefMut<MappedPages, TransmitHead>,
}

#[derive(FromBytes)]
#[repr(C, align(64))]
pub struct TransmitHead {
    pub value: Volatile<u32>,
    pub value2: Volatile<u32>,
    pub value3: Volatile<u32>,
}

impl TxQueue<{TxState::Enabled}> {
    pub(crate) fn new(mut regs: TxQueueRegisters, num_desc: NumDesc, cpu_id: Option<u8>, rs_bit: ReportStatusBit) -> Result<(TxQueue<{TxState::Enabled}>, TDHSet), &'static str> {
        let (tx_descs, paddr) = create_desc_ring::<LegacyTxDescriptor>(num_desc)?;
        let num_tx_descs = tx_descs.len();

        // write the physical address of the tx descs array
        regs.tdbal.write(paddr.value() as u32); 
        regs.tdbah.write((paddr.value() >> 32) as u32); 

        // write the length (in total bytes) of the tx descs array
        let tdlen_set = regs.tdlen_write(num_desc);               
        
        // write the head index and the tail index (both 0 initially because there are no tx requests yet)
        let tdh_set = regs.tdh_write(0, tdlen_set);
        regs.tdt_write(0);

        let (head_wb, desc_wb_paddr) = create_descriptor_writeback_field()?;

        // regs.tdwba_set_and_enable(desc_wb_paddr.value() as u64);

        // Not sure why but listed in TinyNF as
        //"INTERPRETATION-MISSING: We must disable relaxed ordering of head pointer write-back, since it could cause the head pointer to be updated backwards"
        // regs.dca_txctrl_disable_relaxed_ordering_head_wb();

        Ok((TxQueue { id: regs.id() as u8, regs, tx_descs, num_tx_descs: num_tx_descs as u16, tx_cur: 0, tx_bufs_in_use: Vec::with_capacity(num_tx_descs), tx_clean: 0, cpu_id, rs_bit: rs_bit.value(), head_wb }, tdh_set))
    }

    /// Sends a maximum of `batch_size` number of packets from the stored `buffers`.
    /// The number of packets sent are returned.
    /// 
    /// I don't think this code is very stable, if the TX_CLEAN_THRESHOLD is less than the number of descriptors, and divisor, then we should be good.
    #[inline(always)]
    pub fn tx_batch(&mut self, batch_size: usize,  buffers: &mut Vec<PacketBuffer>, pool: &mut Mempool) -> u16 {
        const TX_CLEAN_THRESHOLD: u16 = 32; // make sure this is less than and an even divisor fo the queue size
        // error!("before cleaning: tx_cur = {}, tx_clean ={}", self.tx_cur, self.tx_clean);

        // let mut descs_to_clean = self.tx_cur as i32 - self.tx_clean as i32;
        // if descs_to_clean < 0 { descs_to_clean += self.num_tx_descs as i32; }

        // if descs_to_clean as u16 >= 2 * TX_CLEAN_THRESHOLD {
        //     // error!("Cleaning {} descs", descs_to_clean);
        //     let mut old_clean_batch = 0;
        //     let mut clean_batch = TX_CLEAN_THRESHOLD;

        //     while clean_batch <= descs_to_clean as u16 {
        //         let desc_to_check = (self.tx_clean + clean_batch - 1) & (self.num_tx_descs - 1);
        //         if self.tx_descs[desc_to_check as usize].desc_done() {
        //             old_clean_batch = clean_batch;
        //             clean_batch += TX_CLEAN_THRESHOLD;
        //             // error!("desc {} is done, old = {} new clean batch = {}", desc_to_check, old_clean_batch, clean_batch);
        //         } 
        //     }
    
        //     pool.extend(self.tx_bufs_in_use.drain(..old_clean_batch as usize)); // if descs_to_clean = 0, won't do anything
        //     self.tx_clean = (self.tx_clean + old_clean_batch as u16) % self.num_tx_descs;
        // }

        self.tx_clean(pool);

        let mut pkts_sent = 0;

        for _ in 0..batch_size {
            let tx_next = (self.tx_cur + 1) % self.num_tx_descs;
            if (tx_next == self.tx_clean) { //&& (tx_next != 0) {
                break;
            }

            if let Some(packet) = buffers.pop() {
                // if packet.length == 0 {
                //     error!("zero sized packet!");
                //     panic!();
                // }
                let rs_bit = if (self.tx_cur % TX_CLEAN_THRESHOLD) == TX_CLEAN_THRESHOLD - 1 { self.rs_bit } else { 0 };
                // error!("rs_bit = {}", rs_bit);
                self.tx_descs[self.tx_cur as usize].send(pool.paddr(&packet), packet.get_length(), rs_bit);
                self.tx_bufs_in_use.push(packet);
    
                self.tx_cur = tx_next;
                pkts_sent += 1;
            } else {
                break;
            }
        }

        // error!("sent {} packets", pkts_sent);
        // error!("end cleaning: tx_cur = {}, tx_clean ={}", self.tx_cur, self.tx_clean);
        // for i in 0..self.num_tx_descs {
        //     error!("desc{}: {:?}", i, self.tx_descs[i as usize].desc_done());
        // }
        if pkts_sent > 0 { self.regs.tdt_write(self.tx_cur); }
        pkts_sent
    }

    // /// Sends a maximum of `batch_size` number of packets from the stored `buffers`.
    // /// The number of packets sent are returned.
    // #[inline(always)]
    // pub fn tx_batch_wo_wb(&mut self, batch_size: usize,  buffers: &mut Vec<PacketBufferS>, pool: &mut Vec<PacketBufferS>) -> u16 {
    //     // verified_functions::tx_batch(
    //     //     &mut self.tx_descs, 
    //     //     &mut self.tx_bufs_in_use,
    //     //     self.num_tx_descs,
    //     //     &mut self.tx_clean,
    //     //     &mut self.tx_cur,
    //     //     &mut self.regs,
    //     //     batch_size,  
    //     //     buffers, 
    //     //     used_buffers,
    //     //     self.RS_bit
    //     // ).0
    //     let mut pkts_sent = 0;
    //     let mut tx_cur = self.tx_cur;

    //     self.tx_clean(pool);
    //     let tx_clean = self.tx_clean;
    //     // debug!("tx_cur = {}, tx_clean ={}", tx_cur, tx_clean);

    //     for _ in 0..batch_size {
    //         if let Some(packet) = buffers.pop() {
    //             let tx_next = (tx_cur + 1) % self.num_tx_descs;
    
    //             if tx_clean == tx_next {
    //                 // tx queue of device is full, push packet back onto the
    //                 // queue of to-be-sent packets
    //                 // error!("could not send packet, tx queue is full {}", buffers.len() + 1);
    //                 // error!("tx clean {}, tx cur {}", tx_clean, tx_cur);
    //                 buffers.push(packet);
    //                 break;
    //             }
    //             if packet.length == 0 {
    //                 error!("zero sized packet!");
    //                 panic!();
    //             }
    //             self.tx_descs[tx_cur as usize].send(packet.phys_addr(), packet.length, self.rs_bit);
    //             self.tx_bufs_in_use.push(packet);
    
    //             tx_cur = tx_next;
    //             pkts_sent += 1;
    //         } else {
    //             break;
    //         }
    //     }

    //     // error!("sent {} packets", pkts_sent);
    //     // error!("tx_clean = {}, tx_cur = {}", tx_clean, tx_cur);

    //     self.tx_cur = tx_cur;
    //     self.regs.tdt_write(tx_cur);

    //     pkts_sent
    // }

    /// Removes multiples of `TX_CLEAN_BATCH` packets from `queue`.    
    /// (code taken from https://github.com/ixy-languages/ixy.rs/blob/master/src/ixgbe.rs#L1016)
    fn tx_clean(&mut self, used_buffers: &mut Mempool)  {
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

}


impl Deref for TxQueue<{TxState::Enabled}> {
    type Target = BorrowedSliceMappedPages<LegacyTxDescriptor, Mutable>;

    fn deref(&self) -> &Self::Target {
        &self.tx_descs
    }
}

impl DerefMut for TxQueue<{TxState::Enabled}> {    
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.tx_descs
    }
}

// implementation of pseudo functions that should only be used for testing
impl TxQueue<{TxState::Enabled}> {
    // /// Sets all the descriptors in the tx queue with a valid packet buffer but doesn't update the TDT.
    // /// Requires that the length of `buffers` is equal to the number of descriptors in the queue
    // pub fn tx_populate(&mut self, buffers: &mut Vec<PacketBufferS>) {
    //     // assert!(buffers.len() == self.tx_descs.len());

    //     for desc in self.tx_descs.iter_mut() {
    //         let mut packet = buffers.pop().unwrap();
    //         packet.length = 64;
    //         desc.send(packet.phys_addr(), packet.length, self.rs_bit);
    //         self.tx_bufs_in_use.push(packet);
    //     }

    //     assert!(self.tx_bufs_in_use.len() == self.tx_descs.len());
    // }


    // /// Send a max `batch_size` number of packets.
    // /// There is no buffer management, and this function simply reuses the packet buffer that's already stored.
    // /// The number of packets sent is returned.
    // pub fn tx_batch_pseudo(&mut self, batch_size: usize) -> usize {
    //     let mut pkts_sent = 0;
    //     let mut tx_cur = self.tx_cur;

    //     self.tx_clean_pseudo();
    //     let tx_clean = self.tx_clean;

    //     for _ in 0..batch_size {
    //         let tx_next = (tx_cur + 1) % self.num_tx_descs;

    //         if tx_clean == tx_next {
    //             // tx queue of device is full
    //             break;
    //         }

    //         self.tx_descs[tx_cur as usize].send(self.tx_bufs_in_use[tx_cur as usize].phys_addr(), self.tx_bufs_in_use[tx_cur as usize].length, self.rs_bit);

    //         tx_cur = tx_next;
    //         pkts_sent += 1;
    //     }

    //     self.tx_cur = tx_cur;
    //     self.regs.tdt_write(tx_cur);

    //     pkts_sent
    // }

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