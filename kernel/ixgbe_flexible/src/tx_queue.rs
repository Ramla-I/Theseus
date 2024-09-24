use memory::{create_contiguous_mapping, BorrowedMappedPages, BorrowedSliceMappedPages, Mutable, DMA_FLAGS};
use crate::hal::{*, regs::{ReportStatusBit, TDHSet}, descriptors::AdvancedTxDescriptor, transmit_head_wb::TransmitHead};
use crate::queue_registers::TxQueueRegisters;
use crate::mempool::PktBuff;
use alloc::vec::Vec;
use core::marker::ConstParamTy;

#[derive(PartialEq, Eq)]
#[derive(ConstParamTy)]
pub enum TxState {
    Disabled,
    Enabled,
}

pub type TxQueueE = TxQueue<{TxState::Enabled}>;
pub type TxQueueD = TxQueue<{TxState::Disabled}>;


/// A struct that holds all information for a transmit queue. 
/// There should be one such object per queue.
pub struct TxQueue<const S: TxState> {
    /// The number of the queue, stored here for our convenience.
    pub id: QueueID,
    /// Registers for this transmit queue
    pub(crate) regs: TxQueueRegisters,
    /// Transmit descriptors 
    desc_ring: BorrowedSliceMappedPages<AdvancedTxDescriptor, Mutable>,
    /// The number of transmit descriptors in the descriptor ring
    num_descs: u16,
    /// Current transmit descriptor index (first desc that can be used)
    curr_desc: u16,
    /// The packet buffers that descriptors have stored information of
    buffs_in_use: Vec<PktBuff>,
    /// first descriptor that has been used but not checked for transmit completion
    /// or in some cases, it hasn't been used like when we start out and right after clean
    tx_clean: u16,
    /// value of Report Status flag in the descriptors
    rs_bit: ReportStatusBit,
    /// descriptor write back address
    head_wb: BorrowedMappedPages<TransmitHead, Mutable>,
}

impl TxQueue<{TxState::Enabled}> {
    pub(crate) fn new(mut regs: TxQueueRegisters, num_descs: NumDesc) -> Result<(TxQueue<{TxState::Enabled}>, TDHSet), &'static str> {
        // create the descriptor ring
        let (descs_mapped_pages, descs_paddr) = create_contiguous_mapping(num_descs as usize * core::mem::size_of::<AdvancedTxDescriptor>(), DMA_FLAGS)?;
        let desc_ring = descs_mapped_pages.into_borrowed_slice_mut::<AdvancedTxDescriptor>(0, num_descs as usize).map_err(|(_mp, err)| err)?;

        let (head_wb_mp, head_wb_paddr) = create_contiguous_mapping(core::mem::size_of::<TransmitHead>(), DMA_FLAGS)?;

        // write the physical address of the tx descs array
        regs.tdbal.write(descs_paddr.value() as u32); 
        regs.tdbah.write((descs_paddr.value() >> 32) as u32); 

        // write the length (in total bytes) of the tx descs array
        let tdlen_set = regs.tdlen_write(num_descs);               
       
        // Set tx descriptor pre-fetch threshold and host threshold 
        let pthresh = U7::B5 | U7::B4 | U7::B3 | U7::B2;// U7::B5 | U7::B2; // b100100 = 36 (DPDK), TInyNF uses 60 b111100
        let hthresh = HThresh::B2; //HThresh::B3; // b1000 = 8 (DPDK), TinyNF uses 4  b100
        let wthresh = U7::zero(); // b100 = 4 
        let rs_bit = regs.txdctl_write_wthresh(wthresh); 
        regs.txdctl_write_pthresh_hthresh(pthresh, hthresh); 
        
        // here we should set the head writeback address
        regs.tdwba_set_and_enable(head_wb_paddr.value() as u64);
        regs.dca_txctrl_disable_relaxed_ordering_head_wb();

        // write the head index and the tail index (both 0 initially because there are no tx requests yet)
        let tdh_set = regs.tdh_write(0, tdlen_set);
        regs.tdt_write(0);
        
        // // enable tx queue and make sure it's enabled
        // regs.txdctl_txq_enable(tdh_set); 
        // const TX_Q_ENABLE: u32 = 1 << 25;
        // while regs.txdctl_read() & TX_Q_ENABLE == 0 {} 

        Ok((TxQueue { 
            id: QueueID::try_from(regs.id() as u8).map_err(|_| "tried to create queue with id >= 64")?, 
            regs, 
            desc_ring, 
            num_descs: num_descs as u16, 
            curr_desc: 0, 
            buffs_in_use: Vec::with_capacity(num_descs as usize), 
            tx_clean: 0,
            rs_bit: rs_bit, 
            head_wb: head_wb_mp.into_borrowed_mut(0).map_err(|(_mp, err)| err)? 
        }, tdh_set))
    }

    // /// Sends a maximum of `batch_size` number of packets from the stored `buffers`.
    // /// The number of packets sent are returned.
    // /// 
    // /// I don't think this code is very stable, if the TX_CLEAN_THRESHOLD is less than the number of descriptors, and divisor, then we should be good.
    // #[inline(always)]
    // pub fn tx_batch(&mut self, batch_size: usize,  buffers: &mut Vec<PacketBuffer>, pool: &mut Mempool) -> u16 {
    //     const TX_CLEAN_THRESHOLD: u16 = 64; // make sure this is less than and an even divisor fo the queue size
    //     // error!("before cleaning: tx_cur = {}, tx_clean ={}", self.tx_cur, self.tx_clean);

    //     // let mut descs_to_clean = self.tx_cur as i32 - self.tx_clean as i32;
    //     // if descs_to_clean < 0 { descs_to_clean += self.num_tx_descs as i32; }

    //     // if descs_to_clean as u16 >= 2 * TX_CLEAN_THRESHOLD {
    //     //     // error!("Cleaning {} descs", descs_to_clean);
    //     //     let mut old_clean_batch = 0;
    //     //     let mut clean_batch = TX_CLEAN_THRESHOLD;

    //     //     while clean_batch <= descs_to_clean as u16 {
    //     //         let desc_to_check = (self.tx_clean + clean_batch - 1) & (self.num_tx_descs - 1);
    //     //         if self.tx_descs[desc_to_check as usize].desc_done() {
    //     //             old_clean_batch = clean_batch;
    //     //             clean_batch += TX_CLEAN_THRESHOLD;
    //     //             // error!("desc {} is done, old = {} new clean batch = {}", desc_to_check, old_clean_batch, clean_batch);
    //     //         } 
    //     //     }
    
    //     //     pool.extend(self.tx_bufs_in_use.drain(..old_clean_batch as usize)); // if descs_to_clean = 0, won't do anything
    //     //     self.tx_clean = (self.tx_clean + old_clean_batch as u16) % self.num_tx_descs;
    //     // }

    //     self.tx_clean(pool);
    //     let mut pkts_sent = 0;

    //     for _ in 0..batch_size {
    //         let tx_next = (self.tx_cur + 1) % self.num_tx_descs;
    //         if (tx_next == self.tx_clean) { //&& (tx_next != 0) {
    //             break;
    //         }

    //         if let Some(packet) = buffers.pop() {
    //             // if packet.length == 0 {
    //             //     error!("zero sized packet!");
    //             //     panic!();
    //             // }
    //             let rs_bit = if (self.tx_cur % TX_CLEAN_THRESHOLD) == TX_CLEAN_THRESHOLD - 1 { self.rs_bit.value() } else { 0 };
    //             // error!("rs_bit = {}", rs_bit);
    //             let (paddr, length) = (packet.paddr, packet.length);
    //             self.tx_descs[self.tx_cur as usize].send(paddr, length, rs_bit);
    //             self.tx_bufs_in_use.push(packet);
    
    //             self.tx_cur = tx_next;
    //             pkts_sent += 1;
    //         } else {
    //             break;
    //         }
    //     }


    //     // error!("sent {} packets", pkts_sent);
    //     // error!("end cleaning: tx_cur = {}, tx_clean ={}", self.tx_cur, self.tx_clean);
    //     // for i in 0..self.num_tx_descs {
    //     //     error!("desc{}: {:?}", i, self.tx_descs[i as usize].desc_done());
    //     // }
    //     if pkts_sent > 0 { self.regs.tdt_write(self.tx_cur); }
    //     pkts_sent
    // }

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

    // /// Removes multiples of `TX_CLEAN_BATCH` packets from `queue`.    
    // /// (code taken from https://github.com/ixy-languages/ixy.rs/blob/master/src/ixgbe.rs#L1016)
    // #[inline(always)]
    // fn tx_clean(&mut self, used_buffers: &mut Mempool)  {
    //     const TX_CLEAN_BATCH: u16 = 64;
    //     let head = self.head_wb.value.read() as u16;
    //     // error!("head = {}", head);

    //     let cleanable = (head as i32 - self.tx_clean as i32) as u16 & (self.num_tx_descs - 1);
    //     // error!("Cleanable = {}", cleanable);
    //     // if cleanable < 0 {
    //     //     cleanable += self.num_tx_descs as i32;
    //     // }
    //     if cleanable < TX_CLEAN_BATCH {
    //         return;
    //     }

    //     // let cleanup_to = (tx_clean + cleanable - 1) & (self.num_tx_descs - 1);

    //     // if cleanup_to >= self.num_tx_descs as usize {
    //     //     cleanup_to -= self.num_tx_descs as usize;
    //     // }

    //     if cleanable as usize >= self.tx_bufs_in_use.len() {
    //         used_buffers.extend(self.tx_bufs_in_use.drain(..))
    //     } else {
    //         used_buffers.extend(self.tx_bufs_in_use.drain(..cleanable as usize))
    //     };

    //     // tx_clean = (cleanup_to + 1) % self.num_tx_descs;

    //     // self.tx_clean = tx_clean;
    //     self.tx_clean = head;
    // }
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

    // fn tx_clean_pseudo(&mut self) {
    //     const TX_CLEAN_BATCH: usize = 32;
    //     let mut tx_clean = self.tx_clean as usize;
    //     let tx_cur = self.tx_cur;

    //     loop {
    //         let mut cleanable = tx_cur as i32 - tx_clean as i32;

    //         if cleanable < 0 {
    //             cleanable += self.num_tx_descs as i32;
    //         }
    
    //         if cleanable < TX_CLEAN_BATCH as i32 {
    //             break;
    //         }
    
    //         let mut cleanup_to = tx_clean + TX_CLEAN_BATCH - 1;

    //         if cleanup_to >= self.num_tx_descs as usize {
    //             cleanup_to -= self.num_tx_descs as usize;
    //         }

    //         if self.tx_descs[cleanup_to].desc_done() {
    //             tx_clean = (cleanup_to + 1) % self.num_tx_descs as usize;
    //         } else {
    //             break;
    //         }
    //     }
    //     self.tx_clean = tx_clean as u16;
    // }
}

