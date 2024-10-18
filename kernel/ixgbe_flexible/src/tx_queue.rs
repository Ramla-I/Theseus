// To Do: test the state transition functions

extern crate assert_fields_type;

use memory::{create_contiguous_mapping, BorrowedMappedPages, BorrowedSliceMappedPages, Mutable, DMA_FLAGS};
use crate::hal::{*, regs::{TDHSet, ReportStatusBit}, descriptors::AdvancedTxDescriptor, transmit_head_wb::TransmitHead};
use crate::queue_registers::TxQueueRegisters;
use assert_fields_type::assert_fields_type;
use crate::mempool::{Mempool, PktBuff};
use crate::verified;
use alloc::vec::Vec;
use core::marker::ConstParamTy;
use proc_static_assertions::{nomutates, consumes};
use prusti_external_spec::vec_wrapper::VecWrapper;

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
    num_descs: NumDesc,
    /// Current transmit descriptor index (first desc that can be used)
    curr_desc: u16,
    /// The packet buffers that descriptors have stored information of
    buffs_in_use: VecWrapper<PktBuff>,
    /// first descriptor that has been used but not checked for transmit completion
    /// or in some cases, it hasn't been used like when we start out and right after clean
    tx_clean: u16,
    /// value of Report Status flag in the descriptors
    rs_bit: ReportStatusBit,
    /// descriptor write back address
    head_wb: BorrowedMappedPages<TransmitHead, Mutable>,
}


assert_fields_type!(TxQueueE: regs:TxQueueRegisters, buffs_in_use: VecWrapper<PktBuff>, desc_ring: BorrowedSliceMappedPages<AdvancedTxDescriptor, Mutable>);

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
        
        Ok((TxQueue { 
            id: QueueID::try_from(regs.id() as u8).map_err(|_| "tried to create queue with id >= 64")?, 
            regs, 
            desc_ring, 
            num_descs: num_descs, 
            curr_desc: 0, 
            buffs_in_use: VecWrapper::with_capacity(num_descs as usize), 
            tx_clean: 0,
            rs_bit: rs_bit, 
            head_wb: head_wb_mp.into_borrowed_mut(0).map_err(|(_mp, err)| err)? 
        }, tdh_set))
    }

    // To Do: wait until all descriptors are written back (check head writeback)
    #[consumes("mut self")]
    #[nomutates(TxQueue: ("curr_desc"))]
    pub fn disable(mut self) -> TxQueueD {
        self.regs.txdctl_txq_disable();
        self.buffs_in_use.0.clear();
        TxQueueD{.. self}
    }

    #[inline(always)]
    #[cfg(verified)]
    pub fn send_batch(&mut self, batch_size: usize,  buffers: &mut VecWrapper<PktBuff>, pool: &mut Mempool) -> u16 {
        const TX_CLEAN_THRESHOLD: u16 = 64; // make sure this is less than and an even divisor to the queue size

        self.tx_clean(pool);
        let mut pkts_sent = 0;

        let(sent_pkts, tdt) = verified::transmit(
            &mut self.curr_desc, 
            self.tx_clean, 
            &mut self.desc_ring, 
            &mut self.buffs_in_use, 
            buffers, 
            batch_size as u16, 
            self.rs_bit
        );

        if sent_pkts > 0 { 
            self.regs.tdt_write(tdt.value()); 
        }
        sent_pkts
    }

    /// Sends a maximum of `batch_size` number of packets from the stored `buffers`.
    /// The number of packets sent are returned.
    /// 
    /// I don't think this code is very stable, if the TX_CLEAN_THRESHOLD is less than the number of descriptors, and divisor, then we should be good.
    #[inline(always)]
    #[cfg(not(verified))]
    pub fn send_batch(&mut self, batch_size: usize,  buffers: &mut Vec<PktBuff>, pool: &mut Mempool) -> u16 {
        const TX_CLEAN_THRESHOLD: u16 = 64; // make sure this is less than and an even divisor to the queue size

        self.tx_clean(pool);
        let mut pkts_sent = 0;

        for _ in 0..batch_size {
            let next_desc = (self.curr_desc + 1) % self.num_descs as u16;
            if next_desc == self.tx_clean { //&& (next_desc != 0) {
                break;
            }

            if let Some(packet) = buffers.pop() {
                let rs_bit = if (self.curr_desc % TX_CLEAN_THRESHOLD) == TX_CLEAN_THRESHOLD - 1 { self.rs_bit } else { ReportStatusBit::zero() };
                self.desc_ring[self.curr_desc as usize].send(packet.paddr, packet.length, rs_bit);
                self.buffs_in_use.push(packet);
    
                self.curr_desc = next_desc;
                pkts_sent += 1;
            } else {
                break;
            }
        }

        if pkts_sent > 0 { 
            // To Do: Try adding all buffers together at this point. should be fewer vec operations
            self.regs.tdt_write(self.curr_desc); 
        }
        pkts_sent
    }

    /// Removes sent packets from the descriptor ring.    
    #[inline(always)]
    #[nomutates(TxQueue: ("curr_desc"))]
    fn tx_clean(&mut self, pool: &mut Mempool)  {
        const TX_CLEAN_BATCH: u16 = 64;
        let head = self.head_wb.value() as u16;

        let cleanable = (head as i32 - self.tx_clean as i32) as u16 & (self.num_descs as u16 - 1);
        if cleanable < TX_CLEAN_BATCH {
            return;
        }

        if cleanable as usize >= self.buffs_in_use.len() {
            pool.buffers.0.extend(self.buffs_in_use.0.drain(..))
        } else {
            pool.buffers.0.extend(self.buffs_in_use.0.drain(..cleanable as usize))
        };

        self.tx_clean = head;
    }
}


impl TxQueue<{TxState::Disabled}> {
    #[consumes("mut self")]
    pub fn enable(mut self) -> TxQueueE {
        self.tx_clean = 0;
        self.curr_desc = 0;

        let tdlen_set = self.regs.tdlen_write(self.num_descs);
        let tdh_set = self.regs.tdh_write(0, tdlen_set);
        self.regs.tdt_write(0);
        // descriptor ring is still stored so need to update it.'
        // probably would be more memory efficient to drop desc ring when disabled, 
        // have an entirely separate struct for the disabled state
        
        self.regs.txdctl_txq_enable(tdh_set);
        TxQueueE{.. self}
    }
}