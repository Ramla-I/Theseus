// To Do: test the state transition functions

use crate::hal::{*, descriptors::AdvancedRxDescriptor};
use crate::queue_registers::RxQueueRegisters;
use crate::mempool::*;
use memory::{BorrowedSliceMappedPages, Mutable, DMA_FLAGS, create_contiguous_mapping};
use core::convert::TryFrom;
use alloc::vec::Vec;
use core::marker::ConstParamTy;
use log::error;

#[derive(PartialEq, Eq)]
#[derive(ConstParamTy)]
pub enum RxState {
    Disabled,
    Enabled,
    L3L4Filter,
    RSS
}

pub type RxQueueE   = RxQueue<{RxState::Enabled}>;
pub type RxQueueD   = RxQueue<{RxState::Disabled}>;
pub type RxQueueF  = RxQueue<{RxState::L3L4Filter}>;
pub type RxQueueRSS = RxQueue<{RxState::RSS}>;

/// A struct that holds all information for one receive queue.
/// There should be one such object per queue.
pub struct RxQueue<const S: RxState> {
    /// The number of the queue, stored here for our convenience.
    pub(crate) id: QueueID,
    /// Registers for this receive queue
    regs: RxQueueRegisters,
    /// Receive descriptors
    desc_ring: BorrowedSliceMappedPages<AdvancedRxDescriptor, Mutable>,
    /// The number of receive descriptors in the descriptor ring
    num_descs: u16,
    /// Current receive descriptor index
    curr_desc: u16,
    /// The list of rx buffers, in which the index in the vector corresponds to the index in `rx_descs`.
    /// For example, `rx_bufs_in_use[2]` is the receive buffer that will be used when `rx_descs[2]` is the current rx descriptor (curr_desc = 2).
    buffs_in_use: Vec<PktBuff>,
    /// Extra packet buffers
    pub(crate) mempool: Mempool,
    /// The filter id for the physical NIC filter that is set for this queue
    filter_id: Option<L5FilterID>
}


impl RxQueue<{RxState::Enabled}> {
    pub(crate) fn new(mut regs: RxQueueRegisters, num_descs: NumDesc) -> Result<RxQueue<{RxState::Enabled}>, &'static str> {
        let mut mempool = Mempool::new(num_descs as usize * 2)?;

        // create the descriptor ring
        let (descs_mapped_pages, descs_paddr) = create_contiguous_mapping(num_descs as usize * core::mem::size_of::<AdvancedRxDescriptor>(), DMA_FLAGS)?;
        let mut desc_ring = descs_mapped_pages.into_borrowed_slice_mut::<AdvancedRxDescriptor>(0, num_descs as usize).map_err(|(_mp, err)| err)?;

        // now that we've created the rx descriptors, we can fill them in with initial values
        let mut buffs_in_use: Vec<PktBuff> = Vec::with_capacity(num_descs as usize);
        for rd in desc_ring.iter_mut()
        {
            // obtain a receive buffer for each rx_desc
            // letting this fail instead of allocating here alerts us to a logic error, we should always have more buffers in the pool than the fdescriptor ringh
            let rx_buf = mempool.buffers.pop_front()
                .ok_or("Couldn't obtain a PktBuff from the pool")?; 
            
            rd.set_packet_address(rx_buf.paddr); 
            buffs_in_use.push(rx_buf); 
        }
        
        // write the physical address of the rx descs ring
        regs.rdbal.write(descs_paddr.value() as u32);
        regs.rdbah.write((descs_paddr.value() >> 32) as u32);

        // write the length (in total bytes) of the rx descs array
        regs.rdlen_write(num_descs); // should be 128 byte aligned, minimum 8 descriptors

        // Write the head index (the first receive descriptor)
        // regs.rdh_write(0);
        // regs.rdt_write(0);   

        // set the size of the packet buffers(leave default value) and the descriptor format used
        regs.srrctl_write(DescType::AdvDesc1Buf, RxBufferSizeKiB::Buffer2KiB);
        regs.srrctl_drop_enable();

        // enable the rx queue and make sure it's enabled
        regs.rxdctl_rxq_enable();
        const RX_Q_ENABLE: u32 = 1 << 25;
        while regs.rxdctl_read() & RX_Q_ENABLE == 0 {}

        // Write the tail index.
        // Note that the 82599 datasheet (section 8.2.3.8.5) states that we should set the RDT (tail index) to the index *beyond* the last receive descriptor, 
        // but we set it to the last receive descriptor for the same reason as the e1000 driver
        regs.rdt_write(num_descs as u16 - 1);

        // set bit 12 to 0
        regs.dca_rxctrl_clear_bit_12();

        Ok(RxQueue { 
            id: QueueID::try_from(regs.id() as u8).map_err(|_| "tried to create queue with id >= 64")?, 
            regs, 
            desc_ring, 
            num_descs: num_descs as u16,
            curr_desc: 0, 
            buffs_in_use, 
            mempool,
            filter_id: None
        })
    }

    // To Do: wait until all descriptors are written back
    pub fn disable(mut self) -> RxQueueD {
        self.regs.rxdctl_rxq_disable();
        self.buffs_in_use.clear();
        RxQueueD{.. self}
    }

    /// Retrieves a maximum of `batch_size` number of packets and stores them in `buffers`.
    /// Returns the total number of received packets.
    #[inline(always)]
    #[cfg(verified)]
    pub fn receive_batch(&mut self, buffers: &mut Vec<PktBuff>, batch_size: usize) -> u16 {
        let (rcvd_pkts, rdt) = verified::receive(
            &mut self.curr_desc, 
            &mut self.desc_ring, 
            &mut self.buffs_in_use, 
            buffers, 
            &mut self.mempool.buffers, 
            batch_size as u16
        );

        if rcvd_pkts > 0 {
            self.regs.rdt_write(rdt.0); 
        }
        rcvd_pkts
    }

    /// Retrieves a maximum of `batch_size` number of packets and stores them in `buffers`.
    /// Returns the total number of received packets.
    #[inline(always)]
    #[cfg(not(verified))]
    pub fn receive_batch(&mut self, buffers: &mut Vec<PktBuff>, batch_size: usize) -> u16 {
        let mut curr_desc = self.curr_desc;
        let mut prev_curr_desc = self.curr_desc;

        let mut rcvd_pkts = 0;

        for _ in 0..batch_size {
            let desc = &mut self.desc_ring[curr_desc as usize];
            let (dd, length) = desc.rx_metadata();

            if !dd { break; }

            // if !desc.end_of_packet() {
            //     error!("intel_ethernet::rx_batch(): multi-descriptor packets are not supported yet!");
            //     panic!();
            // }

            // Now that we are "removing" the current receive buffer from the list of receive buffers that the NIC can use,
            // (because we're saving it for higher layers to use),
            // we need to obtain a new `PktBuff` and set it up such that the NIC will use it for future receivals.
            if let Some(new_receive_buf) = self.mempool.buffers.pop_front() {
                // actually tell the NIC about the new receive buffer, and that it's ready for use now
                desc.set_packet_address(new_receive_buf.paddr);
                
                let mut current_rx_buf = core::mem::replace(&mut self.buffs_in_use[curr_desc as usize], new_receive_buf);

                // unsafe{ core::arch::x86_64::_mm_prefetch(current_rx_buf.buffer.as_ptr() as *const i8, _MM_HINT_ET0);}
                current_rx_buf.length = length as u16; // set the ReceiveBuffer's length to the size of the actual packet received
                buffers.push(current_rx_buf);

                rcvd_pkts += 1;
                prev_curr_desc = curr_desc;
                curr_desc = (curr_desc + 1) & (self.num_descs - 1);
            } else {
                error!("Ran out of packet buffers in the pool!");
                panic!("Ran out of packet buffers in the pool!");
            }
        }

        if prev_curr_desc != curr_desc {
            self.curr_desc = curr_desc as u16;
            self.regs.rdt_write(prev_curr_desc); 
        }

        rcvd_pkts
    }

    pub fn mempool(&mut self) -> &mut Mempool {
        &mut self.mempool
    }
}


impl RxQueue<{RxState::Disabled}> {
    pub fn enable(mut self) -> Result<RxQueueE, &'static str> {
        self.curr_desc = 0;

        for rd in self.desc_ring.iter_mut()
        {
            // obtain a receive buffer for each rx_desc
            // letting this fail instead of allocating here alerts us to a logic error, we should always have more buffers in the pool than the fdescriptor ringh
            let rx_buf = self.mempool.buffers.pop_front()
                .ok_or("Couldn't obtain a PktBuff from the pool")?; 
            
            rd.set_packet_address(rx_buf.paddr); 
            self.buffs_in_use.push(rx_buf); 
        }
        // descriptor ring is still stored so need to update it.'
        // probably would be more memory efficient to drop desc ring when disabled, 
        // have an entirely separate struct for the disabled state
        
        // Write the head index (the first receive descriptor)
        self.regs.rdh_write(0);        
        // enable the rx queue and make sure it's enabled
        self.regs.rxdctl_rxq_enable();
        const RX_Q_ENABLE: u32 = 1 << 25;
        while self.regs.rxdctl_read() & RX_Q_ENABLE == 0 {}

        // Write the tail index.
        // Note that the 82599 datasheet (section 8.2.3.8.5) states that we should set the RDT (tail index) to the index *beyond* the last receive descriptor, 
        // but we set it to the last receive descriptor for the same reason as the e1000 driver
        self.regs.rdt_write(self.num_descs - 1);

        Ok(RxQueueE{.. self})
    }
}