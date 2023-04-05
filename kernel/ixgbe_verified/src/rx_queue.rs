use memory::{MappedPages, BorrowedSliceMappedPages, Mutable};
use crate::{hal::{*, descriptors::LegacyRxDescriptor}, RxBufferSizeKiB, DEFAULT_RX_BUFFER_SIZE_2KB, L5FilterID, regs::*, FilterParameters, FilterError};
// use crate::vec_wrapper::VecWrapper;
use crate::queue_registers::RxQueueRegisters;
use crate::NumDesc;
use crate::allocator::*;
use owning_ref::BoxRefMut;
use core::{ops::{DerefMut, Deref}, convert::TryFrom, arch::x86_64::_MM_HINT_ET0};
use alloc::vec::Vec;
use crate::mempool::*;

pub type RxQueueE   = RxQueue<{RxState::Enabled}>;
pub type RxQueueD   = RxQueue<{RxState::Disabled}>;
pub type RxQueueL5  = RxQueue<{RxState::L5Filter}>;
pub type RxQueueRSS = RxQueue<{RxState::RSS}>;

/// A struct that holds all information for one receive queue.
/// There should be one such object per queue.
pub struct RxQueue<const S: RxState> {
    /// The number of the queue, stored here for our convenience.
    pub id: QueueID,
    /// Registers for this receive queue
    pub regs: RxQueueRegisters,
    /// Receive descriptors
    pub rx_descs: BorrowedSliceMappedPages<LegacyRxDescriptor, Mutable>,
    /// The number of receive descriptors in the descriptor ring
    pub num_rx_descs: u16,
    /// Current receive descriptor index
    pub rx_cur: u16,
    /// The list of rx buffers, in which the index in the vector corresponds to the index in `rx_descs`.
    /// For example, `rx_bufs_in_use[2]` is the receive buffer that will be used when `rx_descs[2]` is the current rx descriptor (rx_cur = 2).
    pub rx_bufs_in_use: Vec<PacketBuffer>,
    pub rx_buffer_size: RxBufferSizeKiB,
    /// Pool where `ReceiveBuffer`s are stored.
    pub rx_buffer_pool: Mempool,
    /// The cpu which this queue is mapped to. 
    /// This in itself doesn't guarantee anything, but we use this value when setting the cpu id for interrupts and DCA.
    pub cpu_id: Option<u8>,
    /// The filter id for the physical NIC filter that is set for this queue
    pub filter_num: Option<L5FilterID>
}


impl RxQueue<{RxState::Enabled}> {
    pub(crate) fn new(mut regs: RxQueueRegisters, num_desc: NumDesc, cpu_id: Option<u8>) -> Result<RxQueue<{RxState::Enabled}>, &'static str> {
        // create the descriptor ring
        let (mut rx_descs, rx_descs_starting_phys_addr) = create_desc_ring::<LegacyRxDescriptor>(num_desc)?;
        let num_rx_descs = rx_descs.len();

        // create a buffer pool with 2KiB size buffers. This ensures that 1 ethernet frame (which can be 1.5KiB) will always fit in one buffer
        let mut rx_buffer_pool = Mempool::new(num_rx_descs * 3)?;

        // now that we've created the rx descriptors, we can fill them in with initial values
        let mut rx_bufs_in_use: Vec<PacketBuffer> = Vec::with_capacity(num_rx_descs);
        for rd in rx_descs.iter_mut()
        {
            // obtain a receive buffer for each rx_desc
            // letting this fail instead of allocating here alerts us to a logic error, we should always have more buffers in the pool than the fdescriptor ringh
            let rx_buf = rx_buffer_pool.pop()
                .ok_or("Couldn't obtain a ReceiveBuffer from the pool")?; 
            
            rd.init(rx_buffer_pool.paddr(&rx_buf)); 
            rx_bufs_in_use.push(rx_buf); 
        }

        // debug!("intel_ethernet::init_rx_queue(): phys_addr of rx_desc: {:#X}", rx_descs_starting_phys_addr);
        let rx_desc_phys_addr_lower  = rx_descs_starting_phys_addr.value() as u32;
        let rx_desc_phys_addr_higher = (rx_descs_starting_phys_addr.value() >> 32) as u32;
        
        // write the physical address of the rx descs ring
        regs.rdbal.write(rx_desc_phys_addr_lower);
        regs.rdbah.write(rx_desc_phys_addr_higher);

        // write the length (in total bytes) of the rx descs array
        regs.rdlen_write(num_desc); // should be 128 byte aligned, minimum 8 descriptors
        
        // Write the head index (the first receive descriptor)
        regs.rdh_write(0);
        regs.rdt_write(0);   

        Ok(RxQueue { 
            id: QueueID::try_from(regs.id() as u8).map_err(|_| "tried to create queue with id >= 64")?, 
            regs, rx_descs, 
            num_rx_descs: num_rx_descs as u16, 
            rx_cur: 0, 
            rx_bufs_in_use, 
            rx_buffer_size: DEFAULT_RX_BUFFER_SIZE_2KB, 
            rx_buffer_pool,
            cpu_id,
            filter_num: None
        })
    }

    /// Retrieves a maximum of `batch_size` number of packets and stores them in `buffers`.
    /// Returns the total number of received packets.
    #[inline(always)]
    pub fn rx_batch(&mut self, buffers: &mut Vec<PacketBuffer>, batch_size: usize, pool: &mut Mempool) -> u16 {
        // verified_functions::rx_batch(
        //     &mut self.rx_descs, 
        //     &mut self.rx_cur, 
        //     &mut self.rx_bufs_in_use, 
        //     &mut self.regs, 
        //     self.num_rx_descs, 
        //     buffers, 
        //     batch_size, 
        //     pool
        // )

        let mut rx_cur = self.rx_cur;
        let mut last_rx_cur = self.rx_cur;

        let mut rcvd_pkts = 0;

        for _ in 0..batch_size {
            let desc = &mut self.rx_descs[rx_cur as usize];

            if !desc.descriptor_done() {
                break;
            }

            if !desc.end_of_packet() {
                error!("intel_ethernet::rx_batch(): multi-descriptor packets are not supported yet!");
                panic!();
                // return Err("Currently do not support multi-descriptor packets");
            }

            let length = desc.length();

            // Now that we are "removing" the current receive buffer from the list of receive buffers that the NIC can use,
            // (because we're saving it for higher layers to use),
            // we need to obtain a new `ReceiveBuffer` and set it up such that the NIC will use it for future receivals.
            if let Some(new_receive_buf) = pool.pop() {
                // actually tell the NIC about the new receive buffer, and that it's ready for use now
                desc.set_packet_address(pool.paddr(&new_receive_buf));
                desc.reset_status();
                
                let mut current_rx_buf = core::mem::replace(&mut self.rx_bufs_in_use[rx_cur as usize], new_receive_buf);
                // current_rx_buf.length = length as u16; // set the ReceiveBuffer's length to the size of the actual packet received
                // unsafe{ core::arch::x86_64::_mm_prefetch(current_rx_buf.buffer.as_ptr() as *const i8, _MM_HINT_ET0);}
                current_rx_buf.set_length(length as u16); // set the ReceiveBuffer's length to the size of the actual packet received
                buffers.push(current_rx_buf);

                rcvd_pkts += 1;
                last_rx_cur = rx_cur;
                rx_cur = (rx_cur + 1) & (self.num_rx_descs - 1);
            } else {
                error!("Ran out of packet buffers in the pool!");
                panic!("Ran out of packet buffers in the pool!");
            }
        }

        if last_rx_cur != rx_cur {
            self.rx_cur = rx_cur as u16;
            self.regs.rdt_write(last_rx_cur); 
        }

        rcvd_pkts
    }
}

impl Deref for RxQueue<{RxState::Enabled}> {
    type Target = BorrowedSliceMappedPages<LegacyRxDescriptor, Mutable>;

    fn deref(&self) -> &Self::Target {
        &self.rx_descs
    }
}

impl DerefMut for RxQueue<{RxState::Enabled}> {    
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.rx_descs
    }
}





#[derive(PartialEq, Eq)]
pub enum RxState {
    Disabled,
    Enabled,
    L5Filter,
    RSS
}


// // implementation of pseudo functions that should only be used for testing
// impl RxQueue<{RxState::Enabled}> {
//     /// Simply iterates through a max of `batch_size` descriptors and resets the descriptors. 
//     /// There is no buffer management, and this function simply reuses the packet buffer that's already stored.
//     /// Returns the total number of received packets.
//     pub fn rx_batch_pseudo(&mut self, batch_size: usize) -> usize {
//         let mut rx_cur = self.rx_cur as usize;
//         let mut last_rx_cur = self.rx_cur as usize;

//         let mut rcvd_pkts = 0;

//         for _ in 0..batch_size {
//             let desc = &mut self.rx_descs[rx_cur];
//             if !desc.descriptor_done() {
//                 break;
//             }

//             // actually tell the NIC about the new receive buffer, and that it's ready for use now
//             desc.set_packet_address(self.rx_bufs_in_use[rx_cur].phys_addr());
//             desc.reset_status();
                
//             rcvd_pkts += 1;
//             last_rx_cur = rx_cur;
//             rx_cur = (rx_cur + 1) & (self.num_rx_descs as usize - 1);
//         }

//         if last_rx_cur != rx_cur {
//             self.rx_cur = rx_cur as u16;
//             self.regs.rdt_write(last_rx_cur as u16); 
//         }

//         rcvd_pkts
//     }

// }