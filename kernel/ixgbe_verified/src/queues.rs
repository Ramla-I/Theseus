use memory::{MappedPages, create_contiguous_mapping, PhysicalAddress, EntryFlags};
use zerocopy::FromBytes;
use crate::{hal::descriptors::{AdvancedTxDescriptor, AdvancedRxDescriptor, Descriptor}, packet_buffers::{MTU, PacketBufferS}, RxBufferSizeKiB, DEFAULT_RX_BUFFER_SIZE_2KB};
use crate::queue_registers::{TxQueueRegisters, RxQueueRegisters};
use crate::NumDesc;
use crate::packet_buffers::PacketBuffer;
use crate::allocator::*;
use owning_ref::BoxRefMut;
use core::{ops::{DerefMut, Deref}};
use alloc::{
    boxed::Box,
    vec::Vec, collections::VecDeque,
};

/// A struct that holds all information for a transmit queue. 
/// There should be one such object per queue.
pub struct TxQueue {
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

impl TxQueue {
    pub(crate) fn new(mut regs: TxQueueRegisters, num_desc: NumDesc, cpu_id: Option<u8>) -> Result<TxQueue, &'static str> {
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
}


impl Deref for TxQueue {
    type Target = BoxRefMut<MappedPages, [AdvancedTxDescriptor]>;

    fn deref(&self) -> &Self::Target {
        &self.tx_descs
    }
}

impl DerefMut for TxQueue {    
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.tx_descs
    }
}


/// A struct that holds all information for one receive queue.
/// There should be one such object per queue.
pub struct RxQueue {
    /// The number of the queue, stored here for our convenience.
    pub id: u8,
    /// Registers for this receive queue
    pub regs: RxQueueRegisters,
    /// Receive descriptors
    pub rx_descs: BoxRefMut<MappedPages, [AdvancedRxDescriptor]>,
    /// The number of receive descriptors in the descriptor ring
    pub num_rx_descs: u16,
    /// Current receive descriptor index
    pub rx_cur: u16,
    /// The list of rx buffers, in which the index in the vector corresponds to the index in `rx_descs`.
    /// For example, `rx_bufs_in_use[2]` is the receive buffer that will be used when `rx_descs[2]` is the current rx descriptor (rx_cur = 2).
    pub rx_bufs_in_use: Vec<PacketBufferS>,
    pub rx_buffer_size: RxBufferSizeKiB,
    /// Pool where `ReceiveBuffer`s are stored.
    pub rx_buffer_pool: Vec<PacketBufferS>,
    /// The cpu which this queue is mapped to. 
    /// This in itself doesn't guarantee anything, but we use this value when setting the cpu id for interrupts and DCA.
    pub cpu_id: Option<u8>,
    /// The filter id for the physical NIC filter that is set for this queue
    pub filter_num: Option<u8>
}

impl RxQueue {
    pub(crate) fn new(mut regs: RxQueueRegisters, num_desc: NumDesc, cpu_id: Option<u8>) -> Result<RxQueue, &'static str> {
        // create the descriptor ring
        let (mut rx_descs, rx_descs_starting_phys_addr) = create_desc_ring::<AdvancedRxDescriptor>(num_desc)?;
        let num_rx_descs = rx_descs.len();

        // create a buffer pool with 2KiB size buffers. This ensures that 1 ethernet frame (which can be 1.5KiB) will always fit in one buffer
        let mut rx_buffer_pool = init_rx_buf_pool(num_rx_descs * 2)?;

        // now that we've created the rx descriptors, we can fill them in with initial values
        let mut rx_bufs_in_use: Vec<PacketBuffer<{MTU::Standard}>> = Vec::with_capacity(num_rx_descs);
        for rd in rx_descs.iter_mut()
        {
            // obtain a receive buffer for each rx_desc
            // letting this fail instead of allocating here alerts us to a logic error, we should always have more buffers in the pool than the fdescriptor ringh
            let rx_buf = rx_buffer_pool.pop()
                .ok_or("Couldn't obtain a ReceiveBuffer from the pool")?; 
            
            rd.init(rx_buf.phys_addr); 
            rx_bufs_in_use.push(rx_buf); 
        }

        // debug!("intel_ethernet::init_rx_queue(): phys_addr of rx_desc: {:#X}", rx_descs_starting_phys_addr);
        let rx_desc_phys_addr_lower  = rx_descs_starting_phys_addr.value() as u32;
        let rx_desc_phys_addr_higher = (rx_descs_starting_phys_addr.value() >> 32) as u32;
        
        // write the physical address of the rx descs ring
        regs.rdbal.write(rx_desc_phys_addr_lower);
        regs.rdbah.write(rx_desc_phys_addr_higher);

        // write the length (in total bytes) of the rx descs array
        regs.rdlen.write((num_rx_descs * core::mem::size_of::<AdvancedRxDescriptor>()) as u32); // should be 128 byte aligned, minimum 8 descriptors
        
        // Write the head index (the first receive descriptor)
        regs.rdh.write(0);
        regs.rdt.write(0);   

        Ok(RxQueue { 
            id: regs.id() as u8, 
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
    pub fn rx_batch(&mut self, buffers: &mut Vec<PacketBufferS>, batch_size: usize, pool: &mut Vec<PacketBufferS>) -> Result<usize, &'static str> {
        let mut rx_cur = self.rx_cur as usize;
        let mut last_rx_cur = self.rx_cur as usize;

        let mut rcvd_pkts = 0;

        for _ in 0..batch_size {
            let desc = &mut self.rx_descs[rx_cur];

            if !desc.descriptor_done() {
                break;
            }

            if !desc.end_of_packet() {
                return Err("Currently do not support multi-descriptor packets");
            }

            let length = desc.length();

            // Now that we are "removing" the current receive buffer from the list of receive buffers that the NIC can use,
            // (because we're saving it for higher layers to use),
            // we need to obtain a new `ReceiveBuffer` and set it up such that the NIC will use it for future receivals.
            if let Some(new_receive_buf) = pool.pop() {
                // actually tell the NIC about the new receive buffer, and that it's ready for use now
                desc.set_packet_address(new_receive_buf.phys_addr);
                desc.reset_status();
                
                let mut current_rx_buf = core::mem::replace(&mut self.rx_bufs_in_use[rx_cur], new_receive_buf);
                current_rx_buf.length = length as u16; // set the ReceiveBuffer's length to the size of the actual packet received
                buffers.push(current_rx_buf);

                rcvd_pkts += 1;
                last_rx_cur = rx_cur;
                rx_cur = (rx_cur + 1) & (self.num_rx_descs as usize - 1);
            } else {
                return Err("Ran out of packet buffers");
            }
        }

        if last_rx_cur != rx_cur {
            self.rx_cur = rx_cur as u16;
            self.regs.rdt.write(last_rx_cur as u32); 
        }

        Ok(rcvd_pkts)

    }
}

impl Deref for RxQueue {
    type Target = BoxRefMut<MappedPages, [AdvancedRxDescriptor]>;

    fn deref(&self) -> &Self::Target {
        &self.rx_descs
    }
}

impl DerefMut for RxQueue {    
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.rx_descs
    }
}


/// Allocates the memory for a descriptor ring, maps it to a slice of descriptors `T`, and clears each descriptor in the ring.
fn create_desc_ring<T: Descriptor + FromBytes>(num_desc: NumDesc) -> Result<(BoxRefMut<MappedPages, [T]>, PhysicalAddress), &'static str> {
    let size_in_bytes_of_all_tx_descs = num_desc as usize * core::mem::size_of::<T>();
    
    // descriptor rings must be 128 byte-aligned, which is satisfied below because it's aligned to a page boundary.
    let (descs_mapped_pages, descs_starting_phys_addr) = create_contiguous_mapping(size_in_bytes_of_all_tx_descs, EntryFlags::WRITABLE | EntryFlags::NO_EXECUTE)?;

    let mut desc_ring = BoxRefMut::new(Box::new(descs_mapped_pages))
        .try_map_mut(|mp| mp.as_slice_mut::<T>(0, num_desc as usize))?;

    for desc in desc_ring.iter_mut() { desc.clear() }

    Ok((desc_ring, descs_starting_phys_addr))
}



// implementation of pseudo functions that should only be used for testing
impl TxQueue {
    /// Sets all the descriptors in the tx queue with a valid packet buffer but doesn't update the TDT.
    /// Requires that the length of `buffers` is equal to the number of descriptors in the queue
    pub fn tx_populate(&mut self, buffers: &mut Vec<PacketBufferS>) {
        assert!(buffers.len() == self.tx_descs.len());

        for desc in self.tx_descs.iter_mut() {
            let packet = buffers.pop().unwrap();
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

// implementation of pseudo functions that should only be used for testing
impl RxQueue {
    /// Simply iterates through a max of `batch_size` descriptors and resets the descriptors. 
    /// There is no buffer management, and this function simply reuses the packet buffer that's already stored.
    /// Returns the total number of received packets.
    pub fn rx_batch_pseudo(&mut self, batch_size: usize) -> usize {
        let mut rx_cur = self.rx_cur as usize;
        let mut last_rx_cur = self.rx_cur as usize;

        let mut rcvd_pkts = 0;

        for _ in 0..batch_size {
            let desc = &mut self.rx_descs[rx_cur];
            if !desc.descriptor_done() {
                break;
            }

            // actually tell the NIC about the new receive buffer, and that it's ready for use now
            desc.set_packet_address(self.rx_bufs_in_use[rx_cur].phys_addr);
            desc.reset_status();
                
            rcvd_pkts += 1;
            last_rx_cur = rx_cur;
            rx_cur = (rx_cur + 1) & (self.num_rx_descs as usize - 1);
        }

        if last_rx_cur != rx_cur {
            self.rx_cur = rx_cur as u16;
            self.regs.rdt.write(last_rx_cur as u32); 
        }

        rcvd_pkts
    }

}