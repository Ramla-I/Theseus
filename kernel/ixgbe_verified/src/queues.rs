use memory::{MappedPages, create_contiguous_mapping, PhysicalAddress, EntryFlags};
use zerocopy::FromBytes;
use crate::{hal::descriptors::{AdvancedTxDescriptor, AdvancedRxDescriptor, Descriptor}, packet_buffers::MTU, RxBufferSizeKiB, DEFAULT_RX_BUFFER_SIZE_2KB};
use crate::queue_registers::{TxQueueRegisters, RxQueueRegisters};
use crate::NumDesc;
use crate::packet_buffers::PacketBuffer;
use crate::allocator::*;
use owning_ref::BoxRefMut;
use core::{ops::{DerefMut, Deref}};
use alloc::{
    boxed::Box,
    vec::Vec,
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

        Ok(TxQueue { id: regs.id() as u8, regs, tx_descs, num_tx_descs: num_tx_descs as u16, tx_cur: 0, cpu_id })
    }

    /// Sends a packet on the transmit queue
    /// 
    /// # Arguments:
    /// * `transmit_buffer`: buffer containing the packet to be sent
    pub fn send_on_queue(&mut self, packet_buffer: PacketBuffer<{MTU::Standard}>) {
        self.tx_descs[self.tx_cur as usize].send(packet_buffer.phys_addr, packet_buffer.length);  
        // update the tx_cur value to hold the next free descriptor
        let old_cur = self.tx_cur;
        self.tx_cur = (self.tx_cur + 1) % self.num_tx_descs;
        // update the tdt register by 1 so that it knows the previous descriptor has been used
        // and has a packet to be sent
        self.regs.tdt.write(self.tx_cur as u32);
        // Wait for the packet to be sent
        self.tx_descs[old_cur as usize].wait_for_packet_tx();
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
    pub rx_bufs_in_use: Vec<PacketBuffer<{MTU::Standard}>>,
    pub rx_buffer_size: RxBufferSizeKiB,
    /// Pool where `ReceiveBuffer`s are stored.
    pub rx_buffer_pool: Vec<PacketBuffer<{MTU::Standard}>>,
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
        let mut rx_buffer_pool = init_rx_buf_pool(num_rx_descs * 2, DEFAULT_RX_BUFFER_SIZE_2KB)?;

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