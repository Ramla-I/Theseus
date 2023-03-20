use memory::{MappedPages};
use crate::{hal::{*, descriptors::AdvancedRxDescriptor}, packet_buffers::{MTU, PacketBufferS}, RxBufferSizeKiB, DEFAULT_RX_BUFFER_SIZE_2KB, L5FilterID, regs::*};
use crate::vec_wrapper::VecWrapper;
use crate::queue_registers::RxQueueRegisters;
use crate::NumDesc;
use crate::allocator::*;
use crate::verified_functions;
use packet_buffers::PacketBuffer;
use owning_ref::BoxRefMut;
use core::{ops::{DerefMut, Deref}, convert::TryFrom};

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
    pub rx_descs: BoxRefMut<MappedPages, [AdvancedRxDescriptor]>,
    /// The number of receive descriptors in the descriptor ring
    pub num_rx_descs: u16,
    /// Current receive descriptor index
    pub rx_cur: u16,
    /// The list of rx buffers, in which the index in the vector corresponds to the index in `rx_descs`.
    /// For example, `rx_bufs_in_use[2]` is the receive buffer that will be used when `rx_descs[2]` is the current rx descriptor (rx_cur = 2).
    pub rx_bufs_in_use: VecWrapper<PacketBufferS>,
    pub rx_buffer_size: RxBufferSizeKiB,
    /// Pool where `ReceiveBuffer`s are stored.
    pub rx_buffer_pool: VecWrapper<PacketBufferS>,
    /// The cpu which this queue is mapped to. 
    /// This in itself doesn't guarantee anything, but we use this value when setting the cpu id for interrupts and DCA.
    pub cpu_id: Option<u8>,
    /// The filter id for the physical NIC filter that is set for this queue
    pub filter_num: Option<L5FilterID>
}


impl RxQueue<{RxState::Enabled}> {
    pub(crate) fn new(mut regs: RxQueueRegisters, num_desc: NumDesc, cpu_id: Option<u8>) -> Result<RxQueue<{RxState::Enabled}>, &'static str> {
        // create the descriptor ring
        let (mut rx_descs, rx_descs_starting_phys_addr) = create_desc_ring::<AdvancedRxDescriptor>(num_desc)?;
        let num_rx_descs = rx_descs.len();

        // create a buffer pool with 2KiB size buffers. This ensures that 1 ethernet frame (which can be 1.5KiB) will always fit in one buffer
        let mut rx_buffer_pool = init_rx_buf_pool(num_rx_descs * 2)?;

        // now that we've created the rx descriptors, we can fill them in with initial values
        let mut rx_bufs_in_use: VecWrapper<PacketBuffer<{MTU::Standard}>> = VecWrapper::with_capacity(num_rx_descs);
        for rd in rx_descs.iter_mut()
        {
            // obtain a receive buffer for each rx_desc
            // letting this fail instead of allocating here alerts us to a logic error, we should always have more buffers in the pool than the fdescriptor ringh
            let rx_buf = rx_buffer_pool.pop()
                .ok_or("Couldn't obtain a ReceiveBuffer from the pool")?; 
            
            rd.init(rx_buf.phys_addr()); 
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
    pub fn rx_batch(&mut self, buffers: &mut VecWrapper<PacketBufferS>, batch_size: usize, pool: &mut VecWrapper<PacketBufferS>) -> Result<u16, ()> {
        verified_functions::rx_batch(
            &mut self.rx_descs, 
            &mut self.rx_cur, 
            &mut self.rx_bufs_in_use, 
            &mut self.regs, 
            self.num_rx_descs, 
            buffers, 
            batch_size, 
            pool
        )
    }

    /// To Do: disable queue according to data sheet (set registers)
    /// policy descisions: do we empty out all packets waiting to be transmitted?
    pub fn disable(self) -> RxQueue<{RxState::Disabled}> {
        panic!("Not fully implemented");
        RxQueue {
            id: self.id,
            regs: self.regs,
            rx_descs: self.rx_descs,
            num_rx_descs: self.num_rx_descs,
            rx_cur: self.rx_cur,
            rx_bufs_in_use: self.rx_bufs_in_use,
            rx_buffer_size: self.rx_buffer_size,
            rx_buffer_pool: self.rx_buffer_pool,
            cpu_id: self.cpu_id,
            filter_num: self.filter_num
        }
    }

    /// This function personally doesn't change anything about the queue except its state, since all steps to 
    /// start RSS have to be done at the device level and not at the queue level.
    pub(crate) fn add_to_reta(self) -> RxQueue<{RxState::RSS}> {
        RxQueue {
            id: self.id,
            regs: self.regs,
            rx_descs: self.rx_descs,
            num_rx_descs: self.num_rx_descs,
            rx_cur: self.rx_cur,
            rx_bufs_in_use: self.rx_bufs_in_use,
            rx_buffer_size: self.rx_buffer_size,
            rx_buffer_pool: self.rx_buffer_pool,
            cpu_id: self.cpu_id,
            filter_num: self.filter_num
        }
    }


    pub(crate) fn l5_filter(
        self, 
        source_ip: Option<[u8;4]>, 
        dest_ip: Option<[u8;4]>, 
        source_port: Option<u16>, 
        dest_port: Option<u16>, 
        protocol: Option<L5FilterProtocol>, 
        priority: L5FilterPriority, 
        enabled_filters: &mut [bool; 128],
        regs3: &mut IntelIxgbeRegisters3
    ) -> Result<RxQueue<{RxState::L5Filter}>, &'static str> {
        
        if source_ip.is_none() && dest_ip.is_none() && source_port.is_none() && dest_port.is_none() && protocol.is_none() {
            return Err("Must set one of the five filter options");
        }

        // find a free filter
        let filter_num = L5FilterID::try_from(enabled_filters.iter().position(|&r| r == false).ok_or("Ixgbe: No filter available")?)
            .map_err(|_| "Invalid filter ID")?;

        // start off with the filter mask set for all the filters, and clear bits for filters that are enabled
        // bits 29:25 are set to 1.
        let mut filter_mask = L5FilterMaskFlags::zero();

        // IP addresses are written to the registers in big endian form (LSB is first on wire)
        // set the source ip address for the filter
        if let Some (addr) = source_ip {
            regs3.saqf[filter_num as usize].write(((addr[3] as u32) << 24) | ((addr[2] as u32) << 16) | ((addr[1] as u32) << 8) | (addr[0] as u32));
        } else {
            filter_mask = filter_mask | L5FilterMaskFlags::SOURCE_ADDRESS;
        }

        // set the destination ip address for the filter
        if let Some(addr) = dest_ip {
            regs3.daqf[filter_num as usize].write(((addr[3] as u32) << 24) | ((addr[2] as u32) << 16) | ((addr[1] as u32) << 8) | (addr[0] as u32));
        } else {
            filter_mask = filter_mask | L5FilterMaskFlags::DESTINATION_ADDRESS;
        }        

        // set the source port for the filter    
        if let Some(port) = source_port {
            regs3.sdpqf[filter_num as usize].write((port as u32) << SPDQF_SOURCE_SHIFT);
        } else {
            filter_mask = filter_mask | L5FilterMaskFlags::SOURCE_PORT;
        }   

        // set the destination port for the filter    
        if let Some(port) = dest_port {
            let port_val = regs3.sdpqf[filter_num as usize].read();
            regs3.sdpqf[filter_num as usize].write(port_val | (port as u32) << SPDQF_DEST_SHIFT);
        } else {
            filter_mask = filter_mask | L5FilterMaskFlags::DESTINATION_PORT;
        }

        // set the filter protocol
        let mut filter_protocol = L5FilterProtocol::Other;
        if let Some(p) = protocol {
            filter_protocol = p;
        } else {
            filter_mask = filter_mask | L5FilterMaskFlags::PROTOCOL;
        }

        // write the parameters of the filter
        regs3.ftqf_set_filter_and_enable(filter_num, priority, filter_protocol, filter_mask);

        //set the rx queue that the packets for this filter should be sent to
        regs3.l34timir_write(filter_num, self.id);

        //mark the filter as used
        enabled_filters[filter_num as usize] = true;

        Ok(RxQueue {
            id: self.id,
            regs: self.regs,
            rx_descs: self.rx_descs,
            num_rx_descs: self.num_rx_descs,
            rx_cur: self.rx_cur,
            rx_bufs_in_use: self.rx_bufs_in_use,
            rx_buffer_size: self.rx_buffer_size,
            rx_buffer_pool: self.rx_buffer_pool,
            cpu_id: self.cpu_id,
            filter_num: Some(filter_num)
        })
    }
}

impl Deref for RxQueue<{RxState::Enabled}> {
    type Target = BoxRefMut<MappedPages, [AdvancedRxDescriptor]>;

    fn deref(&self) -> &Self::Target {
        &self.rx_descs
    }
}

impl DerefMut for RxQueue<{RxState::Enabled}> {    
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.rx_descs
    }
}



impl RxQueue<{RxState::Disabled}> {
    /// To Do: enable queue according to data sheet (set registers)
    pub fn enable(self) -> RxQueue<{RxState::Enabled}> {
        panic!("Not fully implemented");
        RxQueue {
            id: self.id,
            regs: self.regs,
            rx_descs: self.rx_descs,
            num_rx_descs: self.num_rx_descs,
            rx_cur: self.rx_cur,
            rx_bufs_in_use: self.rx_bufs_in_use,
            rx_buffer_size: self.rx_buffer_size,
            rx_buffer_pool: self.rx_buffer_pool,
            cpu_id: self.cpu_id,
            filter_num: self.filter_num
        }
    }
}

impl RxQueue<{RxState::L5Filter}> {
    /// Retrieves a maximum of `batch_size` number of packets and stores them in `buffers`.
    /// Returns the total number of received packets.
    pub fn rx_batch(&mut self, buffers: &mut VecWrapper<PacketBufferS>, batch_size: usize, pool: &mut VecWrapper<PacketBufferS>) -> Result<u16, ()> {
        verified_functions::rx_batch(
            &mut self.rx_descs, 
            &mut self.rx_cur, 
            &mut self.rx_bufs_in_use, 
            &mut self.regs, 
            self.num_rx_descs, 
            buffers, 
            batch_size, 
            pool
        )
    }

    /// Right now we restrict each queue to be used for only one filter,
    /// so disabling the filter returns it to an enabled state.
    pub fn disable_filter(self, enabled_filters: &mut [bool; 128], regs3: &mut IntelIxgbeRegisters3) -> RxQueue<{RxState::Enabled}> {
        // We can unwrap here because created a RxQueue<L5Filter> always sets the filter_num.
        let filter_num = self.filter_num.unwrap();

        // disables filter by setting enable bit to 0
        regs3.ftqf_disable_filter(filter_num);

        // sets the record in the nic struct to false
        enabled_filters[filter_num as usize] = false;
        
        RxQueue {
            id: self.id,
            regs: self.regs,
            rx_descs: self.rx_descs,
            num_rx_descs: self.num_rx_descs,
            rx_cur: self.rx_cur,
            rx_bufs_in_use: self.rx_bufs_in_use,
            rx_buffer_size: self.rx_buffer_size,
            rx_buffer_pool: self.rx_buffer_pool,
            cpu_id: self.cpu_id,
            filter_num: None
        }
    }
}

impl RxQueue<{RxState::RSS}> {
    /// Retrieves a maximum of `batch_size` number of packets and stores them in `buffers`.
    /// Returns the total number of received packets.
    pub fn rx_batch(&mut self, buffers: &mut VecWrapper<PacketBufferS>, batch_size: usize, pool: &mut VecWrapper<PacketBufferS>) -> Result<u16, ()> {
        verified_functions::rx_batch(
            &mut self.rx_descs, 
            &mut self.rx_cur, 
            &mut self.rx_bufs_in_use, 
            &mut self.regs, 
            self.num_rx_descs, 
            buffers, 
            batch_size, 
            pool
        )
    }

    /// This function personally doesn't change anything about the queue except its state, since all steps to 
    /// start RSS have to be done at the device level and not at the queue level.
    pub(crate) fn add_to_reta(self) -> RxQueue<{RxState::RSS}> {
        self
    }

    /// Once the queue is removed from the RETA it is moved to the Enabled state.
    pub fn remove_from_reta(self) -> RxQueue<{RxState::Enabled}> {
        RxQueue {
            id: self.id,
            regs: self.regs,
            rx_descs: self.rx_descs,
            num_rx_descs: self.num_rx_descs,
            rx_cur: self.rx_cur,
            rx_bufs_in_use: self.rx_bufs_in_use,
            rx_buffer_size: self.rx_buffer_size,
            rx_buffer_pool: self.rx_buffer_pool,
            cpu_id: self.cpu_id,
            filter_num: None
        }
    }
}



#[derive(PartialEq, Eq)]
pub enum RxState {
    Disabled,
    Enabled,
    L5Filter,
    RSS
}


// implementation of pseudo functions that should only be used for testing
impl RxQueue<{RxState::Enabled}> {
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
            desc.set_packet_address(self.rx_bufs_in_use.index_mut(rx_cur).phys_addr());
            desc.reset_status();
                
            rcvd_pkts += 1;
            last_rx_cur = rx_cur;
            rx_cur = (rx_cur + 1) & (self.num_rx_descs as usize - 1);
        }

        if last_rx_cur != rx_cur {
            self.rx_cur = rx_cur as u16;
            self.regs.rdt_write(last_rx_cur as u16); 
        }

        rcvd_pkts
    }

}