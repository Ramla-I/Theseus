use memory::{BorrowedSliceMappedPages, BorrowedMappedPages, Mutable, PhysicalAddress, create_contiguous_mapping};
use crate::queue_registers::{RxQueueRegisters, TxQueueRegisters};
use crate::{NumDesc, IxgbeNic, DescType, RxBufferSizeKiB, U7, HThresh};
use crate::allocator::*;
use crate::regs::{TDHSet, IntelIxgbeRegisters1, IntelIxgbeRegisters2};
use packet_buffers::EthernetFrame;
use volatile::Volatile;
use zerocopy::FromBytes;

const IXGBE_AGENT_RECYCLE_PERIOD: u64 = 32;
const IXGBE_AGENT_FLUSH_PERIOD: u64 = 8;
const IXGBE_RING_SIZE: u16 = 1024;

#[derive(FromBytes)]
#[repr(C)]
pub struct TransmitHead {
    pub value: Volatile<u32>,
}

pub struct IxgbeAgent {
    pub(crate) rx_regs: RxQueueRegisters,
    pub(crate) tx_regs: TxQueueRegisters,
    pub desc_ring: BorrowedSliceMappedPages<LegacyDescriptor, Mutable>,
    pub buffer: BorrowedSliceMappedPages<EthernetFrame, Mutable>,
    pub head_wb: BorrowedMappedPages<TransmitHead, Mutable>,
    pub num_descs: u16,
    pub processed_delimiter: u16,
    pub flush_counter: u64,
    pub tx_clean: u16,
    pub descs_paddr: PhysicalAddress,
    pub buffers_paddr: PhysicalAddress,
    pub head_wb_paddr: PhysicalAddress
}

impl IxgbeAgent {
    pub fn new(device_rx: &mut IxgbeNic, device_tx: &mut IxgbeNic) -> Result<IxgbeAgent, &'static str> {
        let num_desc = NumDesc::Descs1k;
        assert!(num_desc as u16 == IXGBE_RING_SIZE);
        let (buffers_mp, buffers_paddr) = create_contiguous_mapping(num_desc as usize * core::mem::size_of::<EthernetFrame>(), NIC_MAPPING_FLAGS_CACHED)?;
        let (descs_mp, descs_paddr) = create_contiguous_mapping(num_desc as usize * core::mem::size_of::<LegacyDescriptor>(), NIC_MAPPING_FLAGS_CACHED)?;
        let (head_wb_mp, head_wb_paddr) = create_contiguous_mapping(core::mem::size_of::<TransmitHead>(), NIC_MAPPING_FLAGS_CACHED)?;

        let mut desc_ring = descs_mp.into_borrowed_slice_mut(0, num_desc as usize).map_err(|(_mp, err)| err)?;

        // should be impossible for the queues to be enabled at this point in our current design
        // later on we should check if the rxq registers are alreadyin use
        Self::rx_init(num_desc, descs_paddr, device_rx.rxq_registers.as_mut().unwrap(), &mut device_rx.regs1);
        Self::tx_init(num_desc, buffers_paddr, descs_paddr, head_wb_paddr, &mut desc_ring, device_tx.txq_registers.as_mut().unwrap(), &mut device_tx.regs2);

        Ok(IxgbeAgent {
            rx_regs: device_rx.rxq_registers.take().unwrap(),
            tx_regs: device_tx.txq_registers.take().unwrap(),
            desc_ring,
            buffer: buffers_mp.into_borrowed_slice_mut(0, num_desc as usize).map_err(|(_mp, err)| err)?,
            head_wb: head_wb_mp.into_borrowed_mut(0).map_err(|(_mp, err)| err)?,
            num_descs: num_desc as u16,
            processed_delimiter: 0,
            flush_counter: 0,
            tx_clean: 0,
            descs_paddr,
            buffers_paddr,
            head_wb_paddr
        })
    }

    pub fn new_single(device: &mut IxgbeNic) -> Result<IxgbeAgent, &'static str> {
        let num_desc = NumDesc::Descs1k;
        assert!(num_desc as u16 == IXGBE_RING_SIZE);
        let (buffers_mp, buffers_paddr) = create_contiguous_mapping(num_desc as usize * core::mem::size_of::<EthernetFrame>(), NIC_MAPPING_FLAGS_CACHED)?;
        let (descs_mp, descs_paddr) = create_contiguous_mapping(num_desc as usize * core::mem::size_of::<LegacyDescriptor>(), NIC_MAPPING_FLAGS_CACHED)?;
        let (head_wb_mp, head_wb_paddr) = create_contiguous_mapping(core::mem::size_of::<TransmitHead>(), NIC_MAPPING_FLAGS_CACHED)?;
        
        let mut desc_ring = descs_mp.into_borrowed_slice_mut(0, num_desc as usize).map_err(|(_mp, err)| err)?;

        // should be impossible for the queues to be enabled at this point in our current design
        // later on we should check if the rxq registers are alreadyin use
        Self::rx_init(num_desc, descs_paddr, device.rxq_registers.as_mut().unwrap(), &mut device.regs1);
        Self::tx_init(num_desc, buffers_paddr, descs_paddr, head_wb_paddr, &mut desc_ring, device.txq_registers.as_mut().unwrap(), &mut device.regs2);

        Ok(IxgbeAgent {
            rx_regs: device.rxq_registers.take().unwrap(),
            tx_regs: device.txq_registers.take().unwrap(),
            desc_ring,
            buffer: buffers_mp.into_borrowed_slice_mut(0, num_desc as usize).map_err(|(_mp, err)| err)?,
            head_wb: head_wb_mp.into_borrowed_mut(0).map_err(|(_mp, err)| err)?,
            num_descs: num_desc as u16,
            processed_delimiter: 0,
            flush_counter: 0,
            tx_clean: 0,
            descs_paddr,
            buffers_paddr,
            head_wb_paddr
        })
    }

    fn rx_init(num_desc: NumDesc, descs_paddr: PhysicalAddress, rxq_regs: &mut RxQueueRegisters, regs1: &mut IntelIxgbeRegisters1) {
        // write the physical address of the rx descs ring
        rxq_regs.rdbal.write(descs_paddr.value() as u32);
        rxq_regs.rdbah.write((descs_paddr.value() >> 32) as u32);
        // write the length (in total bytes) of the rx descs array
        // should be 128 byte aligned, minimum 8 descriptors
        rxq_regs.rdlen_write(num_desc); 
        // set the size of the packet buffers(leave default value) and the descriptor format used
        rxq_regs.srrctl_write(DescType::Legacy, RxBufferSizeKiB::Buffer2KiB);
        rxq_regs.srrctl_drop_enable();
        // enable the rx queue and make sure it's enabled
        rxq_regs.rxdctl_rxq_enable();
        const RX_Q_ENABLE: u32 = 1 << 25;
        while rxq_regs.rxdctl_read() & RX_Q_ENABLE == 0 {}
        // Write the tail index.
        // Note that the 82599 datasheet (section 8.2.3.8.5) states that we should set the RDT (tail index) to the index *beyond* the last receive descriptor, 
        // but we set it to the last receive descriptor for the same reason as the e1000 driver
        rxq_regs.rdt_write(num_desc as u16 - 1);
        
        {
            // *** Here we do the operations that should only be done once, not per receive queue 
            // *** Since we only enable one queue by default, and don't have support for a software reset, we don't check if the rx queue is already enabled
            // *** or if this has already been done
            // TinyNF also programs secrxctrl here, but let's ignore that for now
            // TinyNf also enables rx in rxctrl, but we already did that in the promiscuous mode function
            regs1.ctrl_ext_no_snoop_disable();
        }

        // set bit 12 to 0
        rxq_regs.dca_rxctrl_clear_bit_12();
    }

    fn tx_init(num_desc: NumDesc, buffers_paddr: PhysicalAddress, descs_paddr: PhysicalAddress, head_wb_paddr: PhysicalAddress, desc_ring: &mut [LegacyDescriptor], txq_regs: &mut TxQueueRegisters, regs2: &mut IntelIxgbeRegisters2) {
        // set buffer addresses
        for i in 0..num_desc as usize {
            let packet_paddr = buffers_paddr + (i * core::mem::size_of::<EthernetFrame>());
            desc_ring[i].phys_addr.write(packet_paddr.value() as u64);
        }
        // write the physical address of the tx descs ring
        txq_regs.tdbal.write(descs_paddr.value() as u32); 
        txq_regs.tdbah.write((descs_paddr.value() >> 32) as u32); 
        // write the length (in total bytes) of the tx desc ring
        let tdlen_set = txq_regs.tdlen_write(num_desc);               
        // Set tx descriptor pre-fetch threshold and host threshold 
        let pthresh = U7::B5 | U7::B4 | U7::B3 | U7::B2;// U7::B5 | U7::B2; // b100100 = 36 (DPDK), TInyNF uses 60 b111100
        let hthresh = HThresh::B2; //HThresh::B3; // b1000 = 8 (DPDK), TinyNF uses 4  b100
        txq_regs.txdctl_write_pthresh_hthresh(pthresh, hthresh); 
        // here we should set the head writeback address
        txq_regs.tdwba_set_and_enable(head_wb_paddr.value() as u64);
        txq_regs.dca_txctrl_disable_relaxed_ordering_head_wb();
        {
            // *** Here we do the operations that should only be done once, not per transmit queue
            regs2.dmatxctl_enable_tx();
        }
        // write the head index and the tail index (both 0 initially because there are no tx requests yet)
        let tdh_set = txq_regs.tdh_write(0, tdlen_set);
        txq_regs.tdt_write(0);
        //enable tx queue and make sure it's enabled
        txq_regs.txdctl_txq_enable(tdh_set); 
        const TX_Q_ENABLE: u32 = 1 << 25;
        while txq_regs.txdctl_read() & TX_Q_ENABLE == 0 {} 
    }

    #[inline(always)]
    fn receive(&mut self, packet_length: &mut u16) -> Option<&mut EthernetFrame> {
        // error!("rx: {}", self.processed_delimiter);

        let rx_metadata = self.desc_ring[self.processed_delimiter as usize].other.read();
        if rx_metadata & (1 << 32) == 0 {
            // no packet
            if self.flush_counter != 0 {
                self.tx_regs.tdt_write(self.processed_delimiter);
                self.flush_counter = 0;
            }
            return None;
        }
        *packet_length = rx_metadata as u16 & 0xFFFF; 
        // error!("rx pkt received: {} {}", self.processed_delimiter, packet_length);

        Some(&mut self.buffer[self.processed_delimiter as usize])
    }

    #[inline(always)]
    fn transmit(&mut self, packet_length: u16) {
        // error!("tx: {} {} {}", self.processed_delimiter, self.tx_clean, self.flush_counter);

        let rs_bit = if (self.processed_delimiter as u64 & (IXGBE_AGENT_RECYCLE_PERIOD - 1)) == (IXGBE_AGENT_RECYCLE_PERIOD - 1) { 1 << (24 + 3) } else { 0 };
        self.desc_ring[self.processed_delimiter as usize].other.write(packet_length as u64 | rs_bit | (1 << (24 + 1)) | (1 << 24));
        self.processed_delimiter = (self.processed_delimiter + 1) & (IXGBE_RING_SIZE - 1);

        self.flush_counter += 1;
        if self.flush_counter == IXGBE_AGENT_FLUSH_PERIOD {
            self.tx_regs.tdt_write(self.processed_delimiter);
            self.flush_counter = 0;
        }

        if rs_bit != 0 {
            // self.tx_clean();
            let head = self.head_wb.value.read() as u16;
            // if head == 0 { 
            //     self.rx_regs.rdt_write(IXGBE_RING_SIZE - 1);
            // } else {
                self.rx_regs.rdt_write((head - 1) & (IXGBE_RING_SIZE - 1));
            // }
            // error!("rs: {} {}", rs_bit, (self.tx_clean - 1) & (IXGBE_RING_SIZE - 1));
        }
        // error!("tx: {} {} {}", self.processed_delimiter, self.tx_clean, self.flush_counter);


    }

    #[inline(always)]
    fn tx_clean(&mut self)  {
        const TX_CLEAN_BATCH: usize = IXGBE_AGENT_RECYCLE_PERIOD as usize;

        let mut tx_clean = self.tx_clean as usize;
        let tx_cur = self.processed_delimiter;

        loop {
            let mut cleanable = tx_cur as i32 - tx_clean as i32;

            if cleanable < 0 {
                cleanable += IXGBE_RING_SIZE as i32;
            }
    
            if cleanable < TX_CLEAN_BATCH as i32 {
                break;
            }
    
            let mut cleanup_to = tx_clean + TX_CLEAN_BATCH - 1;

            if cleanup_to >= IXGBE_RING_SIZE as usize {
                cleanup_to -= IXGBE_RING_SIZE as usize;
            }

            if self.desc_ring[cleanup_to].desc_done() {
                tx_clean = (cleanup_to + 1) % IXGBE_RING_SIZE as usize;
                self.desc_ring[cleanup_to].other.write(0);
            } else {
                break;
            }
        }

        self.tx_clean = tx_clean as u16;
    }

    // #[inline(always)]
    // pub fn run(&mut self, src_addr: &[u8; 6], print: bool) -> usize {
    //     let mut packet_length = 0;
    //     let mut received = 0;

    //     for i in 0.. IXGBE_AGENT_FLUSH_PERIOD {
    //         if let Some(pkt) = self.receive(&mut packet_length) {
    //             pkt.src_addr = *src_addr;
    //             pkt.dest_addr = [0,0,0,0,0,1];
    //             self.transmit(packet_length);
    //             received += 1;
    //         } else {
    //             break;
    //         }
    //     }
    //     if print{
    //         error!("no packet received: {} {} {} {} {}", self.processed_delimiter, self.tx_clean, self.flush_counter, self.tx_regs.tdt_read(), self.rx_regs.rdt_read());
    //     }

    //     received
    // }

    #[inline(always)]
    pub fn tx(&mut self) -> usize {
        // let rs_bit = if (self.processed_delimiter as u64 & (IXGBE_AGENT_RECYCLE_PERIOD - 1)) == (IXGBE_AGENT_RECYCLE_PERIOD - 1) { 1 << (24 + 3) } else { 0 };

        // if rs_bit != 0 {
        //     self.tx_clean();
        //     // self.rx_regs.rdt_write((self.tx_clean - 1) & (IXGBE_RING_SIZE - 1));
        //     // error!("rs: {} {}", rs_bit, (self.tx_clean - 1) & (IXGBE_RING_SIZE - 1));
        // }
        let next = (self.processed_delimiter + 1) & (IXGBE_RING_SIZE - 1);
        self.tx_clean = self.head_wb.value.read() as u16;
        if next == self.tx_clean {
            return 0;
        }
        let rs_bit = if (self.processed_delimiter as u64 & (IXGBE_AGENT_RECYCLE_PERIOD - 1)) == (IXGBE_AGENT_RECYCLE_PERIOD - 1) { 1 << (24 + 3) } else { 0 };
        self.desc_ring[self.processed_delimiter as usize].other.write(60 | rs_bit | (1 << (24 + 1)) | (1 << 24));
        self.processed_delimiter = next;

        self.flush_counter += 1;
        if self.flush_counter == IXGBE_AGENT_FLUSH_PERIOD {
            self.tx_regs.tdt_write(self.processed_delimiter);
            self.flush_counter = 0;
        }

        // if rs_bit != 0 {
        //     self.tx_clean = self.head_wb.value.read() as u16;
        // }
        1
    }

    #[inline(always)]
    pub fn rx(&mut self, packet_length: &mut u16) -> usize {
        // error!("rx: {}", self.processed_delimiter);

        let rx_metadata = self.desc_ring[self.processed_delimiter as usize].other.read();
        if rx_metadata & (1 << 32) == 0 {
            // no packet
            // if self.flush_counter != 0 {
            //     self.tx_regs.tdt_write(self.processed_delimiter);
            //     self.flush_counter = 0;
            // }
            return 0;
        }
        self.desc_ring[self.processed_delimiter as usize].other.write(0);
        *packet_length = rx_metadata as u16 & 0xFFFF; 

        // error!("rx pkt received: {} {}", self.processed_delimiter, packet_length);
        self.flush_counter += 1;
        if self.flush_counter == 32 {
            self.rx_regs.rdt_write(self.processed_delimiter);
            self.flush_counter = 0;
        }
        self.processed_delimiter = (self.processed_delimiter + 1) & (IXGBE_RING_SIZE - 1);
        1
    }   

    #[inline(always)]
    pub fn run(&mut self){
        let mut packet_length = 0;
        // let mut received = 0;

        for i in 0.. IXGBE_AGENT_FLUSH_PERIOD {
            if let Some(pkt) = self.receive(&mut packet_length) {
                pkt.src_addr = [0,0,0,0,0,0];
                pkt.dest_addr = [0,0,0,0,0,1];
                self.transmit(packet_length);
                // received += 1;
            } else {
                break;
            }
        }
        // if print{
        //     error!("no packet received: {} {} {} {} {} {}", self.processed_delimiter, self.tx_clean, self.flush_counter, self.tx_regs.tdt_read(), self.rx_regs.rdt_read(), self.head_wb.value.read());
        // }
        // received
    }
}

/// This struct is a Legacy Transmit Descriptor. 
/// It's the descriptor type used in older Intel NICs and the E1000 driver.
#[derive(FromBytes)]
#[repr(C)]
pub struct LegacyDescriptor {
    /// The starting physical address of the transmit buffer
    pub phys_addr:  Volatile<u64>,
    pub other: Volatile<u64>
}

impl LegacyDescriptor {
    #[inline(always)]
    fn desc_done(&self) -> bool {
        (self.other.read() & (1 << 32)) == (1 << 32)
    }
}