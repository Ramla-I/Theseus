use memory::{BorrowedSliceMappedPages, BorrowedMappedPages, Mutable, PhysicalAddress, create_contiguous_mapping};
use crate::queue_registers::{RxQueueRegisters, TxQueueRegisters};
use crate::{NumDesc, IxgbeNic, DescType, RxBufferSizeKiB, U7, HThresh};
use crate::allocator::*;
use crate::regs::{IntelIxgbeRegisters1, IntelIxgbeRegisters2};
use crate::descriptor::*;
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
    fn receive(&mut self, packet_length: &mut PacketLength) -> Option<&mut EthernetFrame> {
        let (dd, length) = self.desc_ring[self.processed_delimiter as usize].rx_metadata();
        // let rx_metadata = unsafe{ self.desc_ring.get_unchecked(self.processed_delimiter as usize).other.read()};

        if !dd {
            // no packet
            if self.flush_counter != 0 {
                self.tx_regs.tdt_write(self.processed_delimiter);
                self.flush_counter = 0;
            }
            return None;
        }
        *packet_length = length; 
        Some(&mut self.buffer[self.processed_delimiter as usize])
    }

    #[inline(always)]
    fn transmit(&mut self, packet_length: PacketLength) {
        let rs_bit = if (self.processed_delimiter as u64 & (IXGBE_AGENT_RECYCLE_PERIOD - 1)) == (IXGBE_AGENT_RECYCLE_PERIOD - 1) { 1 << (24 + 3) } else { 0 };
        self.desc_ring[self.processed_delimiter as usize].send(packet_length, rs_bit);
        // unsafe{ self.desc_ring.get_unchecked_mut(self.processed_delimiter as usize).other.write(packet_length as u64 | rs_bit | (1 << (24 + 1)) | (1 << 24)); }
        self.processed_delimiter = (self.processed_delimiter + 1) & (IXGBE_RING_SIZE - 1);

        self.flush_counter += 1;
        if self.flush_counter == IXGBE_AGENT_FLUSH_PERIOD {
            self.tx_regs.tdt_write(self.processed_delimiter);
            self.flush_counter = 0;
        }

        if rs_bit != 0 {
            let head = self.head_wb.value.read() as u16;
            self.rx_regs.rdt_write((head - 1) & (IXGBE_RING_SIZE - 1));
        }
    }

    #[inline(always)]
    pub fn run(&mut self){
        let mut packet_length = PacketLength::zero();
        
        for i in 0.. IXGBE_AGENT_FLUSH_PERIOD {
            if let Some(pkt) = self.receive(&mut packet_length) {
                pkt.src_addr = [0,0,0,0,0,0];
                pkt.dest_addr = [0,0,0,0,0,1];
                self.transmit(packet_length);
            } else {
                break;
            }
        }
    }
}

