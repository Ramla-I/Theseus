use core::ops::Deref;

use memory::{create_contiguous_mapping, BorrowedMappedPages, BorrowedSliceMappedPages, Mutable, PhysicalAddress, DMA_FLAGS};
use crate::{DescType, HThresh, IxgbeNic, NumDesc, RegistersRx, RegistersTx, RxBufferSizeKiB, U7};
use crate::regs::{IntelIxgbeRegisters1, IntelIxgbeRegisters2};
use crate::ethernet_frame::EthernetFrame;
use crate::descriptor::*;
use crate::transmit_head_wb::TransmitHead;
use crate::verified::{FlushCounter, ProcessedDelimiter};
use volatile::Volatile;
use zerocopy::FromBytes;
use prusti_memory_buffer::Buffer;

const IXGBE_AGENT_RECYCLE_PERIOD: u64 = 32;
const IXGBE_AGENT_FLUSH_PERIOD: u64 = 8;
const IXGBE_RING_SIZE: NumDesc = NumDesc::Descs1k;

pub struct IxgbeAgent {
    pub(crate) rx_regs: Buffer<RegistersRx>,
    pub(crate) tx_regs: Buffer<RegistersTx>,
    desc_ring: BorrowedSliceMappedPages<LegacyDescriptor, Mutable>,
    buffer: BorrowedSliceMappedPages<EthernetFrame, Mutable>,
    head_wb: BorrowedMappedPages<TransmitHead, Mutable>,
    processed_delimiter: ProcessedDelimiter,
    flush_counter: FlushCounter,
    descs_paddr: PhysicalAddress, // Address of the descriptor ring, stored for speed
    buffers_paddr: PhysicalAddress, // Address of the buffers, stored for speed
    head_wb_paddr: PhysicalAddress // Address of the transmit head writeback, stored for speed
}

impl IxgbeAgent {
    pub fn new(device_rx: &mut IxgbeNic, device_tx: &mut IxgbeNic) -> Result<IxgbeAgent, &'static str> {
        let num_desc = IXGBE_RING_SIZE;

        let (buffers_mp, buffers_paddr) = create_contiguous_mapping(num_desc as usize * core::mem::size_of::<EthernetFrame>(), DMA_FLAGS)?;
        let (descs_mp, descs_paddr) = create_contiguous_mapping(num_desc as usize * core::mem::size_of::<LegacyDescriptor>(), DMA_FLAGS)?;
        let (head_wb_mp, head_wb_paddr) = create_contiguous_mapping(core::mem::size_of::<TransmitHead>(), DMA_FLAGS)?;

        let mut desc_ring = descs_mp.into_borrowed_slice_mut(0, num_desc as usize).map_err(|(_mp, err)| err)?;

        // should be impossible for the queues to be enabled at this point in our current design
        // later on we should check if the rxq registers are already in use
        Self::rx_init(num_desc, descs_paddr, &mut device_rx.regs_rx1.buffers[0], &mut device_rx.regs1);
        Self::tx_init(num_desc, buffers_paddr, descs_paddr, head_wb_paddr, &mut desc_ring, &mut device_tx.regs_tx.buffers[0], &mut device_tx.regs2);

        // clear the head writeback
        let mut head_wb: BorrowedMappedPages<TransmitHead, Mutable> = head_wb_mp.into_borrowed_mut(0).map_err(|(_mp, err)| err)?;
        head_wb.clear();

        Ok(IxgbeAgent {
            rx_regs: device_rx.regs_rx1.buffers.pop_front().unwrap(),
            tx_regs: device_tx.regs_tx.buffers.pop_front().unwrap(),
            desc_ring,
            buffer: buffers_mp.into_borrowed_slice_mut(0, num_desc as usize).map_err(|(_mp, err)| err)?,
            head_wb,
            processed_delimiter: ProcessedDelimiter::new(),
            flush_counter: FlushCounter::new(),
            descs_paddr,
            buffers_paddr,
            head_wb_paddr
        })
    }

    fn rx_init(num_desc: NumDesc, descs_paddr: PhysicalAddress, rxq_regs: &mut RegistersRx, regs1: &mut IntelIxgbeRegisters1) {
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
        rxq_regs.set_rdt(num_desc);
        
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

    fn tx_init(num_desc: NumDesc, buffers_paddr: PhysicalAddress, descs_paddr: PhysicalAddress, head_wb_paddr: PhysicalAddress, desc_ring: &mut [LegacyDescriptor], txq_regs: &mut RegistersTx, regs2: &mut IntelIxgbeRegisters2) {
        // set buffer addresses
        for i in 0..num_desc as usize {
            let packet_paddr = buffers_paddr + (i * core::mem::size_of::<EthernetFrame>());
            desc_ring[i].set_buffer_addr(packet_paddr);
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
        let _ = txq_regs.tdt_write(0);

        //enable tx queue and make sure it's enabled
        txq_regs.txdctl_txq_enable(tdh_set); 
        const TX_Q_ENABLE: u32 = 1 << 25;
        while txq_regs.txdctl_read() & TX_Q_ENABLE == 0 {} 
    }

    #[inline(always)]
    fn receive(&mut self, packet_length: &mut PacketLength) -> Option<&mut EthernetFrame> {
        let processed_delimiter = self.processed_delimiter.value();
        let (dd, length) = self.desc_ring[processed_delimiter as usize].rx_metadata();
        // let rx_metadata = unsafe{ self.desc_ring.get_unchecked(self.processed_delimiter as usize).other.read()};

        // We use linear types to enforce the order:
        // tdt is written -> flush counter is cleared

        if !*dd {
            // no packet
            if self.flush_counter.value() != 0 {
                let tdt_written = self.tx_regs.tdt_write(processed_delimiter);
                self.flush_counter.clear(tdt_written);
            }
            return None;
        }
        *packet_length = length; 
        Some(&mut self.buffer[processed_delimiter as usize])
    }

    #[inline(always)]
    fn transmit(&mut self, packet_length: PacketLength) {
        let processed_delimiter = self.processed_delimiter.value();
        let rs_set = (processed_delimiter as u64 & (IXGBE_AGENT_RECYCLE_PERIOD - 1)) == (IXGBE_AGENT_RECYCLE_PERIOD - 1);
        self.desc_ring[processed_delimiter as usize].send(packet_length, rs_set, DescType::Legacy);
        // unsafe{ self.desc_ring.get_unchecked_mut(self.processed_delimiter as usize).other.write(packet_length as u64 | rs_bit | (1 << (24 + 1)) | (1 << 24)); }
        
        // We use linear types to enforce the order :
        // processed delimiter is incremented -> flush counter is incremented 
        // tdt is written -> flush counter is cleared

        // self.processed_delimiter = (self.processed_delimiter + 1) & (IXGBE_RING_SIZE as u16 - 1);
        let delim_incremented = self.processed_delimiter.increment(IXGBE_RING_SIZE as u16);

        // self.flush_counter += 1;
        self.flush_counter.increment(delim_incremented);
        if self.flush_counter.value() == IXGBE_AGENT_FLUSH_PERIOD {
            let tdt_written = self.tx_regs.tdt_write(self.processed_delimiter.value()); // To Do: just update to take processed delimiter
            // self.flush_counter = 0;
            self.flush_counter.clear(tdt_written);
        }

        if rs_set {
            let head = self.head_wb.read() as u16;
            self.rx_regs.rdt_write((head - 1) & (IXGBE_RING_SIZE as u16 - 1));
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
