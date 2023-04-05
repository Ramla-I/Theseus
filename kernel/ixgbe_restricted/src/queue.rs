use memory::{MappedPages, BorrowedSliceMappedPages, Mutable, PhysicalAddress, create_contiguous_mapping};
use crate::queue_registers::{RxQueueRegisters, TxQueueRegisters};
use crate::NumDesc;
use crate::allocator::*;
use crate::regs::TDHSet;
use core::{ops::{DerefMut, Deref}, convert::TryFrom, arch::x86_64::_MM_HINT_ET0};
use packet_buffers::EthernetFrame;
use volatile::Volatile;
use zerocopy::FromBytes;

const IXGBE_AGENT_RECYCLE_PERIOD: u64 = 32;
const IXGBE_AGENT_FLUSH_PERIOD: u64 = 8;
const IXGBE_RING_SIZE: u16 = 1024;

pub struct Queue {
    pub(crate) rx_regs: RxQueueRegisters,
    pub(crate) tx_regs: TxQueueRegisters,
    pub descs: BorrowedSliceMappedPages<LegacyDescriptor, Mutable>,
    pub buffer: BorrowedSliceMappedPages<EthernetFrame, Mutable>,
    pub num_descs: u16,
    pub processed_delimiter: u16,
    pub flush_counter: u64,
    pub tx_clean: u16,
    pub descs_paddr: PhysicalAddress,
    pub buffers_paddr: PhysicalAddress
}

impl Queue {
    pub(crate) fn new(num_desc: NumDesc, tx_regs: TxQueueRegisters, rx_regs: RxQueueRegisters) -> Result<Queue, &'static str> {
        assert!(num_desc as u16 == IXGBE_RING_SIZE);
        let (buffers_mp, buffers_paddr) = create_contiguous_mapping(num_desc as usize * core::mem::size_of::<EthernetFrame>(), NIC_MAPPING_FLAGS_CACHED)?;
        let (descs_mp, descs_paddr) = create_contiguous_mapping(num_desc as usize * core::mem::size_of::<LegacyDescriptor>(), NIC_MAPPING_FLAGS_CACHED)?;

        Ok(Queue {
            rx_regs,
            tx_regs,
            descs: descs_mp.into_borrowed_slice_mut(0, num_desc as usize).map_err(|(_mp, err)| err)?,
            buffer: buffers_mp.into_borrowed_slice_mut(0, num_desc as usize).map_err(|(_mp, err)| err)?,
            num_descs: num_desc as u16,
            processed_delimiter: 0,
            flush_counter: 0,
            tx_clean: 0,
            descs_paddr,
            buffers_paddr
        })
    }

    pub fn rx_init(&mut self, num_desc: NumDesc) {
        // debug!("intel_ethernet::init_rx_queue(): phys_addr of rx_desc: {:#X}", rx_descs_starting_phys_addr);
        let rx_desc_phys_addr_lower  = self.descs_paddr.value() as u32;
        let rx_desc_phys_addr_higher = (self.descs_paddr.value() >> 32) as u32;
        
        // write the physical address of the rx descs ring
        self.rx_regs.rdbal.write(rx_desc_phys_addr_lower);
        self.rx_regs.rdbah.write(rx_desc_phys_addr_higher);

        // write the length (in total bytes) of the rx descs array
        self.rx_regs.rdlen_write(num_desc); // should be 128 byte aligned, minimum 8 descriptors
        
        // Write the head index (the first receive descriptor)
        self.rx_regs.rdh_write(0);
        self.rx_regs.rdt_write(0);   
    }

    pub fn tx_init(&mut self, num_desc: NumDesc) -> TDHSet{
        // write the physical address of the tx descs array
        self.tx_regs.tdbal.write(self.descs_paddr.value() as u32); 
        self.tx_regs.tdbah.write((self.descs_paddr.value() >> 32) as u32); 

        // write the length (in total bytes) of the tx descs array
        let tdlen_set = self.tx_regs.tdlen_write(num_desc);               
        
        // write the head index and the tail index (both 0 initially because there are no tx requests yet)
        let tdh_set = self.tx_regs.tdh_write(0, tdlen_set);
        self.tx_regs.tdt_write(0);
        tdh_set
    }

    fn receive(&mut self, packet_length: &mut u16) -> Option<&mut EthernetFrame> {
        // error!("rx: {}", self.processed_delimiter);

        let rx_metadata = self.descs[self.processed_delimiter as usize].other.read();
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

    fn transmit(&mut self, packet_length: u16) {
        // error!("tx: {} {} {}", self.processed_delimiter, self.tx_clean, self.flush_counter);

        let rs_bit = if (self.processed_delimiter as u64 & (IXGBE_AGENT_RECYCLE_PERIOD - 1)) == (IXGBE_AGENT_RECYCLE_PERIOD - 1) { 1 << (24 + 3) } else { 0 };
        self.descs[self.processed_delimiter as usize].other.write(packet_length as u64 | rs_bit | (1 << (24 + 1)) | (1 << 24));
        self.processed_delimiter = (self.processed_delimiter + 1) & (IXGBE_RING_SIZE - 1);

        self.flush_counter += 1;
        if self.flush_counter == IXGBE_AGENT_FLUSH_PERIOD {
            self.tx_regs.tdt_write(self.processed_delimiter);
            self.flush_counter = 0;
        }

        if rs_bit != 0 {
            self.tx_clean();
            self.rx_regs.rdt_write((self.tx_clean - 1) & (IXGBE_RING_SIZE - 1));
            // error!("rs: {} {}", rs_bit, (self.tx_clean - 1) & (IXGBE_RING_SIZE - 1));
        }
        // error!("tx: {} {} {}", self.processed_delimiter, self.tx_clean, self.flush_counter);


    }

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

            if self.descs[cleanup_to].desc_done() {
                tx_clean = (cleanup_to + 1) % IXGBE_RING_SIZE as usize;
                self.descs[cleanup_to].other.write(0);
            } else {
                break;
            }
        }

        self.tx_clean = tx_clean as u16;
    }

    #[inline(always)]
    pub fn run(&mut self, src_addr: &[u8; 6], print: bool) -> usize {
        let mut packet_length = 0;
        let mut received = 0;

        for i in 0.. IXGBE_AGENT_FLUSH_PERIOD {
            if let Some(pkt) = self.receive(&mut packet_length) {
                pkt.src_addr = *src_addr;
                pkt.dest_addr = [0,0,0,0,0,1];
                self.transmit(packet_length);
                received += 1;
            } else {
                break;
            }
        }
        if print{
            error!("no packet received: {} {} {} {} {}", self.processed_delimiter, self.tx_clean, self.flush_counter, self.tx_regs.tdt_read(), self.rx_regs.rdt_read());
        }

        received
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