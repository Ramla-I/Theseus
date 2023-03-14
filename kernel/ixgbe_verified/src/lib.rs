//! An ixgbe driver for a 82599 10GbE Network Interface Card.
//! 
//! Currently we support basic send and receive, Receive Side Scaling (RSS), 5-tuple filters, and MSI interrupts. 
//! We also support language-level virtualization of the NIC so that applications can directly access their assigned transmit and receive queues.
//! When using virtualization, we disable RSS since we use 5-tuple filters to ensure packets are routed to the correct queues.
//! We also disable interrupts when using virtualization, since we do not yet have support for allowing applications to register their own interrupt handlers.

#![no_std]
#![allow(dead_code)] //  to suppress warnings for unused functions/methods
#![allow(unaligned_references)] // temporary, just to suppress unsafe packed borrows 
#![allow(incomplete_features)] // to allow adt_const_params without a warning
#![feature(adt_const_params)]
#![feature(array_zip)]
#![feature(rustc_private)]

extern crate prusti_contracts;
extern crate cfg_if;
extern crate alloc;

pub mod hal;
mod spec;
mod queue_registers;
pub mod vec_wrapper;
mod verified_functions;

cfg_if::cfg_if! {
if #[cfg(prusti)] {

extern crate core;

}
else {

#[macro_use] extern crate log;
#[macro_use] extern crate static_assertions;
extern crate spin;
extern crate irq_safety;
extern crate kernel_config;
extern crate memory;
extern crate pci; 
extern crate pit_clock;
extern crate bit_field;
extern crate volatile;
extern crate mpmc;
extern crate owning_ref;
extern crate rand;
extern crate hpet;
extern crate zerocopy;
extern crate mapped_pages_fragments;
extern crate packet_buffers;

pub mod rx_queue;
pub mod tx_queue;
pub mod allocator;

pub use hal::*;
use hal::regs::*;
use queue_registers::*;
use mapped_pages_fragments::MappedPagesFragments;
use rx_queue::{RxQueueE, RxQueueD, RxQueueL5, RxQueueRSS};
use tx_queue::{TxQueueE, TxQueueD};
use allocator::*;
use packet_buffers::*;
use vec_wrapper::VecWrapper;

use spin::Once;
use alloc::{
    vec::Vec,
    boxed::Box,
};
use irq_safety::MutexIrqSafe;
use memory::MappedPages;
use pci::{PciDevice, PciConfigSpaceAccessMechanism, PciLocation, BAR, PciBaseAddr};
use owning_ref::BoxRefMut;
use bit_field::BitField;
use hpet::get_hpet;
use rand::{SeedableRng, RngCore};
use core::ops::{Deref};

/// Vendor ID for Intel
pub const INTEL_VEND:                   u16 = 0x8086;  

/// Device ID for the 82599ES ethernet controller, used to identify the device from the PCI space
/// (https://www.intel.com/content/www/us/en/products/sku/41282/intel-82599es-10-gigabit-ethernet-controller/specifications.html)
pub const INTEL_82599ES:                  u16 = 0x10FB;  

/// Device ID for the X520-DA2 network adapter, used to identify the device from the PCI space.
/// I'm not sure what makes this different from the `INTEL_82599ES` device, because its spec states it uses the 82599 controller as well.
/// (https://ark.intel.com/content/www/us/en/ark/products/39776/intel-ethernet-converged-network-adapter-x520da2.html)
pub const INTEL_X520_DA2:                  u16 = 0x154D;  


/*** Developer Parameters of the Intel 82599 NIC ***/

/// The number of receive queues that are enabled. 
/// Do NOT set this greater than 64 since the queues 65-128 don't seem to work, 
/// most likely because they need additional configuration.
pub const IXGBE_NUM_RX_QUEUES_ENABLED: u8      = 64;
/// The number of transmit queues that are enabled. 
/// Do NOT set this greater than 64 since the queues 65-128 don't seem to work, 
/// most likely because they need additional configuration.
pub const IXGBE_NUM_TX_QUEUES_ENABLED: u8      = 64;
/// All buffers are created with 2KiB so that the max ethernet frame can fit in one packet buffer
pub const DEFAULT_RX_BUFFER_SIZE_2KB: RxBufferSizeKiB   = RxBufferSizeKiB::Buffer2KiB;


/*** Functions to get access to the IXGBE NICs once they've been initialized ***/

/// All the 82599 NICs found in the PCI space are initialized and then stored here.
pub static IXGBE_NICS: Once<Vec<MutexIrqSafe<IxgbeNic>>> = Once::new();

/// Returns a reference to the IxgbeNic wrapped in a MutexIrqSafe, if it exists and has been initialized.
/// Currently we use the pci location of the device as identification since it should not change after initialization.
pub fn get_ixgbe_nic(id: PciLocation) -> Result<&'static MutexIrqSafe<IxgbeNic>, &'static str> {
    let nics = IXGBE_NICS.get().ok_or("Ixgbe NICs weren't initialized")?;
    nics.iter()
        .find( |nic| { nic.lock().dev_id == id } )
        .ok_or("Ixgbe NIC with this ID does not exist")
}

/// Returns a reference to the list of all initialized ixgbe NICs
pub fn get_ixgbe_nics_list() -> Option<&'static Vec<MutexIrqSafe<IxgbeNic>>> {
    IXGBE_NICS.get()
}



/// A struct representing an ixgbe network interface card.
pub struct IxgbeNic {
    /// Device ID of the NIC assigned by the device manager.
    dev_id: PciLocation,
    /// Type of Base Address Register 0,
    /// if it's memory mapped or I/O.
    bar_type: PciConfigSpaceAccessMechanism,
    /// MMIO Base Address     
    mem_base: PciBaseAddr,
    /// The actual MAC address burnt into the hardware  
    mac_hardware: [u8;6],       
    /// Memory-mapped control registers
    regs1: BoxRefMut<MappedPages, IntelIxgbeRegisters1>,
    /// Memory-mapped control registers
    regs2: BoxRefMut<MappedPages, IntelIxgbeRegisters2>,
    /// Memory-mapped control registers
    regs3: BoxRefMut<MappedPages, IntelIxgbeRegisters3>,
    /// Memory-mapped control registers
    regs_mac: BoxRefMut<MappedPages, IntelIxgbeMacRegisters>,
    /// Array to store which L3/L4 5-tuple filters have been used.
    /// There are 128 such filters available.
    l34_5_tuple_filters: [bool; 128],
    /// The number of rx queues enabled
    num_rx_queues: u8,
    /// Vector of the enabled rx queues
    rx_queues: Vec<RxQueueE>,
    /// Vector of the disabled rx queues
    rx_queues_disabled: Vec<RxQueueD>,
    /// Vector of the rx queues being used in the L5 filters
    rx_queues_filters: Vec<RxQueueL5>,
    /// Vector of the rx queues being used for RSS
    rx_queues_rss: Vec<RxQueueRSS>,
    /// Registers for the queues >= 64, since they don't seem to work
    rx_registers_unusable: Vec<RxQueueRegisters>,
    /// The number of tx queues enabled
    num_tx_queues: u8,
    /// Vector of the enabled tx queues
    tx_queues: Vec<TxQueueE>,
    /// Vector of the disabled tx queues
    tx_queues_disabled: Vec<TxQueueD>,
    /// Registers for the disabled queues
    tx_registers_unusable: Vec<TxQueueRegisters>,
}

// Functions that setup the NIC struct and handle the sending and receiving of packets.
impl IxgbeNic {
    /// Store required values from the device's PCI config space, and initialize different features of the nic.
    /// 
    /// # Arguments
    /// * `ixgbe_pci_dev`: Contains the pci device information for this NIC.
    /// * `dev_id`: Device id as assigned by the device manager.
    ///     Currently this is just the pci location.
    /// * `link_speed`: The link speed of the ethernet connection which depends on the SFI module attached to the cable.
    ///     We do not access the PHY module for link information yet and currently only support 1 Gbps and 10 Gbps links.
    /// * `enable_virtualization`: True if language-level virtualization is enabled.
    ///     If this is true then interrupts and RSS need to be disabled. When the virtual NIC is created, these features 
    ///     should be enabled on a per-queue basis. We do not support that as of yet.
    /// * `interrupts`: A vector of packet reception interrupt handlers where the length of the vector is the number of
    ///     receive queues for which interrupts are enabled. We have currently tested for 16 receive queues.
    ///     The interrupt handler at index `i` is for receive queue `i`.
    ///     The number of handlers must be less than or equal to `IXGBE_NUM_RX_QUEUES_ENABLED`.
    ///     If interrupts are disabled, this should be set to None.
    /// * `enable_rss`: true if receive side scaling is enabled.
    /// * `rx_buffer_size_kbytes`: The size of receive buffers. 
    /// * `num_rx_descriptors`: The number of descriptors in each receive queue.
    /// * `num_tx_descriptors`: The number of descriptors in each transmit queue.
    pub fn init(
        ixgbe_pci_dev: &PciDevice,
        // enable_virtualization: bool,
        // enable_rss: bool,
        num_rx_descriptors: NumDesc,
        num_tx_descriptors: NumDesc
    ) -> Result<MutexIrqSafe<IxgbeNic>, &'static str> {
        // // Series of checks to determine if starting parameters are acceptable
        // if (enable_virtualization && interrupts.is_some()) || (enable_virtualization && enable_rss) {
        //     return Err("Cannot enable virtualization when interrupts or RSS are enabled");
        // }

        let bar_type = ixgbe_pci_dev.determine_pci_space();

        // If the base address is not memory mapped then exit
        if bar_type == PciConfigSpaceAccessMechanism::IoPort {
            error!("ixgbe::init(): BAR0 is of I/O type");
            return Err("ixgbe::init(): BAR0 is of I/O type")
        }

        // 16-byte aligned memory mapped base address
        let mem_base =  ixgbe_pci_dev.determine_pci_base_addr(BAR::BAR0)?;

        // set the bus mastering bit for this PciDevice, which allows it to use DMA
        ixgbe_pci_dev.pci_set_command_bus_master_bit();

        // map the IntelIxgbeRegisters structs to the address found from the pci space
        let (mut mapped_registers1, 
            mut mapped_registers2, 
            mut mapped_registers3, 
            mut mapped_registers_mac, 
            mut rx_mapped_registers, 
            mut tx_mapped_registers
        ) = Self::mapped_reg(&mem_base)?;

        // link initialization
        Self::start_link(&mut mapped_registers1, &mut mapped_registers2, &mut mapped_registers3, &mut mapped_registers_mac)?;

        // clear stats registers
        Self::clear_stats_internal(&mapped_registers2);

        // store the mac address of this device
        let mac_addr_hardware = Self::read_mac_address_from_nic(&mut mapped_registers_mac);

        // create the rx descriptor queues
        let rx_registers_unusable = rx_mapped_registers.split_off(IXGBE_NUM_RX_QUEUES_ENABLED as usize);
        let rx_queues = Self::rx_init(&mut mapped_registers1, &mut mapped_registers2, rx_mapped_registers, num_rx_descriptors)?;
        
         // create the tx descriptor queues
        let tx_registers_unusable = tx_mapped_registers.split_off(IXGBE_NUM_TX_QUEUES_ENABLED as usize);
        let tx_queues = Self::tx_init(&mut mapped_registers2, &mut mapped_registers_mac, tx_mapped_registers, num_tx_descriptors)?;
        
        // // enable Receive Side Scaling if required
        // if enable_rss {
        //     Self::enable_rss(&mut mapped_registers2, &mut mapped_registers3)?;
        // }

        // wait 10 seconds for the link to come up, as seen in other ixgbe drivers
        Self::wait_for_link(&mapped_registers2, 10_000_000);

        let ixgbe_nic = IxgbeNic {
            dev_id: ixgbe_pci_dev.location,
            bar_type: bar_type,
            mem_base: mem_base,
            mac_hardware: mac_addr_hardware,
            regs1: mapped_registers1,
            regs2: mapped_registers2,
            regs3: mapped_registers3,
            regs_mac: mapped_registers_mac,
            l34_5_tuple_filters: [false; NUM_L34_5_TUPLE_FILTERS],
            num_rx_queues: IXGBE_NUM_RX_QUEUES_ENABLED,
            rx_queues,
            rx_queues_disabled: Vec::new(),
            rx_queues_filters: Vec::new(),
            rx_queues_rss: Vec::new(),
            rx_registers_unusable,
            num_tx_queues: IXGBE_NUM_TX_QUEUES_ENABLED,
            tx_queues,
            tx_queues_disabled: Vec::new(),
            tx_registers_unusable
        };

        info!("Link is up with speed: {} Mb/s", ixgbe_nic.link_speed() as u32);

        Ok(MutexIrqSafe::new(ixgbe_nic))
    }

    /// Returns the device id of the PCI device.
    pub fn device_id(&self) -> PciLocation {
        self.dev_id
    }

    pub fn num_rx_queues_available(&self) -> usize {
        self.rx_queues.len()
    }

    pub fn num_tx_queues_available(&self) -> usize {
        self.tx_queues.len()
    }

    /// Returns the Rx queue located at this index. 
    /// This doesn't have to match the queue ID.
    pub fn get_rx_queue(&mut self, idx: usize) -> Result<&mut RxQueueE, &'static str> {
        if idx >= self.rx_queues.len() {
            return Err("Queue index is out of range");
        }

        Ok(&mut self.rx_queues[idx])
    }

    /// Returns the Tx queue located at this index. 
    /// This doesn't have to match the queue ID.
    pub fn get_tx_queue(&mut self, idx: usize) -> Result<&mut TxQueueE, &'static str> {
        if idx >= self.tx_queues.len() {
            return Err("Queue index is out of range");
        }

        Ok(&mut self.tx_queues[idx])
    }

    /// Returns the Tx queue located at this index. 
    /// This doesn't have to match the queue ID.
    pub fn get_queue_pair(&mut self, rq_idx: usize, tq_idx: usize) -> Result<(&mut RxQueueE, &mut TxQueueE), &'static str> {
        if tq_idx >= self.tx_queues.len() || rq_idx >= self.rx_queues.len() {
            return Err("Queue index is out of range");
        }

        Ok((&mut self.rx_queues[rq_idx], &mut self.tx_queues[tq_idx]))
    }

    pub fn tx_batch(&mut self, qid: usize, batch_size: usize,  buffers: &mut Vec<PacketBufferS>, used_buffers: &mut Vec<PacketBufferS>) -> Result<usize, &'static str> {
        if qid >= self.tx_queues.len() {
            return Err("Queue index is out of range");
        }

        self.tx_queues[qid].tx_batch(batch_size, buffers, used_buffers)
    }

    pub fn rx_batch(&mut self, qid: usize, buffers: &mut VecWrapper<PacketBufferS>, batch_size: usize, pool: &mut VecWrapper<PacketBufferS>) -> Result<u16, ()> {
        if qid >= self.rx_queues.len() {
            error!("Queue index is out of range");
            return Err(());
        }

        self.rx_queues[qid].rx_batch(buffers, batch_size, pool)
    }

    #[inline(always)]
    pub fn rx_batch_pseudo(&mut self, qid: usize, batch_size: usize) -> usize {
        // if qid >= self.rx_queues.len() {
        //     return Err("Queue index is out of range");
        // }

        self.rx_queues[qid].rx_batch_pseudo(batch_size)
    }

    pub fn tx_populate(&mut self, qid: usize, pool: &mut Vec<PacketBufferS>){
        // if qid >= self.rx_queues.len() {
        //     return Err("Queue index is out of range");
        // }

        self.tx_queues[qid].tx_populate(pool)
    }

    pub fn tx_batch_pseudo(&mut self, qid: usize, batch_size: usize) -> usize {
        // if qid >= self.rx_queues.len() {
        //     return Err("Queue index is out of range");
        // }

        self.tx_queues[qid].tx_batch_pseudo(batch_size)
    }

    /// Returns the memory-mapped control registers of the nic and the rx/tx queue registers.
    fn mapped_reg(
        mem_base: &PciBaseAddr
    ) -> Result<(
        BoxRefMut<MappedPages, IntelIxgbeRegisters1>, 
        BoxRefMut<MappedPages, IntelIxgbeRegisters2>, 
        BoxRefMut<MappedPages, IntelIxgbeRegisters3>, 
        BoxRefMut<MappedPages, IntelIxgbeMacRegisters>, 
        Vec<RxQueueRegisters>, 
        Vec<TxQueueRegisters>
    ), &'static str> {
        // We've divided the memory-mapped registers into multiple regions.
        // The size of each region is found from the data sheet, but it always lies on a page boundary.
        const GENERAL_REGISTERS_1_SIZE_BYTES:   usize = 4096;
        const RX_REGISTERS_SIZE_BYTES:          usize = 4096;
        const GENERAL_REGISTERS_2_SIZE_BYTES:   usize = 4 * 4096;
        const TX_REGISTERS_SIZE_BYTES:          usize = 2 * 4096;
        const MAC_REGISTERS_SIZE_BYTES:         usize = 5 * 4096;
        const GENERAL_REGISTERS_3_SIZE_BYTES:   usize = 18 * 4096;

        // Allocate memory for the registers, making sure each successive memory region begins where the previous region ended.
        let mut offset = *mem_base.deref();
        let nic_regs1_mapped_page = allocate_memory(offset, GENERAL_REGISTERS_1_SIZE_BYTES, NIC_MAPPING_FLAGS_NO_CACHE)?;

        offset += GENERAL_REGISTERS_1_SIZE_BYTES;
        let nic_rx_regs1_mapped_page = allocate_memory(offset, RX_REGISTERS_SIZE_BYTES, NIC_MAPPING_FLAGS_NO_CACHE)?;

        offset += RX_REGISTERS_SIZE_BYTES;
        let nic_regs2_mapped_page = allocate_memory(offset, GENERAL_REGISTERS_2_SIZE_BYTES, NIC_MAPPING_FLAGS_NO_CACHE)?;  

        offset += GENERAL_REGISTERS_2_SIZE_BYTES;
        let nic_tx_regs_mapped_page = allocate_memory(offset, TX_REGISTERS_SIZE_BYTES, NIC_MAPPING_FLAGS_NO_CACHE)?;

        offset += TX_REGISTERS_SIZE_BYTES;
        let nic_mac_regs_mapped_page = allocate_memory(offset, MAC_REGISTERS_SIZE_BYTES, NIC_MAPPING_FLAGS_NO_CACHE)?;

        offset += MAC_REGISTERS_SIZE_BYTES;
        let nic_rx_regs2_mapped_page = allocate_memory(offset, RX_REGISTERS_SIZE_BYTES, NIC_MAPPING_FLAGS_NO_CACHE)?;   

        offset += RX_REGISTERS_SIZE_BYTES;
        let nic_regs3_mapped_page = allocate_memory(offset, GENERAL_REGISTERS_3_SIZE_BYTES, NIC_MAPPING_FLAGS_NO_CACHE)?;

        // Map the memory as the register struct and tie the lifetime of the struct with its backing mapped pages
        let regs1 = BoxRefMut::new(Box::new(nic_regs1_mapped_page)).try_map_mut(|mp| mp.as_type_mut::<IntelIxgbeRegisters1>(0))?;
        let regs2 = BoxRefMut::new(Box::new(nic_regs2_mapped_page)).try_map_mut(|mp| mp.as_type_mut::<IntelIxgbeRegisters2>(0))?;
        let regs3 = BoxRefMut::new(Box::new(nic_regs3_mapped_page)).try_map_mut(|mp| mp.as_type_mut::<IntelIxgbeRegisters3>(0))?;
        let mac_regs = BoxRefMut::new(Box::new(nic_mac_regs_mapped_page)).try_map_mut(|mp| mp.as_type_mut::<IntelIxgbeMacRegisters>(0))?;
        
        // Divide the pages of the Rx queue registers into multiple 64B regions
        let mut regs_rx = Self::mapped_regs_from_rx_memory(MappedPagesFragments::new(nic_rx_regs1_mapped_page))?;
        regs_rx.append(&mut Self::mapped_regs_from_rx_memory(MappedPagesFragments::new(nic_rx_regs2_mapped_page))?);
        
        // Divide the pages of the Tx queue registers into multiple 64B regions
        let regs_tx = Self::mapped_regs_from_tx_memory(MappedPagesFragments::new(nic_tx_regs_mapped_page))?;
            
        Ok((regs1, regs2, regs3, mac_regs, regs_rx, regs_tx))
    }

    /// Split the pages where rx queue registers are mapped into multiple smaller memory regions.
    /// One region contains all the registers for a single queue.
    fn mapped_regs_from_rx_memory(mut mp: MappedPagesFragments) -> Result<Vec<RxQueueRegisters>, &'static str> {
        const QUEUES_IN_MP: usize = 64;

        // We share the backing mapped pages among all the queue registers
        let mut pointers_to_queues = Vec::with_capacity(QUEUES_IN_MP);

        for i in 0..QUEUES_IN_MP {
            pointers_to_queues.push(
                RxQueueRegisters::new(i, &mut mp)?
            );
        }
        Ok(pointers_to_queues)
    }

    /// Split the pages where tx queue registers are mapped into multiple smaller memory regions.
    /// One region contains all the registers for a single queue.
    fn mapped_regs_from_tx_memory(mut mp: MappedPagesFragments) -> Result<Vec<TxQueueRegisters>, &'static str> {
        const QUEUES_IN_MP: usize = 128;

        // We share the backing mapped pages among all the queue registers
        let mut pointers_to_queues = Vec::with_capacity(QUEUES_IN_MP);

        for i in 0..QUEUES_IN_MP {
            pointers_to_queues.push(
                TxQueueRegisters::new(i, &mut mp)?
            );
        }
        Ok(pointers_to_queues)
    }

    /// Reads the actual MAC address burned into the NIC hardware.
    fn read_mac_address_from_nic(regs: &IntelIxgbeMacRegisters) -> [u8; 6] {
        let mac_32_low = regs.ral.read();
        let mac_32_high = regs.rah.read();

        let mut mac_addr = [0; 6]; 
        mac_addr[0] =  mac_32_low as u8;
        mac_addr[1] = (mac_32_low >> 8) as u8;
        mac_addr[2] = (mac_32_low >> 16) as u8;
        mac_addr[3] = (mac_32_low >> 24) as u8;
        mac_addr[4] =  mac_32_high as u8;
        mac_addr[5] = (mac_32_high >> 8) as u8;

        debug!("Ixgbe: read hardware MAC address: {:02x?}", mac_addr);
        mac_addr
    }   

    /// Software reset of NIC to get it running.
    fn start_link (
        regs1: &mut IntelIxgbeRegisters1, 
        regs2: &mut IntelIxgbeRegisters2, 
        regs3: &mut IntelIxgbeRegisters3, 
        regs_mac: &mut IntelIxgbeMacRegisters,
    ) -> Result<(), &'static str> {
        //disable interrupts: write to EIMC registers, 1 in b30-b0, b31 is reserved
        regs1.eimc_disable_interrupts();

        // master disable algorithm (sec 5.2.5.3.2)
        // global reset = sw reset + link reset 
        regs1.ctrl_reset();

        //wait 10 ms
        let wait_time = 10_000;
        let _ =pit_clock::pit_wait(wait_time);

        //disable flow control.. write 0 TO FCTTV, FCRTL, FCRTH, FCRTV and FCCFG
        for fcttv in regs2.fcttv.iter_mut() {
            fcttv.write(0);
        }

        regs2.fcrtl_clear();
        regs2.fcrth_clear();
        regs2.fcrtv_clear();
        regs2.fccfg_clear();

        //disable interrupts
        regs1.eimc_disable_interrupts();

        //wait for eeprom auto read completion
        while !regs3.eec_auto_read(){}

        //read MAC address
        debug!("Ixgbe: MAC address low: {:#X}", regs_mac.ral.read());
        debug!("Ixgbe: MAC address high: {:#X}", regs_mac.rah.read() & 0xFFFF);

        //wait for dma initialization done (RDRXCTL.DMAIDONE)
        // TODO: can move to a function
        let mut val = regs2.rdrxctl_read();
        let dmaidone_bit = 1 << 3;
        while val & dmaidone_bit != dmaidone_bit {
            val = regs2.rdrxctl_read();
        }

        // debug!("STATUS: {:#X}", regs1.status.read()); 
        // debug!("CTRL: {:#X}", regs1.ctrl.read());
        // debug!("LINKS: {:#X}", regs2.links.read()); //b7 and b30 should be 1 for link up 
        // debug!("AUTOC: {:#X}", regs2.autoc.read()); 
        // debug!("AUTOC2: {:#X}", regs2.autoc2.read()); 

        Ok(())
    }

    /// Returns value of (links, links2) registers
    pub fn link_status(&self) -> (u32, u32) {
        (self.regs2.links.read(), self.regs2.links2.read())
    }

    /// Returns link speed in Mb/s
    pub fn link_speed(&self) -> LinkSpeedMbps {
        let speed = self.regs2.links.read() & LINKS_SPEED_MASK; 
        LinkSpeedMbps::from_links_register_value(speed)
    }

    /// Wait for link to be up for upto 10 seconds.
    fn wait_for_link(regs2: &IntelIxgbeRegisters2, total_wait_time_in_us: u32) {
        // wait 10 ms between tries
        let wait_time = 10_000;
        // wait for a total of 10 s
        let total_tries = total_wait_time_in_us / wait_time;
        let mut tries = 0;

        while (regs2.links.read() & LINKS_SPEED_MASK == 0) && (tries < total_tries) {
            let _ = pit_clock::pit_wait(wait_time);
            tries += 1;
        }
    }

    /// Clear the statistic registers by reading from them.
    fn clear_stats_internal(regs: &IntelIxgbeRegisters2) {
        regs.gprc.read();
        regs.gptc.read();
        regs.gorcl.read();
        regs.gorch.read();
        regs.gotcl.read();
        regs.gotch.read();
    }

    /// Clear the statistic registers by reading from them.
    pub fn clear_stats(&self) {
        Self::clear_stats_internal(&self.regs2);
    }

    /// Returns the Rx and Tx statistics for good packets.
    /// A good packet is one that is >= 64 bytes including ethernet header and CRC
    pub fn get_stats(&self, stats: &mut IxgbeStats) {
        let rx_bytes =  ((self.regs2.gorch.read() as u64 & 0xF) << 32) | self.regs2.gorcl.read() as u64;
        let tx_bytes =  ((self.regs2.gotch.read() as u64 & 0xF) << 32) | self.regs2.gotcl.read() as u64;

        stats.rx_bytes = rx_bytes;
        stats.tx_bytes = tx_bytes;
        stats.rx_packets = self.regs2.gprc.read();
        stats.tx_packets = self.regs2.gptc.read();
    }

    /// Initializes the array of receive descriptors and their corresponding receive buffers,
    /// and returns a tuple including both of them for all rx queues in use.
    /// Also enables receive functionality for the NIC.
    fn rx_init(
        regs1: &mut IntelIxgbeRegisters1, 
        regs: &mut IntelIxgbeRegisters2, 
        rx_regs: Vec<RxQueueRegisters>,
        num_rx_descs: NumDesc
    ) -> Result<Vec<RxQueueE>, &'static str> {

        Self::disable_rx_function(regs);
        // program RXPBSIZE according to DCB and virtualization modes (both off)
        // regs.rxpbsize_set_buffer_size(0, RXPBSIZE_128KB)?;
        for i in 1..8 {
            regs.rxpbsize_set_buffer_size(i, 0)?;
        }
        //CRC offloading
        regs.hlreg0_crc_strip();
        regs.rdrxctl_crc_strip();
        
        // Clear bits
        regs.rdrxctl.write(regs.rdrxctl.read() & !RDRXCTL_RSCFRSTSIZE);
        
        let mut rx_all_queues = Vec::new();

        for rxq_reg in rx_regs {      
            let mut rxq = RxQueueE::new(rxq_reg, num_rx_descs, None)?;

            // set the size of the packet buffers and the descriptor format used
            let mut val = rxq.regs.srrctl_read();
            // val.set_bits(0..4, DEFAULT_RX_BUFFER_SIZE_2KB as u32);
            // val.set_bits(8..13, BSIZEHEADER_0B);
            val.set_bits(25..27, DESCTYPE_ADV_1BUFFER);
            val = val | DROP_ENABLE;
            rxq.regs.srrctl_write(val)?;

            // enable the rx queue
            rxq.regs.rxdctl_rxq_enable();

            // make sure queue is enabled
            while rxq.regs.rxdctl_read() & RX_Q_ENABLE == 0 {}
        
            // set bit 12 to 0
            let val = rxq.regs.dca_rxctrl_read();
            rxq.regs.dca_rxctrl_write(val & !DCA_RXCTRL_CLEAR_BIT_12)?;

            // Write the tail index.
            // Note that the 82599 datasheet (section 8.2.3.8.5) states that we should set the RDT (tail index) to the index *beyond* the last receive descriptor, 
            // but we set it to the last receive descriptor for the same reason as the e1000 driver
            rxq.regs.rdt.write(num_rx_descs as u32 - 1);
            
            rx_all_queues.push(rxq);
        }
        
        Self::enable_rx_function(regs1,regs)?;
        Ok(rx_all_queues)
    }

    /// disable receive functionality
    fn disable_rx_function(regs: &mut IntelIxgbeRegisters2) {        
        regs.rxctrl_rx_disable();
    }

    /// enable receive functionality
    fn enable_rx_function(regs1: &mut IntelIxgbeRegisters1,regs: &mut IntelIxgbeRegisters2) -> Result<(), &'static str> {
        // set rx parameters of which type of packets are accepted by the nic
        // right now we allow the nic to receive all types of packets, even incorrectly formed ones
        regs.fctrl_write(STORE_BAD_PACKETS | MULTICAST_PROMISCUOUS_ENABLE | UNICAST_PROMISCUOUS_ENABLE | BROADCAST_ACCEPT_MODE)?; 

        regs1.ctrl_ext_no_snoop_disable();

        // enable receive functionality
        regs.rxctrl_rx_enable(); 

        Ok(())
    }


    /// Initialize the array of transmit descriptors for all queues and returns them.
    /// Also enables transmit functionality for the NIC.
    fn tx_init(
        regs: &mut IntelIxgbeRegisters2, 
        regs_mac: &mut IntelIxgbeMacRegisters, 
        tx_regs: Vec<TxQueueRegisters>,
        num_tx_descs: NumDesc
    ) -> Result<Vec<TxQueueE>, &'static str> {
        // disable transmission
        regs.dmatxctl_disable_tx();

        // CRC offload and small packet padding enable
        regs.hlreg0_crc_en();
        regs.hlreg0_tx_pad_en();

        // Set RTTFCS.ARBDIS to 1
        regs.rttdcs_set_arbdis();


        // program DTXMXSZRQ and TXPBSIZE according to DCB and virtualization modes (both off)
        regs_mac.txpbsize_write(0, TXPBSIZE_160KB)?;
        for i in 1..8 {
            regs_mac.txpbsize_write(i, 0)?;
        }
        regs_mac.dtxmxszrq_write(DTXMXSZRQ_MAX_BYTES)?; 

        // Clear RTTFCS.ARBDIS
        regs.rttdcs_clear_arbdis();

        let mut tx_all_queues = Vec::new();

        // enable transmit operation, only have to do this for the first queue
        regs.dmatxctl_enable_tx();

        for txq_reg in tx_regs {
            let mut txq = TxQueueE::new(txq_reg, num_tx_descs, None)?;
        
            // Set descriptor thresholds
            // If we enable this then we need to change the packet send function to stop polling
            // for a descriptor done on every packet sent
            txq.regs.txdctl.write(TXDCTL_PTHRESH | TXDCTL_HTHRESH | TXDCTL_WTHRESH); 

            //enable tx queue
            txq.regs.txdctl_txq_enable(); 

            //make sure queue is enabled
            while txq.regs.txdctl_read() & TX_Q_ENABLE == 0 {} 

            tx_all_queues.push(txq);
        }
        Ok(tx_all_queues)
    }  

    /// 4.2.1.6
    fn software_reset(&mut self) {
        self.master_disable();

        // write to Device Reset bit CTRL.RST

        //wait 1 ms to check if this bit is cleared
        
        // now initialize the NIC as you would do after power up, except the PCI space is th same so can keep all device regs
    }

    /// 5.2.5.3.2
    fn master_disable(&mut self) {
        // disable all rx queues (4.6.7.1.2)

        // sets the PCIe Master Disable bit

        // wait for PCIe MAster Disable bit to clear?

        // poll PCIe Master Enable Status bit until it's cleared

        // If the driver times out at this point then check Transaction Pending bit and send another SW rest

        // flush transmit data path

        // now a sw reset is safe
    }

    pub(crate) fn extract_rss_queues(reta: &[[QueueID; 4]; 32], enabled_queues: &mut Vec<RxQueueE>) -> Result<Vec<RxQueueE>, &'static str> {
        let mut used_queue_ids = Vec::new();
        for reg in reta {
            for i in 0..4 {
                if !used_queue_ids.contains(&reg[i]) {
                    used_queue_ids.push(reg[i]);
                }
            }
        }
    
        let mut queues_for_rss = Vec::with_capacity(used_queue_ids.len());
        for qid in used_queue_ids {
            let index = enabled_queues.iter().position(|x| x.id == qid as u8)
                .ok_or("Required Queue for RSS is not in the enabled list")?;
            queues_for_rss.push(enabled_queues.remove(index));
        }
    
        Ok(queues_for_rss)
    }

    pub fn enable_rss(&mut self, reta: [[QueueID; 4]; 32]) -> Result<(), &'static str> {
        // software reset
        self.software_reset();
        // remove all queues that are in RETA to a separate vec
        let queues_in_reta = Self::extract_rss_queues(&reta, &mut self.rx_queues)?;
        // call enable_rss
        let rss_queues = Self::enable_rss_internal(&mut self.regs2, &mut self.regs3, reta, queues_in_reta)?;
        // store RSS queues in the NIC
        self.rx_queues_rss = rss_queues;

        Ok(())
    }

    pub fn disable_rss(&mut self) {
        // software reset       
        self.software_reset();
    }

    /// Enable multiple receive queues with RSS.
    /// Part of queue initialization is done in the rx_init function.
    pub fn enable_rss_internal(
        regs2: &mut IntelIxgbeRegisters2, 
        regs3: &mut IntelIxgbeRegisters3,
        redirection_table: [[QueueID; 4];32],
        queues: Vec<RxQueueE>
    ) -> Result<Vec<RxQueueRSS>, &'static str> {
        // enable RSS writeback in the header field of the receive descriptor
        regs2.rxcsum_enable_rss_writeback();
        
        // enable RSS and set fields that will be used by hash function
        // right now we're using the udp port and ipv4 address.
        regs3.mrqc.write(MRQC_MRQE_RSS | MRQC_UDPIPV4 ); 

        //set the random keys for the hash function
        let seed = get_hpet().as_ref().ok_or("couldn't get HPET timer")?.get_counter();
        let mut rng = rand::rngs::SmallRng::seed_from_u64(seed);
        for rssrk in regs3.rssrk.iter_mut() {
            rssrk.write(rng.next_u32());
        }

        // Initialize the RSS redirection table
        // each reta register has 4 redirection entries
        // since mapping to queues is random and based on a hash, we randomly assign 1 queue to each reta register
        // let mut qid = 0;
        // for reta in regs3.reta.iter_mut() {
        //     //set 4 entries to the same queue number
        //     let val = qid << RETA_ENTRY_0_OFFSET | qid << RETA_ENTRY_1_OFFSET | qid << RETA_ENTRY_2_OFFSET | qid << RETA_ENTRY_3_OFFSET;
        //     reta.write(val);

        //     // next 4 entries will be assigned to the next queue
        //     qid = (qid + 1) % IXGBE_NUM_RX_QUEUES_ENABLED as u32;
        // }

        for (idx, reta) in regs3.reta.iter_mut().enumerate() {
            let queue_ids = &redirection_table[idx];
            //set 4 entries to the same queue number
            let val = (queue_ids[0] as u32) << RETA_ENTRY_0_OFFSET 
                | (queue_ids[1] as u32) << RETA_ENTRY_1_OFFSET 
                | (queue_ids[2] as u32) << RETA_ENTRY_2_OFFSET 
                | (queue_ids[3] as u32) << RETA_ENTRY_3_OFFSET;
            reta.write(val);
        }

        let mut rss_queues = Vec::with_capacity(queues.len());
        for queue in queues {
            rss_queues.push(queue.rss())
        }
        Ok(rss_queues)
    }

    fn find_enabled_queue_with_id(&mut self, qid: QueueID) -> Option<RxQueueE> {
        self.rx_queues.iter().position(|x| x.id == qid as u8)
            .and_then(|idx| Some(self.rx_queues.remove(idx)))
    }

    /// Sets the L3/L4 5-tuple filter which can do an exact match of the packet's header with the filter and send to chosen rx queue (7.1.2.5).
    /// There are up to 128 such filters. If more are needed, will have to enable Flow Director filters.
    /// 
    /// # Argument
    /// * `source_ip`: ipv4 source address
    /// * `dest_ip`: ipv4 destination address
    /// * `source_port`: TCP/UDP/SCTP source port
    /// * `dest_port`: TCP/UDP/SCTP destination port
    /// * `protocol`: IP L4 protocol
    /// * `priority`: priority relative to other filters, can be from 0 (lowest) to 7 (highest)
    /// * `qid`: number of the queue to forward packet to
    pub fn set_5_tuple_filter(
        &mut self, 
        source_ip: Option<[u8;4]>, 
        dest_ip: Option<[u8;4]>, 
        source_port: Option<u16>, 
        dest_port: Option<u16>, 
        protocol: Option<FilterProtocol>, 
        priority: L5FilterPriority, 
        qid: QueueID
    ) -> Result<(), &'static str> {
        let queue = self.find_enabled_queue_with_id(qid).ok_or("Requested queue is not an enabled state")?;
        let l5_queue = self.set_5_tuple_filter_internal(source_ip, dest_ip, source_port, dest_port, protocol, priority, queue)?;
        self.rx_queues_filters.push(l5_queue);
        Ok(())
    }

    /// Sets the L3/L4 5-tuple filter which can do an exact match of the packet's header with the filter and send to chosen rx queue (7.1.2.5).
    /// There are up to 128 such filters. If more are needed, will have to enable Flow Director filters.
    /// 
    /// # Argument
    /// * `source_ip`: ipv4 source address
    /// * `dest_ip`: ipv4 destination address
    /// * `source_port`: TCP/UDP/SCTP source port
    /// * `dest_port`: TCP/UDP/SCTP destination port
    /// * `protocol`: IP L4 protocol
    /// * `priority`: priority relative to other filters, can be from 0 (lowest) to 7 (highest)
    /// * `queue`: Rx Queue to forward the packet to
    pub fn set_5_tuple_filter_internal(
        &mut self, 
        source_ip: Option<[u8;4]>, 
        dest_ip: Option<[u8;4]>, 
        source_port: Option<u16>, 
        dest_port: Option<u16>, 
        protocol: Option<FilterProtocol>, 
        priority: L5FilterPriority, 
        queue: RxQueueE
    ) -> Result<RxQueueL5, &'static str> {

        if source_ip.is_none() && dest_ip.is_none() && source_port.is_none() && dest_port.is_none() && protocol.is_none() {
            return Err("Must set one of the five filter options");
        }

        let enabled_filters = &mut self.l34_5_tuple_filters;

        // find a free filter
        let filter_num = enabled_filters.iter().position(|&r| r == false).ok_or("Ixgbe: No filter available")?;

        // start off with the filter mask set for all the filters, and clear bits for filters that are enabled
        // bits 29:25 are set to 1.
        let mut filter_mask = 0x3E000000;

        // IP addresses are written to the registers in big endian form (LSB is first on wire)
        // set the source ip address for the filter
        if let Some (addr) = source_ip {
            self.regs3.saqf[filter_num].write(((addr[3] as u32) << 24) | ((addr[2] as u32) << 16) | ((addr[1] as u32) << 8) | (addr[0] as u32));
            filter_mask = filter_mask & !FTQF_SOURCE_ADDRESS_MASK;
        };

        // set the destination ip address for the filter
        if let Some(addr) = dest_ip {
            self.regs3.daqf[filter_num].write(((addr[3] as u32) << 24) | ((addr[2] as u32) << 16) | ((addr[1] as u32) << 8) | (addr[0] as u32));
            filter_mask = filter_mask & !FTQF_DEST_ADDRESS_MASK;
        };        

        // set the source port for the filter    
        if let Some(port) = source_port {
            self.regs3.sdpqf[filter_num].write((port as u32) << SPDQF_SOURCE_SHIFT);
            filter_mask = filter_mask & !FTQF_SOURCE_PORT_MASK;
        };   

        // set the destination port for the filter    
        if let Some(port) = dest_port {
            let port_val = self.regs3.sdpqf[filter_num].read();
            self.regs3.sdpqf[filter_num].write(port_val | (port as u32) << SPDQF_DEST_SHIFT);
            filter_mask = filter_mask & !FTQF_DEST_PORT_MASK;
        };

        // set the filter protocol
        let mut filter_protocol = FilterProtocol::Other;
        if let Some(protocol) = protocol {
            filter_protocol = protocol;
            filter_mask = filter_mask & !FTQF_PROTOCOL_MASK;
        };

        // write the parameters of the filter
        let filter_priority = (priority as u32 & FTQF_PRIORITY) << FTQF_PRIORITY_SHIFT;
        self.regs3.ftqf[filter_num].write(filter_protocol as u32 | filter_priority | filter_mask | FTQF_Q_ENABLE);

        //set the rx queue that the packets for this filter should be sent to
        self.regs3.l34timir[filter_num].write(L34TIMIR_BYPASS_SIZE_CHECK | L34TIMIR_RESERVED | ((queue.id as u32) << L34TIMIR_RX_Q_SHIFT));

        //mark the filter as used
        enabled_filters[filter_num] = true;
        Ok(queue.l5_filter(filter_num as u8))
    }

    /// Disables the the L3/L4 5-tuple filter for the given filter number
    /// but keeps the values stored in the filter registers.
    fn disable_5_tuple_filter(&mut self, qid: QueueID) -> Result<(), &'static str> {
        let queue = self.rx_queues_filters.iter().position(|x| x.id == qid as u8)
            .and_then(|idx| Some(self.rx_queues_filters.remove(idx)))
            .ok_or("requested qid is not in the list of L5 queues")?;
        let enabled_queue = self.disable_5_tuple_filter_internal(queue)?; 
        self.rx_queues.push(enabled_queue);
        Ok(())
    }

    /// Disables the the L3/L4 5-tuple filter for the given filter number
    /// but keeps the values stored in the filter registers.
    fn disable_5_tuple_filter_internal(&mut self, queue: RxQueueL5) -> Result<RxQueueE, &'static str> {
        let filter_num = queue.filter_num.ok_or("filter num not availble for an L5 queue, this is a logical error!")?;
        // disables filter by setting enable bit to 0
        let val = self.regs3.ftqf[filter_num as usize].read();
        self.regs3.ftqf[filter_num as usize].write(val | !FTQF_Q_ENABLE);

        // sets the record in the nic struct to false
        self.l34_5_tuple_filters[filter_num as usize] = false;
        Ok(queue.enable())
    }


    // /// Removes `num_queues` Rx queues from this "physical" NIC device and gives up ownership of them.
    // /// This function is used when creating a virtual NIC that will own the returned queues.
    // fn take_rx_queues_from_physical_nic(
    //     &mut self, 
    //     num_queues: usize
    // ) -> Result<Vec<RxQueue<RxQueueRegisters, AdvancedRxDescriptor>>, &'static str> {
    //     // We always ensure queue 0 is kept for the physical NIC
    //     if num_queues >= self.rx_queues.len()  {
    //         return Err("Not enough rx queues for the NIC to remove any");
    //     }
    //     let start_remove_index = self.rx_queues.len() - num_queues;
    //     let queues = self.rx_queues.drain(start_remove_index..).collect(); 
    //     Ok(queues)
    // }

    // /// Removes `num_queues` Tx queues from this "physical" NIC device and gives up ownership of them.
    // /// This function is when creating a virtual NIC that will own the returned queues.
    // fn take_tx_queues_from_physical_nic(
    //     &mut self, 
    //     num_queues: usize
    // ) -> Result<Vec<TxQueue<TxQueueRegisters, AdvancedTxDescriptor>>, &'static str> {
    //     // We always ensure queue 0 is kept for the physical NIC
    //     if num_queues >= self.tx_queues.len()  {
    //         return Err("Not enough tx queues for the NIC to remove any");
    //     }
    //     let start_remove_index = self.tx_queues.len() - num_queues;
    //     let queues = self.tx_queues.drain(start_remove_index..).collect(); 
    //     Ok(queues)
    // }
}



#[derive(Default, Debug)]
pub struct IxgbeStats{
    pub rx_bytes: u64,
    pub tx_bytes: u64,
    pub rx_packets: u32,
    pub tx_packets: u32,
}

/// The list of valid Queues that can be used in the 82599 (0,64]
#[repr(u8)]
#[derive(PartialEq, Eq, Clone, Copy)]
pub enum QueueID {
    Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,
    Q10,Q11,Q12,Q13,Q14,Q15,Q16,Q17,Q18,Q19,
    Q20,Q21,Q22,Q23,Q24,Q25,Q26,Q27,Q28,Q29,
    Q30,Q31,Q32,Q33,Q34,Q35,Q36,Q37,Q38,Q39,
    Q40,Q41,Q42,Q43,Q44,Q45,Q46,Q47,Q48,Q49,
    Q50,Q51,Q52,Q53,Q54,Q55,Q56,Q57,Q58,Q59,
    Q60,Q61,Q62,Q63,Q64
}

/// The list of valid filter priority levels that can be used for the L5 filters. They range from (0,7).
#[repr(u8)]
#[derive(PartialEq, Eq, Clone, Copy)]
pub enum L5FilterPriority {
    P0,
    P1,
    P2,
    P3,
    P4,
    P5,
    P6,
    P7
}

// /// A helper function to poll the nic receive queues (only for testing purposes).
// pub fn rx_poll_mq(qid: usize, nic_id: PciLocation) -> Result<ReceivedFrame, &'static str> {
//     let nic_ref = get_ixgbe_nic(nic_id)?;
//     let mut nic = nic_ref.lock();      
//     nic.rx_queues[qid as usize].poll_queue_and_store_received_packets()?;
//     let frame = nic.rx_queues[qid as usize].return_frame().ok_or("no frame")?;
//     Ok(frame)
// }

// /// A helper function to send a test packet on a nic transmit queue (only for testing purposes).
// pub fn tx_send_mq(qid: usize, nic_id: PciLocation, packet: Option<TransmitBuffer>) -> Result<(), &'static str> {
//     let packet = packet.unwrap_or(test_packets::create_dhcp_test_packet()?);
//     let nic_ref = get_ixgbe_nic(nic_id)?;
//     let mut nic = nic_ref.lock();  

//     nic.tx_queues[qid].send_on_queue(packet);
//     Ok(())
// }

}
}