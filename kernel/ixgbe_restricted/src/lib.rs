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
#![feature(ptr_internals)]

#[macro_use] extern crate prusti_contracts;
extern crate cfg_if;
extern crate alloc;

pub mod hal;
mod queue_registers;
pub mod queue;
pub use hal::*;

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
extern crate num_enum;
#[macro_use] extern crate bitflags;

pub mod allocator;

use hal::regs::*;
use queue_registers::*;
use mapped_pages_fragments::MappedPagesFragments;
use queue::*;
use allocator::*;

use spin::Once;
use alloc::{
    vec::Vec,
    boxed::Box,
};
use irq_safety::MutexIrqSafe;
use memory::{MappedPages, BorrowedMappedPages, Mutable};
use pci::{PciDevice, PciConfigSpaceAccessMechanism, PciLocation, BAR, PciBaseAddr, PciMemSize};
use owning_ref::BoxRefMut;
use core::num;
// use bit_field::BitField;
// use hpet::get_hpet;
// use rand::{SeedableRng, RngCore};
use core::ops::{Deref};
use core::convert::{TryFrom};

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
pub const IXGBE_NUM_RX_QUEUES_ENABLED: u8      = 1;
/// The number of transmit queues that are enabled. 
/// Do NOT set this greater than 64 since the queues 65-128 don't seem to work, 
/// most likely because they need additional configuration.
pub const IXGBE_NUM_TX_QUEUES_ENABLED: u8      = 1;
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
    regs1: BorrowedMappedPages<IntelIxgbeRegisters1, Mutable>,
    /// Memory-mapped control registers
    regs2: BorrowedMappedPages<IntelIxgbeRegisters2, Mutable>,
    /// Memory-mapped control registers
    regs3: BorrowedMappedPages<IntelIxgbeRegisters3, Mutable>,
    /// Memory-mapped control registers
    regs_mac: BorrowedMappedPages<IntelIxgbeMacRegisters, Mutable>,
    pub queue: Queue,
    /// Registers for the disabled queues
    rx_registers_unusable: Vec<RxQueueRegisters>,
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
        let _mem_size = ixgbe_pci_dev.determine_pci_mem_size(BAR::BAR0);

        // set the bus mastering bit for this PciDevice, which allows it to use DMA
        ixgbe_pci_dev.pci_set_command_bus_master_bit();
        ixgbe_pci_dev.pci_set_command_memory_space_bit();

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
        // let rx_queues = Self::rx_init(&mut mapped_registers1, &mut mapped_registers2, rx_mapped_registers, num_rx_descriptors)?;
        
         // create the tx descriptor queues
        let tx_registers_unusable = tx_mapped_registers.split_off(IXGBE_NUM_TX_QUEUES_ENABLED as usize);
        // let tx_queues = Self::tx_init(&mut mapped_registers2, &mut mapped_registers_mac, tx_mapped_registers, num_tx_descriptors)?;
        
        let num_descs = NumDesc::Descs1k;
        let mut queue = Queue::new(num_descs, tx_mapped_registers.pop().unwrap(), rx_mapped_registers.pop().unwrap())?;
        Self::rx_init(&mut mapped_registers1, &mut mapped_registers2, num_descs, &mut queue)?;
        Self::tx_init(&mut mapped_registers2, &mut mapped_registers_mac, num_descs, &mut queue)?;

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
            queue,
            rx_registers_unusable,
            tx_registers_unusable
        };

        info!("Link is up with speed: {} Mb/s", ixgbe_nic.link_speed() as u32);

        Ok(MutexIrqSafe::new(ixgbe_nic))
    }

    /// Returns the device id of the PCI device.
    pub fn device_id(&self) -> PciLocation {
        self.dev_id
    }


    /// Returns the memory-mapped control registers of the nic and the rx/tx queue registers.
    fn mapped_reg(
        mem_base: &PciBaseAddr
    ) -> Result<(
        BorrowedMappedPages<IntelIxgbeRegisters1, Mutable>, 
        BorrowedMappedPages<IntelIxgbeRegisters2, Mutable>, 
        BorrowedMappedPages<IntelIxgbeRegisters3, Mutable>, 
        BorrowedMappedPages<IntelIxgbeMacRegisters, Mutable>, 
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
        let regs1 = nic_regs1_mapped_page.into_borrowed_mut(0).map_err(|(_mp, err)| err)?;
        let regs2 = nic_regs2_mapped_page.into_borrowed_mut(0).map_err(|(_mp, err)| err)?;
        let regs3 = nic_regs3_mapped_page.into_borrowed_mut(0).map_err(|(_mp, err)| err)?;
        let mac_regs = nic_mac_regs_mapped_page.into_borrowed_mut(0).map_err(|(_mp, err)| err)?;
        
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
        let mac_16_high = regs.rah();

        let mut mac_addr = [0; 6]; 
        mac_addr[0] =  mac_32_low as u8;
        mac_addr[1] = (mac_32_low >> 8) as u8;
        mac_addr[2] = (mac_32_low >> 16) as u8;
        mac_addr[3] = (mac_32_low >> 24) as u8;
        mac_addr[4] =  mac_16_high as u8;
        mac_addr[5] = (mac_16_high >> 8) as u8;

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
        debug!("Ixgbe: MAC address high: {:#X}", regs_mac.rah());

        //wait for dma initialization done (RDRXCTL.DMAIDONE)
        while !regs2.rdrxctl_dma_init_done() {}

        // debug!("STATUS: {:#X}", regs1.status.read()); 
        // debug!("CTRL: {:#X}", regs1.ctrl.read());
        // debug!("LINKS: {:#X}", regs2.links.read()); //b7 and b30 should be 1 for link up 
        // debug!("AUTOC: {:#X}", regs2.autoc.read()); 
        // debug!("AUTOC2: {:#X}", regs2.autoc2.read()); 

        Ok(())
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

    pub fn fctrl_flags(&self) -> u32 {
        self.regs2.fctrl_read()
    }

    /// Initializes the array of receive descriptors and their corresponding receive buffers,
    /// and returns a tuple including both of them for all rx queues in use.
    /// Also enables receive functionality for the NIC.
    fn rx_init(
        regs1: &mut IntelIxgbeRegisters1, 
        regs: &mut IntelIxgbeRegisters2, 
        num_rx_descs: NumDesc,
        queue: &mut Queue,
    ) -> Result<(), &'static str> {

        let rxctrl_disabled = Self::disable_rx_function(regs);

        // program RXPBSIZE according to DCB and virtualization modes (both off)
        // regs.rxpbsize_set_buffer_size(0, RXPBSIZE_128KB)?;
        for i in 1..8 {
            regs.rxpbsize_reg1_7_set_buffer_size(
                RxPBReg::try_from(i).map_err(|_| "incorrect register ID for RxPBSize")?,
                RxPBSizeReg1_7::Size0KiB 
            );
        }
        //CRC offloading
        regs.crc_strip();
        
        // Clear bits
        regs.rdrxctl_clear_rsc_frst_size_bits();
        
        queue.rx_init(num_rx_descs);
        // set the size of the packet buffers(leave default value) and the descriptor format used
        queue.rx_regs.srrctl_write(DescType::Legacy, RxBufferSizeKiB::Buffer2KiB);
        queue.rx_regs.srrctl_drop_enable();
        // enable the rx queue
        queue.rx_regs.rxdctl_rxq_enable();
        // make sure queue is enabled
        while queue.rx_regs.rxdctl_read() & RX_Q_ENABLE == 0 {}
        // set bit 12 to 0
        queue.rx_regs.dca_rxctrl_clear_bit_12();

        // Write the tail index.
        // Note that the 82599 datasheet (section 8.2.3.8.5) states that we should set the RDT (tail index) to the index *beyond* the last receive descriptor, 
        // but we set it to the last receive descriptor for the same reason as the e1000 driver
        queue.rx_regs.rdt_write(num_rx_descs as u16 - 1);
        
        Self::enable_rx_function(regs1,regs, rxctrl_disabled)?;

        Ok(())
    }

    /// disable receive functionality
    fn disable_rx_function(regs: &mut IntelIxgbeRegisters2) -> RXCTRLDisabled {        
        regs.rxctrl_rx_disable()
    }

    /// enable receive functionality
    fn enable_rx_function(regs1: &mut IntelIxgbeRegisters1,regs: &mut IntelIxgbeRegisters2, rxctrl_disabled: RXCTRLDisabled) -> Result<(), &'static str> {
        // set rx parameters of which type of packets are accepted by the nic
        // right now we allow the nic to receive all types of packets, even incorrectly formed ones
        let fctrl_set = regs.fctrl_write(FilterCtrlFlags::STORE_BAD_PACKETS | FilterCtrlFlags::MULTICAST_PROMISCUOUS_ENABLE | FilterCtrlFlags::UNICAST_PROMISCUOUS_ENABLE | FilterCtrlFlags::BROADCAST_ACCEPT_MODE, rxctrl_disabled); 

        regs1.ctrl_ext_no_snoop_disable();

        // enable receive functionality
        regs.rxctrl_rx_enable(fctrl_set); 

        Ok(())
    }


    /// Initialize the array of transmit descriptors for all queues and returns them.
    /// Also enables transmit functionality for the NIC.
    fn tx_init(
        regs: &mut IntelIxgbeRegisters2, 
        regs_mac: &mut IntelIxgbeMacRegisters, 
        num_tx_descs: NumDesc,
        queue: &mut Queue
    ) -> Result<(), &'static str> {
        // disable transmission
        regs.dmatxctl_disable_tx();

        // CRC offload and small packet padding enable
        regs.hlreg0_crc_en();
        regs.hlreg0_tx_pad_en();

        // Set RTTFCS.ARBDIS to 1
        regs.rttdcs_set_arbdis();


        // program DTXMXSZRQ and TXPBSIZE according to DCB and virtualization modes (both off)
        regs_mac.txpbsize_write(
            TxPBReg::R0, 
            TxPBSize::Size160KiB
        );
        for i in 1..8 {
            regs_mac.txpbsize_write(
                TxPBReg::try_from(i).map_err(|_| "incorrect reg idx for TXPBSIZE")?, 
                TxPBSize::Size0KiB
            );
        }
        regs_mac.dtxmxszrq_allow_max_byte_requests(); 

        // Clear RTTFCS.ARBDIS
        regs.rttdcs_clear_arbdis();

        // enable transmit operation, only have to do this for the first queue
        regs.dmatxctl_enable_tx();

        // set buffer addresses
        for i in 0..num_tx_descs as usize {
            let packet_paddr = queue.buffers_paddr + (i * 2048);
            queue.descs[i].phys_addr.write(packet_paddr.value() as u64);
        }

        // Set descriptor thresholds
        // If we enable this then we need to change the packet send function to stop polling for a descriptor done on every packet sent
        
        // Tx descriptor pre-fetch threshold (value taken from DPDK)
        let pthresh = U7::B5 | U7::B2; // b100100 = 36 (DPDK), TInyNF uses 60
        // Tx descriptor host threshold (value taken from DPDK)
        let hthresh = HThresh::B3; // b1000 = 8 (DPDK), TinyNF uses 4  
        // Tx descriptor write-back threshold (value taken from DPDK)
        let wthresh = U7::zero(); // b100 = 4 
        
        let _rs_bit = queue.tx_regs.txdctl_write_wthresh(wthresh); 
        queue.tx_regs.txdctl_write_pthresh_hthresh(pthresh, hthresh); 
        
        let tdh_set = queue.tx_init(num_tx_descs);

        //enable tx queue
        queue.tx_regs.txdctl_txq_enable(tdh_set); 
        
        //make sure queue is enabled
        while queue.tx_regs.txdctl_read() & TX_Q_ENABLE == 0 {} 

        Ok(())
    }  


    pub fn mac_addr(&self) -> [u8; 6] {
        self.mac_hardware
    }

}


#[derive(Default, Debug)]
pub struct IxgbeStats{
    pub rx_bytes: u64,
    pub tx_bytes: u64,
    pub rx_packets: u32,
    pub tx_packets: u32,
}


#[derive(PartialEq, Eq, Clone, Copy)]
pub struct FilterParameters {
    pub source_ip: Option<[u8; 4]>,
    pub dest_ip: Option<[u8; 4]>,
    pub source_port: Option<u16>,
    pub dest_port: Option<u16>,
    pub protocol: Option<L5FilterProtocol>,
    pub priority: L5FilterPriority,
    pub qid: QueueID
}

impl FilterParameters {
    #[pure]
    pub fn parameters_equal(&self, other: &Self) -> bool {
        self.source_ip == other.source_ip &&
        self.dest_ip == other.dest_ip &&
        self.source_port == other.source_port &&
        self.dest_port == other.dest_port &&
        self.protocol == other.protocol &&
        self.priority == other.priority
    }
}

#[derive(PartialEq, Eq, Clone, Copy)]
pub enum FilterError {
    NoneAvailable,
    IdenticalFilter(usize)
}