//! An ixgbe driver for a 82599 10GbE Network Interface Card.
//! 
//! Currently we support basic send and receive, Receive Side Scaling (RSS), and 5-tuple filters. 

#![no_std]
#![allow(incomplete_features)] // to allow adt_const_params without a warning
#![feature(rustc_private)]
#![feature(ptr_internals)]
#![allow(dead_code)]
#![feature(adt_const_params)]
#![feature(type_changing_struct_update)]

extern crate alloc;

cfg_if::cfg_if! {
if #[cfg(prusti)] {
    mod hal;
    pub mod mempool;
    mod ethernet_frame;
    mod verified;
    mod spec;
} else {

mod hal;
mod queue_registers;
mod ethernet_frame;
pub mod mempool;
pub mod rx_queue;
pub mod tx_queue;
mod verified;
mod spec;


use hal::{*, regs::*, NumDesc, IXGBE_MAX_RX_QUEUES};
use queue_registers::*;
use rx_queue::{RxQueueE, RxQueueD, RxQueueF, RxQueueRSS};
use tx_queue::{TxQueueE, TxQueueD};

use spin::Once;
use alloc::vec::Vec;
use irq_safety::MutexIrqSafe;
use memory::{MappedPages, BorrowedMappedPages, Mutable};
use pci::{PciDevice,PciLocation};
use core::ops::Deref;
use log::{debug, info};

/// Vendor ID for Intel
pub const INTEL_VEND:                   u16 = 0x8086;  

/// Device ID for the 82599ES ethernet controller, used to identify the device from the PCI space
/// (https://www.intel.com/content/www/us/en/products/sku/41282/intel-82599es-10-gigabit-ethernet-controller/specifications.html)
pub const INTEL_82599ES:                u16 = 0x10FB;  

/// Device ID for the X520-DA2 network adapter, used to identify the device from the PCI space.
/// I'm not sure what makes this different from the `INTEL_82599ES` device, because its spec states it uses the 82599 controller as well.
/// (https://ark.intel.com/content/www/us/en/ark/products/39776/intel-ethernet-converged-network-adapter-x520da2.html)
pub const INTEL_X520_DA2:               u16 = 0x154D;  


/*** Developer Parameters of the Intel 82599 NIC ***/

/// The number of receive queues that are enabled. 
/// Do NOT set this greater than 64 since the queues 65-128 don't seem to work, 
/// most likely because they need additional configuration.
pub const IXGBE_NUM_RX_QUEUES_ENABLED:  u8 = 1;
pub const IXGBE_NUM_RX_QUEUES_USABLE:   u8 = 64;

/// The number of transmit queues that are enabled. 
/// Do NOT set this greater than 64 since the queues 65-128 don't seem to work, 
/// most likely because they need additional configuration.
pub const IXGBE_NUM_TX_QUEUES_ENABLED:  u8 = 1;
pub const IXGBE_NUM_TX_QUEUES_USABLE:   u8 = 64;

/// All buffers are created with 2KiB so that the max ethernet frame can fit in one packet buffer
pub const DEFAULT_RX_BUFFER_SIZE_2KB:   RxBufferSizeKiB = RxBufferSizeKiB::Buffer2KiB;


/*** Functions to get access to the IXGBE NICs once they've been initialized ***/

/// All the 82599 NICs found in the PCI space are initialized and then stored here.
pub static IXGBE_NICS: Once<Vec<MutexIrqSafe<IxgbeNic>>> = Once::new();

/// Returns a reference to the IxgbeNic wrapped in a MutexIrqSafe, if it exists and has been initialized.
/// Currently we use the pci location of the device as identification since it should not change after initialization.
pub fn get_ixgbe_nic(id: PciLocation) -> Result<&'static MutexIrqSafe<IxgbeNic>, &'static str> {
    let nics = IXGBE_NICS.get().ok_or("Ixgbe NICs weren't initialized")?;
    nics.iter()
        .find( |nic| { *nic.lock().pci_dev == id } )
        .ok_or("Ixgbe NIC with this ID does not exist")
}

/// Returns a reference to the list of all initialized ixgbe NICs
pub fn get_ixgbe_nics_list() -> Option<&'static Vec<MutexIrqSafe<IxgbeNic>>> {
    IXGBE_NICS.get()
}


/*** Driver Code Starts here ***/

/// A struct representing an ixgbe network interface card.
pub struct IxgbeNic {
    /// Representation of the PCI device of the NIC assigned by the device manager.
    pci_dev: PciDevice,
    /// The actual MAC address burnt into the hardware  
    mac_hardware: [u8;6], 
    regs: IxgbeRegisters,
    /// Array to store which L3/L4 5-tuple filters have been used.
    /// There are 128 such filters available.
    l34_5_tuple_filters: [Option<FilterParameters>; 128],
    /// The current RETA for RSS.
    /// We always enable RSS, but if no RETA is provided all routing is forwarded to queue 0.
    /// This is the default behavior, even when RSS isn't enabled.
    reta: [[RSSQueueID; 4]; 32],
    rx_queues: Vec<RxQueueE>,
    rx_queues_disabled: Vec<RxQueueD>,
    rx_queues_filters: Vec<RxQueueF>,
    rx_queues_rss: Vec<RxQueueRSS>,
    tx_queues: Vec<TxQueueE>,
    tx_queues_disabled: Vec<TxQueueD>,
}

impl IxgbeNic {
    /// Store required values from the device's PCI config space, and initialize different features of the nic.
    /// 
    /// # Arguments
    /// * `ixgbe_pci_dev`: Contains the pci device information for this NIC.
    /// * `reta`: redirection table for RSS
    pub fn init(
        ixgbe_pci_dev: PciDevice,
        reta: Option<[[RSSQueueID; 4]; 32]>
    ) -> Result<MutexIrqSafe<IxgbeNic>, &'static str> {
        let num_rx_descs = NumDesc::Descs512;
        let num_tx_descs = NumDesc::Descs512;

        let mmio_mapped_pages = ixgbe_pci_dev.pci_map_bar_mem(0)?;

        // set the bus mastering bit for this PciDevice, which allows it to use DMA
        ixgbe_pci_dev.pci_set_command_bus_master_bit();
        // ixgbe_pci_dev.pci_set_command_memory_space_bit();

        // map the IntelIxgbeRegisters structs to the address found from the pci space
        let mut regs = Self::cast_mp_into_regs(mmio_mapped_pages).map_err(|(_,err)| err)?;

        // link initialization
        Self::start_link(&mut regs.regs1, &mut regs.regs2, &mut regs.regs3, &mut regs.regs_mac)?;

        // clear stats registers
        Self::clear_stats_internal(&regs.regs2);

        // store the mac address of this device
        let mac_addr_hardware = Self::read_mac_address_from_nic(&mut regs.regs_mac);

        // create the rx descriptor queues
        let regs_rxq_unused = regs.regs_rxq_unused.split_off(IXGBE_NUM_RX_QUEUES_ENABLED as usize);
        let rx_regs = core::mem::replace(&mut regs.regs_rxq_unused, regs_rxq_unused);
        assert!(rx_regs.len() == IXGBE_NUM_RX_QUEUES_ENABLED as usize);
        let rx_queues = Self::rx_init(&mut regs.regs1, &mut regs.regs2, rx_regs, num_rx_descs)?;
        
        // create the tx descriptor queues
        let regs_txq_unused = regs.regs_txq_unused.split_off(IXGBE_NUM_TX_QUEUES_ENABLED as usize);
        let tx_regs = core::mem::replace(&mut regs.regs_txq_unused, regs_txq_unused);
        assert!(tx_regs.len() == IXGBE_NUM_TX_QUEUES_ENABLED as usize);
        let tx_queues = Self::tx_init(&mut regs.regs2, &mut regs.regs_mac, tx_regs, num_tx_descs)?;
        
        // enable Receive Side Scaling if required
        let reta = reta.unwrap_or([[RSSQueueID::Q0; 4]; 32]);
        // let rx_queues_rss = Self::enable_rss(&mut mapped_registers2, &mut mapped_registers3, &mut rx_queues, reta)?;

        // wait 10 seconds for the link to come up, as seen in other ixgbe drivers
        Self::wait_for_link(&regs.regs2, 10_000_000);

        let ixgbe_nic = IxgbeNic {
            pci_dev: ixgbe_pci_dev,
            mac_hardware: mac_addr_hardware,
            regs,
            l34_5_tuple_filters: [None; NUM_L34_5_TUPLE_FILTERS],
            reta,
            rx_queues,
            rx_queues_disabled: Vec::new(),
            rx_queues_filters: Vec::new(),
            rx_queues_rss: Vec::new(),
            tx_queues,
            tx_queues_disabled: Vec::new(),
        };

        info!("Link is up with speed: {} Mb/s", ixgbe_nic.link_speed() as u32);
        Ok(MutexIrqSafe::new(ixgbe_nic))
    }

    fn cast_mp_into_regs(mapped_pages: MappedPages) -> Result<IxgbeRegisters, (MappedPages, &'static str)> {
        // We've divided the memory-mapped registers into multiple regions.
        // The size of each region is found from the data sheet, but it always lies on a page boundary.
        // The size here is in pages, so we divide by 4096.
        const GENERAL_REGISTERS_1_SIZE:   usize = 4096 / 4096;
        const RX_REGISTERS_SIZE:          usize = 4096 / 4096;
        const GENERAL_REGISTERS_2_SIZE:   usize = 4 * 4096 / 4096;
        const TX_REGISTERS_SIZE:          usize = 2 * 4096 / 4096;
        const MAC_REGISTERS_SIZE:         usize = 5 * 4096 / 4096;
        // const GENERAL_REGISTERS_3_SIZE:   usize = 18 * 4096 / 4096;

        // Allocate memory for the registers, making sure each successive memory region begins where the previous region ended.
        let mut offset_page = *mapped_pages.deref().start() + GENERAL_REGISTERS_1_SIZE;
        let (nic_regs1_mapped_page, mapped_pages) = mapped_pages.split(offset_page)
            .map_err(|mp| (mp, "Failed to split mapped pages"))?;

        offset_page = *mapped_pages.deref().start() + RX_REGISTERS_SIZE;
        let (nic_rx_regs1_mapped_page, mapped_pages) = mapped_pages.split(offset_page)
            .map_err(|mp| (mp, "Failed to split mapped pages"))?;

        offset_page = *mapped_pages.deref().start() + GENERAL_REGISTERS_2_SIZE;
        let (nic_regs2_mapped_page, mapped_pages) = mapped_pages.split(offset_page)
            .map_err(|mp| (mp, "Failed to split mapped pages"))?;

        offset_page = *mapped_pages.deref().start() + TX_REGISTERS_SIZE;
        let (nic_tx_regs_mapped_page, mapped_pages) = mapped_pages.split(offset_page)
            .map_err(|mp| (mp, "Failed to split mapped pages"))?;

        offset_page = *mapped_pages.deref().start() + MAC_REGISTERS_SIZE;
        let (nic_mac_regs_mapped_page, mapped_pages) = mapped_pages.split(offset_page)
            .map_err(|mp| (mp, "Failed to split mapped pages"))?;

        offset_page = *mapped_pages.deref().start() + RX_REGISTERS_SIZE;
        let (nic_rx_regs2_mapped_page, nic_regs3_mapped_page) = mapped_pages.split(offset_page)
            .map_err(|mp| (mp, "Failed to split mapped pages"))?;

        // Map the memory as the register struct and tie the lifetime of the struct with its backing mapped pages
        let regs1 = nic_regs1_mapped_page.into_borrowed_mut(0)?;
        let regs2 = nic_regs2_mapped_page.into_borrowed_mut(0)?;
        let regs3 = nic_regs3_mapped_page.into_borrowed_mut(0)?;
        let regs_mac = nic_mac_regs_mapped_page.into_borrowed_mut(0)?;
        
        // Divide the pages of the Rx queue registers into multiple 64B regions
        let length = nic_rx_regs1_mapped_page.size_in_bytes() / core::mem::size_of::<RegistersRx>();
        let mut regs_rx1 = prusti_borrowed_shared_mp::verified::create_buffers_from_mp(nic_rx_regs1_mapped_page, length).map_err(|(mp,e)| (mp, e.into_str()))?;
        
        let length = nic_rx_regs2_mapped_page.size_in_bytes() / core::mem::size_of::<RegistersRx>();
        let mut regs_rx2 = prusti_borrowed_shared_mp::verified::create_buffers_from_mp(nic_rx_regs2_mapped_page, length).map_err(|(mp,e)| (mp, e.into_str()))?;
        regs_rx1.0.append(&mut regs_rx2.0); // move all 128 rx reg sets into 1

        // create the RxQueueRegisters objects which store the ID with the mapped registers
        let mut regs_rxq_unused = Vec::with_capacity(IXGBE_MAX_RX_QUEUES as usize);
        for id in 0..regs_rx1.len() {
            regs_rxq_unused.push(RxQueueRegisters {
                id,
                regs: regs_rx1.pop_front().unwrap()
            });
        }
        assert!(regs_rx1.len() == 0);
        assert!(regs_rx2.len() == 0);


        // Divide the pages of the Tx queue registers into multiple 64B regions
        let length = nic_tx_regs_mapped_page.size_in_bytes() / core::mem::size_of::<RegistersTx>();
        let mut regs_tx = prusti_borrowed_shared_mp::verified::create_buffers_from_mp(nic_tx_regs_mapped_page, length).map_err(|(mp,e)| (mp, e.into_str()))?;
        
        // create the TxQueueRegisters objects which store the ID with the mapped registers
        let mut regs_txq_unused: Vec<TxQueueRegisters> = Vec::with_capacity(IXGBE_MAX_TX_QUEUES as usize);
        for id in 0..regs_tx.len() {
            regs_txq_unused.push(TxQueueRegisters {
                id,
                regs: regs_tx.pop_front().unwrap()
            });
        }
        assert!(regs_tx.len() == 0);

        Ok(IxgbeRegisters {
            regs1, regs2, regs3, regs_mac, regs_rxq_unused, regs_txq_unused
        })
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
        regs2.fcrtv_clear();
        regs2.fccfg_clear();
        for r in &mut regs2.fcttv {
            r.write(0); // TinyNF removes because already 0
        }

        // regs2.fcrth_clear(); // TInyNF actually writes to this
        // Datasheet: Note that FCRTH[n].RTH fields must be set by default regardless if flow control is enabled or not
        // should be set to RXPBSIZE[n]-0x6000 ... and RXPBSIZE[0] is 512KB by default
        regs2.fcrth0_set_rth(((512u32 * 1024u32) - 0x6000) as u16);

        //disable interrupts
        regs1.eimc_disable_interrupts();

        //wait for eeprom auto read completion
        while !regs3.eec_auto_read(){}

        //read MAC address
        debug!("Ixgbe: MAC address low: {:#X}", regs_mac.ral.read());
        debug!("Ixgbe: MAC address high: {:#X}", regs_mac.rah());

        //wait for dma initialization done (RDRXCTL.DMAIDONE)
        while !regs2.rdrxctl_dma_init_done() {}

        // clear unicast table array
        for r in &mut regs3.pfuta {
            r.write(0);
        }

        // The following should all be cleared as their initial state is unknown
        // clear vlan filter table array
        // clear MAC pool select array
        // clear vlan pool filter bitmap
        // clear multicast table array
        // clear five tuple queue filter

        // debug!("STATUS: {:#X}", regs1.status.read()); 
        // debug!("CTRL: {:#X}", regs1.ctrl.read());
        // debug!("LINKS: {:#X}", regs2.links.read()); //b7 and b30 should be 1 for link up 
        // debug!("AUTOC: {:#X}", regs2.autoc.read()); 
        // debug!("AUTOC2: {:#X}", regs2.autoc2.read()); 

        Ok(())
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

    /// Initializes the array of receive descriptors and their corresponding receive buffers,
    /// and returns a tuple including both of them for all rx queues in use.
    /// Also enables receive functionality for the NIC.
    fn rx_init(
        regs1: &mut IntelIxgbeRegisters1, 
        regs2: &mut IntelIxgbeRegisters2, 
        rx_regs: Vec<RxQueueRegisters>,
        num_rx_descs: NumDesc,
    ) -> Result<Vec<RxQueueE>, &'static str> {

        //CRC offloading
        regs2.crc_strip();
        // Set values of reserved bits opposite to what HW sets them to
        regs2.rdrxctl_set_reserved_bits();
        // program RXPBSIZE according to DCB and virtualization modes (both off)
        for i in 1..8 {
            regs2.rxpbsize_reg1_7_set_buffer_size(
                RxPBReg::try_from(i).map_err(|_| "incorrect register ID for RxPBSize")?,
                RxPBSizeReg1_7::Size0KiB 
            );
        }
        regs2.mflcn_enable_receive_flow_control();

        let rxctrl_disabled = regs2.rxctrl_rx_disable();
        // set rx parameters of which type of packets are accepted by the nic
        // right now we allow the nic to receive all types of packets, even incorrectly formed ones
        let fctrl_set = regs2.fctrl_write(
            FilterCtrlFlags::STORE_BAD_PACKETS | 
            FilterCtrlFlags::MULTICAST_PROMISCUOUS_ENABLE | 
            FilterCtrlFlags::UNICAST_PROMISCUOUS_ENABLE | 
            FilterCtrlFlags::BROADCAST_ACCEPT_MODE, 
            rxctrl_disabled
        ); 
        // enable receive functionality
        regs2.rxctrl_rx_enable(fctrl_set); 

        let mut rx_all_queues = Vec::new();
        for rxq_reg in rx_regs {      
            let rxq = RxQueueE::new(rxq_reg, num_rx_descs)?;           
            rx_all_queues.push(rxq);
        }
        regs1.ctrl_ext_no_snoop_disable();        
        Ok(rx_all_queues)
    }

    /// Initialize the array of transmit descriptors for all queues and returns them.
    /// Also enables transmit functionality for the NIC.
    fn tx_init(
        regs2: &mut IntelIxgbeRegisters2, 
        regs_mac: &mut IntelIxgbeMacRegisters, 
        tx_regs: Vec<TxQueueRegisters>,
        num_tx_descs: NumDesc
    ) -> Result<Vec<TxQueueE>, &'static str> {
        regs2.fccfg_enable_transmit_flow_control();
        for i in 0..IXGBE_MAX_TX_QUEUES {
            regs2.rttdqsel_set_queue_id(i);
            regs2.rttdt1c_write(0);
        }
        // Set RTTFCS.ARBDIS to 1
        regs2.rttdcs_set_arbdis();
        
        for i in 1..8 {
            regs_mac.txpbsize_write(
                TxPBReg::try_from(i).map_err(|_| "incorrect reg idx for TXPBSIZE")?, 
                TxPBSize::Size0KiB
            );
        }
        // set to (packet buffer size / txpbsize) - MSS
        regs2.txpbthresh0_write(0xA0 - (RxBufferSizeKiB::Buffer2KiB as u16));
        regs_mac.dtxmxszrq_allow_max_byte_requests(); 
        regs2.rttdcs_clear_arbdis();

        let mut tx_all_queues = Vec::new();

        let mut tx_enabled = false;
        for txq_reg in tx_regs {            
            let (mut txq, tdh_set) = TxQueueE::new(txq_reg, num_tx_descs)?;
            if !tx_enabled {
                // enable transmit operation, only have to do this for the first queue
                // The placement of this line is vvv important!
                // Moving it before all the tx queue initialization will cause the TDWB to not work
                regs2.dmatxctl_enable_tx();
                tx_enabled = true;
            }

            // enable tx queue and make sure it's enabled
            txq.regs.txdctl_txq_enable(tdh_set); 
            const TX_Q_ENABLE: u32 = 1 << 25;
            while txq.regs.txdctl_read() & TX_Q_ENABLE == 0 {} 
            tx_all_queues.push(txq);
        }
        Ok(tx_all_queues)
    }  
}


// public functions
impl IxgbeNic {
    pub fn borrow_queue_pair(&mut self) -> Result<(&mut RxQueueE, &mut TxQueueE), &'static str> {
        if self.rx_queues.is_empty() || self.tx_queues.is_empty() {
            return Err("There is no queue pair available");
        }
        Ok((&mut self.rx_queues[0], &mut self.tx_queues[0]))
    }

    /// Clear the statistic registers by reading from them.
    pub fn clear_stats(&self) {
        Self::clear_stats_internal(&self.regs.regs2);
    }

    /// Returns link speed in Mb/s
    pub fn link_speed(&self) -> LinkSpeedMbps {
        let speed = self.regs.regs2.links.read() & LINKS_SPEED_MASK; 
        LinkSpeedMbps::from_links_register_value(speed)
    }

    /// Returns the Rx and Tx statistics for good packets.
    /// A good packet is one that is >= 64 bytes including ethernet header and CRC
    pub fn get_stats(&self, stats: &mut IxgbeStats) {
        let rx_bytes =  ((self.regs.regs2.gorch.read() as u64 & 0xF) << 32) | self.regs.regs2.gorcl.read() as u64;
        let tx_bytes =  ((self.regs.regs2.gotch.read() as u64 & 0xF) << 32) | self.regs.regs2.gotcl.read() as u64;

        stats.rx_bytes = rx_bytes;
        stats.tx_bytes = tx_bytes;
        stats.rx_packets = self.regs.regs2.gprc.read();
        stats.tx_packets = self.regs.regs2.gptc.read();
    }
}

/// All the MMIO registers for the NIC, except those that are located within the queue instances.
/// This was all done so that we could separate out the queue registers.
/// Otherwise we could have just had 1 single MappedPages cast to 1 register struct.
struct IxgbeRegisters {
    regs1: BorrowedMappedPages<IntelIxgbeRegisters1, Mutable>,
    regs2: BorrowedMappedPages<IntelIxgbeRegisters2, Mutable>,
    regs3: BorrowedMappedPages<IntelIxgbeRegisters3, Mutable>,
    regs_mac: BorrowedMappedPages<IntelIxgbeMacRegisters, Mutable>,
    regs_rxq_unused: Vec<RxQueueRegisters>,
    regs_txq_unused: Vec<TxQueueRegisters>,
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

}
}