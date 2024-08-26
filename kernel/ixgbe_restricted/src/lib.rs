//! A restricted driver for a 82599 10GbE Network Interface Card.

#![no_std]
// #![allow(unaligned_references)] // temporary, just to suppress unsafe packed borrows 
#![allow(incomplete_features)] // to allow adt_const_params without a warning
#![feature(rustc_private)]
#![feature(ptr_internals)]

extern crate alloc;

pub mod hal;
pub mod agent;
pub mod ethernet_frame;
mod agent_state;
pub use hal::*;
use hal::regs::*;

use spin::Once;
use alloc::vec::Vec;
use irq_safety::MutexIrqSafe;
use memory::{BorrowedMappedPages, Mutable, MappedPages};
use pci::{PciDevice, PciLocation};
use prusti_memory_buffer::{BufferBackingStore, create_buffers_from_mp};
use log::{debug, info};
use core::ops::Deref;

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
pub const IXGBE_NUM_RX_QUEUES_ENABLED:      u8 = 1;
/// The number of transmit queues that are enabled. 
/// Do NOT set this greater than 64 since the queues 65-128 don't seem to work, 
/// most likely because they need additional configuration.
pub const IXGBE_NUM_TX_QUEUES_ENABLED:      u8 = 1;
/// All buffers are created with 2KiB so that the max ethernet frame can fit in one packet buffer
pub const DEFAULT_RX_BUFFER_SIZE_2KB:       RxBufferSizeKiB = RxBufferSizeKiB::Buffer2KiB;


/*** Functions to get access to the IXGBE NICs once they've been initialized ***/

/// All the 82599 NICs found in the PCI space are initialized and then stored here.
pub static IXGBE_NICS: Once<Vec<MutexIrqSafe<IxgbeNic>>> = Once::new();

/// Returns a reference to the IxgbeNic wrapped in a MutexIrqSafe, if it exists and has been initialized.
/// Currently we use the pci location of the device as identification since it should not change after initialization.
pub fn get_ixgbe_nic(id: PciLocation) -> Result<&'static MutexIrqSafe<IxgbeNic>, &'static str> {
    let nics = IXGBE_NICS.get().ok_or("Ixgbe NICs weren't initialized")?;
    nics.iter()
        .find( |nic| { *nic.lock().pci_dev.deref() == id } )
        .ok_or("Ixgbe NIC with this ID does not exist")
}

/// Returns a reference to the list of all initialized ixgbe NICs
pub fn get_ixgbe_nics_list() -> Option<&'static Vec<MutexIrqSafe<IxgbeNic>>> {
    IXGBE_NICS.get()
}


#[allow(dead_code)] // for unused registers
/// A struct representing an ixgbe network interface card.
pub struct IxgbeNic {
    /// Representation of the PCI device of the NIC assigned by the device manager.
    pci_dev: PciDevice,
    /// The actual MAC address burnt into the hardware  
    mac_hardware: [u8;6],       
    regs1: BorrowedMappedPages<IntelIxgbeRegisters1, Mutable>,
    regs2: BorrowedMappedPages<IntelIxgbeRegisters2, Mutable>,
    regs3: BorrowedMappedPages<IntelIxgbeRegisters3, Mutable>,
    regs_mac: BorrowedMappedPages<IntelIxgbeMacRegisters, Mutable>,
    regs_rx1: BufferBackingStore<RegistersRx>,
    regs_rx2:  BufferBackingStore<RegistersRx>,
    regs_tx: BufferBackingStore<RegistersTx>
}

// Functions that setup the NIC struct and handle the sending and receiving of packets.
impl IxgbeNic {
    /// Map the MMIO registers, and intialize the NIC.
    /// 
    /// # Arguments
    /// * `ixgbe_pci_dev`: Contains the pci device information for this NIC.
    pub fn init(
        ixgbe_pci_dev: PciDevice,
    ) -> Result<MutexIrqSafe<IxgbeNic>, &'static str> {
        let mmio_mapped_pages = ixgbe_pci_dev.pci_map_bar_mem(0)?;

        // set the bus mastering bit for this PciDevice, which allows it to use DMA
        ixgbe_pci_dev.pci_set_command_bus_master_bit();
        // ixgbe_pci_dev.pci_set_command_memory_space_bit(); // Do we need this???

        // map the IntelIxgbeRegisters structs to the address found from the pci space
        let (mut mapped_registers1, 
            mut mapped_registers2, 
            mut mapped_registers3, 
            mut mapped_registers_mac, 
            rx_mapped_registers1, 
            rx_mapped_registers2, 
            tx_mapped_registers
        ) = Self::cast_mp_into_regs(mmio_mapped_pages).map_err(|(_,err)| err)?;

        // link initialization
        Self::start_link(&mut mapped_registers1, &mut mapped_registers2, &mut mapped_registers3, &mut mapped_registers_mac)?;

        // clear stats registers
        Self::clear_stats_internal(&mapped_registers2);

        // store the mac address of this device
        let mac_addr_hardware = Self::read_mac_address_from_nic(&mut mapped_registers_mac);
        
        // do the basic tx and rx init for the device (not the per queue initialization)
        Self::rx_init_pre(&mut mapped_registers2)?;
        Self::tx_init_pre(&mut mapped_registers2, &mut mapped_registers_mac)?;
        Self::set_promiscuous(&mut mapped_registers2);

        // wait 10 seconds for the link to come up, as seen in other ixgbe drivers
        Self::wait_for_link(&mapped_registers2, 10_000_000);

        let ixgbe_nic = IxgbeNic {
            pci_dev: ixgbe_pci_dev,
            mac_hardware: mac_addr_hardware,
            regs1: mapped_registers1,
            regs2: mapped_registers2,
            regs3: mapped_registers3,
            regs_mac: mapped_registers_mac,
            regs_rx1: rx_mapped_registers1,
            regs_rx2: rx_mapped_registers2,
            regs_tx: tx_mapped_registers,
        };

        info!("Link is up with speed: {} Mb/s", ixgbe_nic.link_speed() as u32);

        Ok(MutexIrqSafe::new(ixgbe_nic))
    }

    fn cast_mp_into_regs(mapped_pages: MappedPages) -> Result<(
        BorrowedMappedPages<IntelIxgbeRegisters1, Mutable>, 
        BorrowedMappedPages<IntelIxgbeRegisters2, Mutable>, 
        BorrowedMappedPages<IntelIxgbeRegisters3, Mutable>, 
        BorrowedMappedPages<IntelIxgbeMacRegisters, Mutable>, 
        BufferBackingStore<RegistersRx>,
        BufferBackingStore<RegistersRx>,
        BufferBackingStore<RegistersTx>
    ), (MappedPages, &'static str)> {
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
        let mac_regs = nic_mac_regs_mapped_page.into_borrowed_mut(0)?;
        
        // Divide the pages of the Rx queue registers into multiple 64B regions
        let length = nic_rx_regs1_mapped_page.size_in_bytes() / core::mem::size_of::<RegistersRx>();
        let regs_rx1 = create_buffers_from_mp(nic_rx_regs1_mapped_page, length).map_err(|(mp,e)| (mp, e.into_str()))?;
        
        let length = nic_rx_regs2_mapped_page.size_in_bytes() / core::mem::size_of::<RegistersRx>();
        let regs_rx2 = create_buffers_from_mp(nic_rx_regs2_mapped_page, length).map_err(|(mp,e)| (mp, e.into_str()))?;

        // Divide the pages of the Tx queue registers into multiple 64B regions
        let length = nic_tx_regs_mapped_page.size_in_bytes() / core::mem::size_of::<RegistersTx>();
        let regs_tx = create_buffers_from_mp(nic_tx_regs_mapped_page, length).map_err(|(mp,e)| (mp, e.into_str()))?;
            
        Ok((regs1, regs2, regs3, mac_regs, regs_rx1, regs_rx2, regs_tx))
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
        // We don't do a SW reset because device is only initialized at power up
        regs1.ctrl_reset();

        //wait 10 ms
        let wait_time = 10_000;
        let _ =pit_clock::pit_wait(wait_time);

        //disable flow control.. write 0 TO FCTTV, FCRTL, FCRTH, FCRTV and FCCFG
        for fcttv in regs2.fcttv.iter_mut() {
            fcttv.write(0);
        }

        regs2.fcrtl_clear(); // TinyNF removes because already 0
        regs2.fcrtv_clear(); // TinyNF removes because already 0
        regs2.fccfg_clear(); // TinyNF removes because already 0
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

    fn rx_init_pre(regs2: &mut IntelIxgbeRegisters2) -> Result<(), &'static str> {
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
        Ok(())
    }

    fn tx_init_pre(regs2: &mut IntelIxgbeRegisters2, regs_mac: &mut IntelIxgbeMacRegisters) -> Result<(), &'static str> {
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

        Ok(())
    }   

    fn set_promiscuous(regs2: &mut IntelIxgbeRegisters2) {
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