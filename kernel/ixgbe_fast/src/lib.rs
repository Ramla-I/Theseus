//! An ixgbe driver for a 82599 10GbE Network Interface Card.
//! 
//! Currently we support basic send and receive, Receive Side Scaling (RSS), 5-tuple filters, and MSI interrupts. 
//! We also support language-level virtualization of the NIC so that applications can directly access their assigned transmit and receive queues.
//! When using virtualization, we disable RSS since we use 5-tuple filters to ensure packets are routed to the correct queues.
//! We also disable interrupts when using virtualization, since we do not yet have support for allowing applications to register their own interrupt handlers.

#![no_std]
#![allow(dead_code)] //  to suppress warnings for unused functions/methods
#![feature(abi_x86_interrupt)]

#[macro_use] extern crate log;
#[macro_use] extern crate lazy_static;
#[macro_use] extern crate static_assertions;
extern crate alloc;
extern crate spin;
extern crate irq_safety;
extern crate kernel_config;
extern crate memory;
extern crate pci; 
extern crate pit_clock_basic;
extern crate bit_field;
extern crate interrupts;
extern crate x86_64;
extern crate apic;
extern crate pic;
extern crate acpi;
extern crate volatile;
extern crate mpmc;
extern crate owning_ref;
extern crate rand;
extern crate hpet;
extern crate runqueue;
extern crate network_interface_card;
extern crate nic_initialization_fast;
extern crate intel_ethernet;
extern crate nic_buffers_fast;
extern crate nic_queues_fast;
extern crate physical_nic;
extern crate virtual_nic;
extern crate zerocopy;
extern crate hashbrown;

mod regs;
mod queue_registers;
use regs::*;
use queue_registers::*;

use spin::Once;
use alloc::{
    vec::Vec,
    collections::VecDeque,
    sync::Arc,
    boxed::Box,
};
use irq_safety::MutexIrqSafe;
use memory::{PhysicalAddress, MappedPages, create_contiguous_mapping, EntryFlags};
use pci::{PciDevice, MSIX_CAPABILITY, PciConfigSpaceAccessMechanism, PciLocation};
use bit_field::BitField;
use interrupts::register_msi_interrupt;
use x86_64::structures::idt::HandlerFunc;
use hpet::get_hpet;
use network_interface_card::NetworkInterfaceCard;
use nic_initialization_fast::*;
use intel_ethernet::descriptors::{AdvancedRxDescriptor, AdvancedTxDescriptor, RxDescriptor, TxDescriptor};    
use nic_buffers_fast::{PacketBuffer, ReceivedFrame};
use nic_queues_fast::{RxQueue, TxQueue, RxQueueRegisters, TxQueueRegisters};
use owning_ref::BoxRefMut;
use rand::{
    SeedableRng,
    RngCore,
    rngs::SmallRng
};
use core::mem::{ManuallyDrop};
use hashbrown::HashMap;

/// Vendor ID for Intel
pub const INTEL_VEND:                   u16 = 0x8086;  

/// Device ID for the 82599ES ethernet controller, used to identify the device from the PCI space
/// (https://www.intel.com/content/www/us/en/products/sku/41282/intel-82599es-10-gigabit-ethernet-controller/specifications.html)
pub const INTEL_82599ES:                  u16 = 0x10FB;  

/// Device ID for the X520-DA2 network adapter, used to identify the device from the PCI space.
/// I'm not sure what makes this different from the `INTEL_82599ES` device, because its spec states it uses the 82599 controller as well.
/// (https://ark.intel.com/content/www/us/en/ark/products/39776/intel-ethernet-converged-network-adapter-x520da2.html)
pub const INTEL_X520_DA2:                  u16 = 0x154D;  


/*** Hardware Device Parameters of the Intel 82599 NIC (taken from the datasheet) ***/

/// The maximum number of receive descriptors per queue.
/// This is the maximum value that has been tested for the 82599 device.
const IXGBE_MAX_RX_DESC:                    u16     = 8192;
/// The maximum number of transmit descriptors per queue.
/// This is the maximum value that has been tested for the 82599 device.
const IXGBE_MAX_TX_DESC:                    u16     = 8192;
/// The maximum number of rx queues available on this NIC. 
const IXGBE_MAX_RX_QUEUES:                  u8      = 128;
/// The maximum number of tx queues available on this NIC.
const IXGBE_MAX_TX_QUEUES:                  u8      = 128;
/// The number of l34 5-tuple filters.
const NUM_L34_5_TUPLE_FILTERS:              usize   = 128; 



/*** Developer Parameters of the Intel 82599 NIC ***/

/// The number of receive queues that are enabled. 
/// Do NOT set this greater than 64 since the queues 65-128 don't seem to work, 
/// most likely because they need additional configuration.
pub const IXGBE_NUM_RX_QUEUES_ENABLED:          u8      = 1;
/// The number of transmit queues that are enabled. 
/// Do NOT set this greater than 64 since the queues 65-128 don't seem to work, 
/// most likely because they need additional configuration.
pub const IXGBE_NUM_TX_QUEUES_ENABLED:          u8      = 1;



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

/// How many ReceiveBuffers are preallocated for this driver to use. 
const RX_BUFFER_POOL_SIZE: usize = IXGBE_NUM_RX_QUEUES_ENABLED as usize * IXGBE_MAX_RX_DESC as usize * 2; 

lazy_static! {
    /// The pool of pre-allocated receive buffers that are used by the IXGBE NIC
    /// and temporarily given to higher layers in the networking stack.
    /// 
    /// # Note
    /// The capacity always has to be greater than the number of buffers in the queue, which is why we multiply by 2.
    /// I'm not sure why that is, but if we try to add packets >= capacity, the addition does not make any progress.
    static ref RX_BUFFER_POOL: mpmc::Queue<PacketBuffer> = mpmc::Queue::with_capacity(RX_BUFFER_POOL_SIZE * 2);
}

/// A struct representing an ixgbe network interface card.
pub struct IxgbeNic {
    /// Device ID of the NIC assigned by the device manager.
    dev_id: PciLocation,
    /// Type of Base Address Register 0,
    /// if it's memory mapped or I/O.
    bar_type: u8,
    /// MMIO Base Address     
    mem_base: PhysicalAddress,
    /// Hashmap to store the interrupt number for each msi vector.
    /// The key is the id of the queue the interrupt is generated for,
    /// and the value is the interrupt number.
    interrupt_num: HashMap<u8,u8>,
    /// The actual MAC address burnt into the hardware  
    mac_hardware: [u8;6],       
    /// The optional spoofed MAC address to use in place of `mac_hardware` when transmitting.  
    mac_spoofed: Option<[u8; 6]>,
    /// Memory-mapped control registers
    regs1: BoxRefMut<MappedPages, IntelIxgbeRegisters1>,
    /// Memory-mapped control registers
    regs2: BoxRefMut<MappedPages, IntelIxgbeRegisters2>,
    /// Memory-mapped control registers
    regs3: BoxRefMut<MappedPages, IntelIxgbeRegisters3>,
    /// Memory-mapped control registers
    regs_mac: BoxRefMut<MappedPages, IntelIxgbeMacRegisters>,
    /// The number of rx queues enabled
    num_rx_queues: u8,
    /// Vector of the enabled rx queues
    pub rx_queues: Vec<RxQueue<IxgbeRxQueueRegisters,AdvancedRxDescriptor>>,
    /// Registers for the disabled queues
    rx_registers_disabled: Vec<IxgbeRxQueueRegisters>,
    /// The number of tx queues enabled
    num_tx_queues: u8,
    /// Vector of the enabled tx queues
    tx_queues: Vec<TxQueue<IxgbeTxQueueRegisters,AdvancedTxDescriptor>>,
    /// Registers for the disabled queues
    tx_registers_disabled: Vec<IxgbeTxQueueRegisters>,
    ///prefetchable
    pub prefetchable: u32
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
        dev_id: PciLocation,
        enable_virtualization: bool,
        interrupts: Option<Vec<HandlerFunc>>,
        enable_rss: bool,
        rx_buffer_size_kbytes: RxBufferSizeKiB,
        num_rx_descriptors: u16,
        num_tx_descriptors: u16
    ) -> Result<MutexIrqSafe<IxgbeNic>, &'static str> {
        // Series of checks to determine if starting parameters are acceptable
        if enable_virtualization && (interrupts.is_some() || enable_rss) {
            return Err("Cannot enable virtualization when interrupts or RSS are enabled");
        }

        if let Some(ref ints) = interrupts {
            if ints.len() > IXGBE_NUM_RX_QUEUES_ENABLED as usize {
                return Err("The number of interrupts must be less than or equal to the number of Rx queues enabled");
            }
        }

        if num_rx_descriptors > IXGBE_MAX_RX_DESC {
            return Err("We can have a maximum of 8K receive descriptors per queue");
        }

        if (num_rx_descriptors as usize * core::mem::size_of::<AdvancedRxDescriptor>()) % 128 != 0 {
            return Err("The total length in bytes of the Rx descriptor ring must be 128-byte aligned");
        }

        if num_tx_descriptors > IXGBE_MAX_TX_DESC {
            return Err("We can have a maximum of 8K transmit descriptors per queue");
        }

        if (num_tx_descriptors as usize * core::mem::size_of::<AdvancedTxDescriptor>()) % 128 != 0 {
            return Err("The total length in bytes of the Tx descriptor ring must be 128-byte aligned");
        }

        // Start the initialization procedure

        let bar0 = ixgbe_pci_dev.bars[0];
        // Determine the type from the base address register
        let bar_type = (bar0 as u8) & 0x01;    

        // If the base address is not memory mapped then exit
        if bar_type == PciConfigSpaceAccessMechanism::IoPort as u8 {
            error!("ixgbe::init(): BAR0 is of I/O type");
            return Err("ixgbe::init(): BAR0 is of I/O type")
        }

        // 16-byte aligned memory mapped base address
        let mem_base =  ixgbe_pci_dev.determine_mem_base(0)?;

        // set the bus mastering bit for this PciDevice, which allows it to use DMA
        ixgbe_pci_dev.pci_set_command_bus_master_bit();

        // map the IntelIxgbeRegisters structs to the address found from the pci space
        let (mut mapped_registers1, mut mapped_registers2, mut mapped_registers3, mut mapped_registers_mac, 
            mut rx_mapped_registers, mut tx_mapped_registers) = Self::mapped_reg(mem_base)?;


        // link initialization
        Self::start_link(&mut mapped_registers1, &mut mapped_registers2, &mut mapped_registers3, &mut mapped_registers_mac)?;

        // clear stats registers
        Self::clear_stats(&mapped_registers2);

        // store the mac address of this device
        let mac_addr_hardware = Self::read_mac_address_from_nic(&mut mapped_registers_mac);

        // initialize the buffer pool
        init_rx_buf_pool(RX_BUFFER_POOL_SIZE, rx_buffer_size_kbytes as u16 * 1024, &RX_BUFFER_POOL)?;

        // create the rx desc queues and their packet buffers
        let (mut rx_descs, mut rx_buffers) = Self::rx_init(&mut mapped_registers1, &mut mapped_registers2, &mut rx_mapped_registers, num_rx_descriptors, rx_buffer_size_kbytes)?;
        
        // create the vec of rx queues
        let mut rx_queues = Vec::with_capacity(rx_descs.len());
        let mut id = 0;
        while !rx_descs.is_empty() {
            let rx_queue = RxQueue {
                id: id,
                regs: rx_mapped_registers.remove(0),
                rx_descs: rx_descs.remove(0),
                num_rx_descs: num_rx_descriptors,
                rx_cur: 0,
                rx_bufs_in_use: rx_buffers.remove(0),  
                rx_buffer_size_bytes: rx_buffer_size_kbytes as u16 * 1024,
                received_frames: VecDeque::new(),
                cpu_id : None,
                rx_buffer_pool: &RX_BUFFER_POOL,
                filter_num: None
            };
            rx_queues.push(rx_queue);
            id += 1;
        }


        // create the tx descriptor queues
        let mut tx_descs = Self::tx_init(&mut mapped_registers2, &mut mapped_registers_mac, &mut tx_mapped_registers, num_tx_descriptors)?;
        
        // create the vec of tx queues
        let mut tx_queues = Vec::with_capacity(tx_descs.len());
        let mut id = 0;
        while !tx_descs.is_empty() {
            let tx_queue = TxQueue {
                id: id,
                regs: tx_mapped_registers.remove(0),
                tx_descs: tx_descs.remove(0),
                num_tx_descs: num_tx_descriptors,
                tx_cur: 0,
                tx_clean: 0,
                tx_bufs_in_use: VecDeque::with_capacity(num_tx_descriptors as usize),
                cpu_id : None,
            };
            tx_queues.push(tx_queue);
            id += 1;
        }

        // enable msi-x interrupts if required and return the assigned interrupt numbers
        let interrupt_num = HashMap::new();


        // wait 10 seconds for the link to come up, as seen in other ixgbe drivers
        Self::wait_for_link(&mapped_registers2, 10_000_000);

        let ixgbe_nic = IxgbeNic {
            dev_id: dev_id,
            bar_type: bar_type,
            mem_base: mem_base,
            interrupt_num: interrupt_num,
            mac_hardware: mac_addr_hardware,
            mac_spoofed: None,
            regs1: mapped_registers1,
            regs2: mapped_registers2,
            regs3: mapped_registers3,
            regs_mac: mapped_registers_mac,
            num_rx_queues: IXGBE_NUM_RX_QUEUES_ENABLED,
            rx_queues: rx_queues,
            rx_registers_disabled: rx_mapped_registers,
            num_tx_queues: IXGBE_NUM_TX_QUEUES_ENABLED,
            tx_queues: tx_queues,
            tx_registers_disabled: tx_mapped_registers,
            prefetchable: ixgbe_pci_dev.determine_prefetchable(0)?
        };

        info!("Link is up with speed: {} Mb/s", ixgbe_nic.link_speed() as u32);

        Ok(MutexIrqSafe::new(ixgbe_nic))
    }

    pub fn mac_address(&self) -> [u8; 6] {
        self.mac_spoofed.unwrap_or(self.mac_hardware)
    }

    /// Returns the device id of the PCI device.
    pub fn device_id(&self) -> PciLocation {
        self.dev_id
    }

    /// Returns the memory-mapped control registers of the nic and the rx/tx queue registers.
    fn mapped_reg(
        mem_base: PhysicalAddress
    ) -> Result<(
        BoxRefMut<MappedPages, IntelIxgbeRegisters1>, 
        BoxRefMut<MappedPages, IntelIxgbeRegisters2>, 
        BoxRefMut<MappedPages, IntelIxgbeRegisters3>, 
        BoxRefMut<MappedPages, IntelIxgbeMacRegisters>, 
        Vec<IxgbeRxQueueRegisters>, 
        Vec<IxgbeTxQueueRegisters>
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
        let mut offset = mem_base;
        let nic_regs1_mapped_page = allocate_memory(offset, GENERAL_REGISTERS_1_SIZE_BYTES)?;

        offset += GENERAL_REGISTERS_1_SIZE_BYTES;
        let nic_rx_regs1_mapped_page = allocate_memory(offset, RX_REGISTERS_SIZE_BYTES)?;

        offset += RX_REGISTERS_SIZE_BYTES;
        let nic_regs2_mapped_page = allocate_memory(offset, GENERAL_REGISTERS_2_SIZE_BYTES)?;  

        offset += GENERAL_REGISTERS_2_SIZE_BYTES;
        let nic_tx_regs_mapped_page = allocate_memory(offset, TX_REGISTERS_SIZE_BYTES)?;

        offset += TX_REGISTERS_SIZE_BYTES;
        let nic_mac_regs_mapped_page = allocate_memory(offset, MAC_REGISTERS_SIZE_BYTES)?;

        offset += MAC_REGISTERS_SIZE_BYTES;
        let nic_rx_regs2_mapped_page = allocate_memory(offset, RX_REGISTERS_SIZE_BYTES)?;   

        offset += RX_REGISTERS_SIZE_BYTES;
        let nic_regs3_mapped_page = allocate_memory(offset, GENERAL_REGISTERS_3_SIZE_BYTES)?;

        // Map the memory as the register struct and tie the lifetime of the struct with its backing mapped pages
        let regs1 = BoxRefMut::new(Box::new(nic_regs1_mapped_page)).try_map_mut(|mp| mp.as_type_mut::<IntelIxgbeRegisters1>(0))?;
        let regs2 = BoxRefMut::new(Box::new(nic_regs2_mapped_page)).try_map_mut(|mp| mp.as_type_mut::<IntelIxgbeRegisters2>(0))?;
        let regs3 = BoxRefMut::new(Box::new(nic_regs3_mapped_page)).try_map_mut(|mp| mp.as_type_mut::<IntelIxgbeRegisters3>(0))?;
        let mac_regs = BoxRefMut::new(Box::new(nic_mac_regs_mapped_page)).try_map_mut(|mp| mp.as_type_mut::<IntelIxgbeMacRegisters>(0))?;
        
        // Divide the pages of the Rx queue registers into multiple 64B regions
        let mut regs_rx = Self::mapped_regs_from_rx_memory(nic_rx_regs1_mapped_page);
        regs_rx.append(&mut Self::mapped_regs_from_rx_memory(nic_rx_regs2_mapped_page));
        
        // Divide the pages of the Tx queue registers into multiple 64B regions
        let regs_tx = Self::mapped_regs_from_tx_memory(nic_tx_regs_mapped_page);
            
        Ok((regs1, regs2, regs3, mac_regs, regs_rx, regs_tx))
    }

    /// Split the pages where rx queue registers are mapped into multiple smaller memory regions.
    /// One region contains all the registers for a single queue.
    fn mapped_regs_from_rx_memory(mp: MappedPages) -> Vec<IxgbeRxQueueRegisters> {
        const QUEUES_IN_MP: usize = 64;
        const RX_QUEUE_REGISTERS_SIZE_BYTES: usize = core::mem::size_of::<RegistersRx>();
        
        assert!(mp.size_in_bytes() >= QUEUES_IN_MP * RX_QUEUE_REGISTERS_SIZE_BYTES);

        let starting_address = mp.start_address();

        // We share the backing mapped pages among all the queue registers
        let shared_mp = Arc::new(mp);
        let mut pointers_to_queues = Vec::with_capacity(QUEUES_IN_MP);

        for i in 0..QUEUES_IN_MP {
            // This is safe because we have checked that the number of queues we want to partition from these mapped pages fit into the allocated memory,
            // and that each queue starts at the end of the previous.
            // We also ensure that the backing mapped pages are included in the same struct as the registers, almost as a pseudo OwningRef
            let registers = unsafe{ Box::from_raw((starting_address.value() + (i * RX_QUEUE_REGISTERS_SIZE_BYTES)) as *mut RegistersRx) };
            pointers_to_queues.push(
                IxgbeRxQueueRegisters {
                    regs: ManuallyDrop::new(registers),
                    backing_pages: shared_mp.clone()
                }
            );
        }
        pointers_to_queues
    }

    /// Split the pages where tx queue registers are mapped into multiple smaller memory regions.
    /// One region contains all the registers for a single queue.
    fn mapped_regs_from_tx_memory(mp: MappedPages) -> Vec<IxgbeTxQueueRegisters> {
        const QUEUES_IN_MP: usize = 128;
        const TX_QUEUE_REGISTERS_SIZE_BYTES: usize = core::mem::size_of::<RegistersTx>();
        
        assert!(mp.size_in_bytes() >= QUEUES_IN_MP * TX_QUEUE_REGISTERS_SIZE_BYTES);

        let starting_address = mp.start_address();

        // We share the backing mapped pages among all the queue registers
        let shared_mp = Arc::new(mp);
        let mut pointers_to_queues = Vec::with_capacity(QUEUES_IN_MP);

        for i in 0..QUEUES_IN_MP {
            // This is safe because we have checked that the number of queues we want to partition from these mapped pages fit into the allocated memory,
            // and that each queue starts at the end of the previous.
            // We also ensure that the backing mapped pages are included in the same struct as the registers, almost as a pseudo OwningRef
            let registers = unsafe{ Box::from_raw((starting_address.value() + (i * TX_QUEUE_REGISTERS_SIZE_BYTES)) as *mut RegistersTx) };
            pointers_to_queues.push(
                IxgbeTxQueueRegisters {
                    regs: ManuallyDrop::new(registers),
                    backing_pages: shared_mp.clone()
                }
            );
        }
        pointers_to_queues
    }

    pub fn spoof_mac(&mut self, spoofed_mac_addr: [u8; 6]) {
        self.mac_spoofed = Some(spoofed_mac_addr);
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
        regs1.eimc.write(DISABLE_INTERRUPTS);

        // master disable algorithm (sec 5.2.5.3.2)
        // global reset = sw reset + link reset 
        let val = regs1.ctrl.read();
        regs1.ctrl.write(val|CTRL_RST|CTRL_LRST);

        //wait 10 ms
        let wait_time = 10_000;
        pit_clock_basic::pit_wait(wait_time)?;

        //disable flow control.. write 0 TO FCTTV, FCRTL, FCRTH, FCRTV and FCCFG
        for fcttv in regs2.fcttv.iter_mut() {
            fcttv.write(0);
        }

        for fcrtl in regs2.fcrtl.iter_mut() {
            fcrtl.write(0);
        }

        for fcrth in regs2.fcrth.iter_mut() {
            fcrth.write(0);
        }

        regs2.fcrtv.write(0);
        regs2.fccfg.write(0);

        //disable interrupts
        regs1.eimc.write(DISABLE_INTERRUPTS);

        //wait for eeprom auto read completion
        while !regs3.eec.read().get_bit(EEC_AUTO_RD as u8){}

        //read MAC address
        debug!("Ixgbe: MAC address low: {:#X}", regs_mac.ral.read());
        debug!("Ixgbe: MAC address high: {:#X}", regs_mac.rah.read() & 0xFFFF);

        //wait for dma initialization done (RDRXCTL.DMAIDONE)
        let mut val = regs2.rdrxctl.read();
        let dmaidone_bit = 1 << 3;
        while val & dmaidone_bit != dmaidone_bit {
            val = regs2.rdrxctl.read();
        }

        // while Self::acquire_semaphore(regs3)? {
        //     //wait 10 ms
        //     pit_clock_basic::pit_wait(wait_time)?;
        // }

        // setup PHY and the link 
        // From looking at other drivers and testing, it seems these registers are set automatically 
        // and driver doesn't need to configure link speed manually.

        // Self::release_semaphore(regs3);        

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
            let _ = pit_clock_basic::pit_wait(wait_time); // wait, or try again regardless
            tries += 1;
        }
    }

    /// Clear the statistic registers by reading from them.
    fn clear_stats(regs: &IntelIxgbeRegisters2) {
        regs.gprc.read();
        regs.gptc.read();
        regs.gorcl.read();
        regs.gorch.read();
        regs.gotcl.read();
        regs.gotch.read();
    }

    /// Clear the statistic registers by reading from them.
    pub fn clear_stats2(&self) {
        self.regs2.gprc.read();
        self.regs2.gptc.read();
        self.regs2.gorcl.read();
        self.regs2.gorch.read();
        self.regs2.gotcl.read();
        self.regs2.gotch.read();
    }

    /// Returns the Rx and Tx statistics in the form: (Good Rx packets, Good Rx bytes, Good Tx packets, Good Tx bytes).
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
        rx_regs: &mut Vec<IxgbeRxQueueRegisters>,
        num_rx_descs: u16,
        rx_buffer_size_kbytes: RxBufferSizeKiB
    ) -> Result<(
        Vec<BoxRefMut<MappedPages, [AdvancedRxDescriptor]>>, 
        Vec<Vec<PacketBuffer>>
    ), &'static str> {

        let mut rx_descs_all_queues = Vec::new();
        let mut rx_bufs_in_use_all_queues = Vec::new();

        Self::disable_rx_function(regs);
        // program RXPBSIZE according to DCB and virtualization modes (both off)
        regs.rxpbsize[0].write(RXPBSIZE_128KB);
        for i in 1..8 {
            regs.rxpbsize[i].write(0);
        }
        //CRC offloading
        regs.hlreg0.write(regs.hlreg0.read() | HLREG0_CRC_STRIP);
        regs.rdrxctl.write(regs.rdrxctl.read() | RDRXCTL_CRC_STRIP);
        // Clear bits
        regs.rdrxctl.write(regs.rdrxctl.read() & !RDRXCTL_RSCFRSTSIZE);

        for qid in 0..IXGBE_NUM_RX_QUEUES_ENABLED {
            let rxq = &mut rx_regs[qid as usize];        

            // get the queue of rx descriptors and their corresponding rx buffers
            let (rx_descs, rx_bufs_in_use) = init_rx_queue(num_rx_descs as usize, &RX_BUFFER_POOL, rx_buffer_size_kbytes as usize * 1024, rxq)?;          
            
            //set the size of the packet buffers and the descriptor format used
            let mut val = rxq.srrctl.read();
            // val.set_bits(0..4, rx_buffer_size_kbytes as u32);
            // val.set_bits(8..13, BSIZEHEADER_0B);
            val.set_bits(25..27, DESCTYPE_ADV_1BUFFER);
            val = val | DROP_ENABLE;
            rxq.srrctl.write(val);

            //enable the rx queue
            let val = rxq.rxdctl.read();
            rxq.rxdctl.write(val | RX_Q_ENABLE);

            //make sure queue is enabled
            while rxq.rxdctl.read() & RX_Q_ENABLE == 0 {}
        
            // set bit 12 to 0
            let val = rxq.dca_rxctrl.read();
            rxq.dca_rxctrl.write(val & !DCA_RXCTRL_CLEAR_BIT_12);

            // Write the tail index.
            // Note that the 82599 datasheet (section 8.2.3.8.5) states that we should set the RDT (tail index) to the index *beyond* the last receive descriptor, 
            // but we set it to the last receive descriptor for the same reason as the e1000 driver
            rxq.rdt.write((num_rx_descs - 1) as u32);
            
            rx_descs_all_queues.push(rx_descs);
            rx_bufs_in_use_all_queues.push(rx_bufs_in_use);
        }
        
        Self::enable_rx_function(regs1,regs);
        Ok((rx_descs_all_queues, rx_bufs_in_use_all_queues))
    }

    /// disable receive functionality
    fn disable_rx_function(regs: &mut IntelIxgbeRegisters2) {        
        let val = regs.rxctrl.read();
        regs.rxctrl.write(val & !RECEIVE_ENABLE); 
    }

    /// enable receive functionality
    fn enable_rx_function(regs1: &mut IntelIxgbeRegisters1,regs: &mut IntelIxgbeRegisters2) {
        // set rx parameters of which type of packets are accepted by the nic
        // right now we allow the nic to receive all types of packets, even incorrectly formed ones
        regs.fctrl.write(STORE_BAD_PACKETS | MULTICAST_PROMISCUOUS_ENABLE | UNICAST_PROMISCUOUS_ENABLE | BROADCAST_ACCEPT_MODE); 
        
        // some magic numbers
        regs1.ctrl_ext.write(regs1.ctrl_ext.read() | CTRL_EXT_NO_SNOOP_DIS);

        // enable receive functionality
        let val = regs.rxctrl.read();
        regs.rxctrl.write(val | RECEIVE_ENABLE); 
    }

    /// Initialize the array of transmit descriptors for all queues and returns them.
    /// Also enables transmit functionality for the NIC.
    fn tx_init(
        regs: &mut IntelIxgbeRegisters2, 
        regs_mac: &mut IntelIxgbeMacRegisters, 
        tx_regs: &mut Vec<IxgbeTxQueueRegisters>,
        num_tx_descs: u16
    ) -> Result<Vec<BoxRefMut<MappedPages, [AdvancedTxDescriptor]>>, &'static str> {
        // disable transmission
        Self::disable_transmission(regs);

        // CRC offload and small packet padding enable
        regs.hlreg0.write(regs.hlreg0.read() | HLREG0_TXCRCEN | HLREG0_TXPADEN);

        // Set RTTFCS.ARBDIS to 1
        regs.rttdcs.write(regs.rttdcs.read() | RTTDCS_ARBDIS);

        // program DTXMXSZRQ and TXPBSIZE according to DCB and virtualization modes (both off)
        regs_mac.txpbsize[0].write(TXPBSIZE_160KB);
        for i in 1..8 {
            regs_mac.txpbsize[i].write(0);
        }
        regs_mac.dtxmxszrq.write(DTXMXSZRQ_MAX_BYTES); 

        // Clear RTTFCS.ARBDIS
        regs.rttdcs.write(regs.rttdcs.read() & !RTTDCS_ARBDIS);

        let mut tx_descs_all_queues = Vec::new();
        
        for qid in 0..IXGBE_NUM_TX_QUEUES_ENABLED {
            let txq = &mut tx_regs[qid as usize];

            let tx_descs = init_tx_queue(num_tx_descs as usize, txq)?;
        
            if qid == 0 {
                // enable transmit operation, only have to do this for the first queue
                Self::enable_transmission(regs);
            }

            // Set descriptor thresholds
            // If we enable this then we need to change the packet send function to stop polling
            // for a descriptor done on every packet sent
            txq.txdctl.write(TXDCTL_PTHRESH | TXDCTL_HTHRESH | TXDCTL_WTHRESH); 

            //enable tx queue
            let val = txq.txdctl.read();
            txq.txdctl.write(val | TX_Q_ENABLE); 

            //make sure queue is enabled
            while txq.txdctl.read() & TX_Q_ENABLE == 0 {} 

            tx_descs_all_queues.push(tx_descs);
        }
        Ok(tx_descs_all_queues)
    }  

    /// disable transmit functionality
    fn disable_transmission(regs: &mut IntelIxgbeRegisters2) {
        let val = regs.dmatxctl.read();
        regs.dmatxctl.write(val & !TE); 
    }

    /// enable transmit functionality
    fn enable_transmission(regs: &mut IntelIxgbeRegisters2) {
        let val = regs.dmatxctl.read();
        regs.dmatxctl.write(val | TE); 
    }


    /// Reads status and clears interrupt
    fn clear_interrupt_status(&self) -> u32 {
        self.regs1.eicr.read()
    }

    /// Retrieves `num_packets` packets from queue `qid` and stores them in `buffers`.
    /// Returns the total number of received packets.
    pub fn rx_batch(&mut self, qid: usize, buffers: &mut Vec<PacketBuffer>, num_packets: usize, pool: &mut Vec<PacketBuffer>) -> Result<usize, &'static str> {
        if qid >= self.rx_queues.len() {
            return Err("Invalid queue id");
        }

        let queue = &mut self.rx_queues[qid];
        let mut rx_cur = queue.rx_cur as usize;
        let mut last_rx_cur = queue.rx_cur as usize;

        let mut rcvd_pkts = 0;

        for _ in 0..num_packets {
            let desc = &mut queue.rx_descs[rx_cur];
            // error!("last_rx_cur = {}, rx_cur = {}", last_rx_cur, rx_cur);
            if !desc.descriptor_done() {
                // error!("DD = {}", queue.rx_descs[rx_cur].descriptor_done());
                break;
            }

            if !desc.end_of_packet() {
                return Err("Currently do not support multi-descriptor packets");
            }

            let length = desc.length();

            // Now that we are "removing" the current receive buffer from the list of receive buffers that the NIC can use,
            // (because we're saving it for higher layers to use),
            // we need to obtain a new `ReceiveBuffer` and set it up such that the NIC will use it for future receivals.
            if let Some(new_receive_buf) = pool.pop() {
                // actually tell the NIC about the new receive buffer, and that it's ready for use now
                desc.set_packet_address(new_receive_buf.phys_addr);
                desc.reset_status();
                
                let mut current_rx_buf = core::mem::replace(&mut queue.rx_bufs_in_use[rx_cur], new_receive_buf);
                current_rx_buf.length = length as u16; // set the ReceiveBuffer's length to the size of the actual packet received
                buffers.push(current_rx_buf);

                rcvd_pkts += 1;
                last_rx_cur = rx_cur;
                rx_cur = (rx_cur + 1) & (queue.num_rx_descs as usize - 1);
            } else {
                return Err("Ran out of packet buffers");
            }
        }

        if last_rx_cur != rx_cur {
            queue.rx_cur = rx_cur as u16;
            queue.regs.set_rdt(last_rx_cur as u32); 
        }

        Ok(rcvd_pkts)

    }

    /// Retrieves `num_packets` packets from queue `qid` and stores them in `buffers`.
    /// Returns the total number of received packets.
    pub fn rx_batch_pseudo(&mut self, qid: usize, num_packets: usize) -> usize {
        let queue = &mut self.rx_queues[qid];
        let mut rx_cur = queue.rx_cur as usize;
        let mut last_rx_cur = queue.rx_cur as usize;

        let mut rcvd_pkts = 0;

        for _ in 0..num_packets {
            let desc = &mut queue.rx_descs[rx_cur];
            if !desc.descriptor_done() {
                break;
            }

            // actually tell the NIC about the new receive buffer, and that it's ready for use now
            desc.set_packet_address(queue.rx_bufs_in_use[rx_cur].phys_addr);
            desc.reset_status();
                
            rcvd_pkts += 1;
            last_rx_cur = rx_cur;
            rx_cur = (rx_cur + 1) & (queue.num_rx_descs as usize - 1);
        }

        if last_rx_cur != rx_cur {
            queue.rx_cur = rx_cur as u16;
            queue.regs.set_rdt(last_rx_cur as u32); 
        }

        rcvd_pkts
    }

    /// Sends all packets in `buffers` on queue `qid` if there are descriptors available.
    /// (number of packets sent, used transmit buffers that can now be dropped or reused) are returned.
    pub fn tx_batch(&mut self, qid: usize, buffers: &mut Vec<PacketBuffer>, used_buffers: &mut Vec<PacketBuffer>) -> Result<usize, &'static str> {
        if qid >= self.tx_queues.len() {
            return Err("Invalid queue id");
        }

        let mut pkts_sent = 0;
        let queue = &mut self.tx_queues[qid];
        let mut tx_cur = queue.tx_cur;

        Self::tx_clean(queue, used_buffers);
        let tx_clean = queue.tx_clean;
        // debug!("tx_cur = {}, tx_clean ={}", tx_cur, tx_clean);

        while let Some(packet) = buffers.pop() {
            let tx_next = (tx_cur + 1) % queue.num_tx_descs;

            if tx_clean == tx_next {
                // tx queue of device is full, push packet back onto the
                // queue of to-be-sent packets
                buffers.push(packet);
                break;
            }

            queue.tx_descs[tx_cur as usize].send(packet.phys_addr, packet.length);
            queue.tx_bufs_in_use.push_back(packet);

            tx_cur = tx_next;
            pkts_sent += 1;
        }

        queue.tx_cur = tx_cur;
        queue.regs.set_tdt(tx_cur as u32);

        Ok(pkts_sent)
    }

    /// Sets all the descriptors in the tx queue with a valid packet buffer but doesn't update the TDT
    /// assumes that the length of `buffers` is equal to the number of descriptors in the queue
    pub fn tx_populate(&mut self, qid: usize, buffers: &mut Vec<PacketBuffer>) {
        let queue = &mut self.tx_queues[qid];

        for desc in queue.tx_descs.iter_mut() {
            let packet = buffers.pop().unwrap();
            desc.send(packet.phys_addr, packet.length);
            queue.tx_bufs_in_use.push_back(packet);
        }
        assert!(queue.tx_bufs_in_use.len() == queue.tx_descs.len());
    }

    /// Sends all packets in `buffers` on queue `qid` if there are descriptors available.
    /// (number of packets sent, used transmit buffers that can now be dropped or reused) are returned.
    pub fn tx_batch_pseudo(&mut self, qid: usize, batch_size: usize) -> usize {
        let mut pkts_sent = 0;
        let queue = &mut self.tx_queues[qid];
        let mut tx_cur = queue.tx_cur;

        Self::tx_clean_pseudo(queue);
        let tx_clean = queue.tx_clean;

        for _ in 0..batch_size {
            let tx_next = (tx_cur + 1) % queue.num_tx_descs;

            if tx_clean == tx_next {
                // tx queue of device is full
                break;
            }

            queue.tx_descs[tx_cur as usize].send(queue.tx_bufs_in_use[tx_cur as usize].phys_addr, queue.tx_bufs_in_use[tx_cur as usize].length);

            tx_cur = tx_next;
            pkts_sent += 1;
        }

        queue.tx_cur = tx_cur;
        queue.regs.set_tdt(tx_cur as u32);

        pkts_sent
    }
   
    pub fn tx_batch_pseudo1(&mut self, qid: usize, batch_size: usize) {
        let queue = &mut self.tx_queues[qid];
        let mut tx_cur = queue.tx_cur as usize;

        for i in 0..batch_size {
            queue.tx_descs[tx_cur].send(queue.tx_bufs_in_use[tx_cur].phys_addr, queue.tx_bufs_in_use[tx_cur].length);
            tx_cur = (tx_cur + 1) % queue.num_tx_descs as usize;
        }

        queue.tx_cur = tx_cur as u16;
        queue.regs.set_tdt(tx_cur as u32);
    }
    
    /// Removes multiples of `TX_CLEAN_BATCH` packets from `queue`.    
    /// (code taken from https://github.com/ixy-languages/ixy.rs/blob/master/src/ixgbe.rs#L1016)
    fn tx_clean(queue: &mut TxQueue<IxgbeTxQueueRegisters,AdvancedTxDescriptor>, used_buffers: &mut Vec<PacketBuffer>)  {
        const TX_CLEAN_BATCH: usize = 32;

        let mut tx_clean = queue.tx_clean as usize;
        let tx_cur = queue.tx_cur;

        loop {
            let mut cleanable = tx_cur as i32 - tx_clean as i32;

            if cleanable < 0 {
                cleanable += queue.num_tx_descs as i32;
            }
    
            if cleanable < TX_CLEAN_BATCH as i32 {
                break;
            }
    
            let mut cleanup_to = tx_clean + TX_CLEAN_BATCH - 1;

            if cleanup_to >= queue.num_tx_descs as usize {
                cleanup_to -= queue.num_tx_descs as usize;
            }

            if queue.tx_descs[cleanup_to].desc_done() {
                if TX_CLEAN_BATCH >= queue.tx_bufs_in_use.len() {
                    used_buffers.extend(queue.tx_bufs_in_use.drain(..))
                } else {
                    used_buffers.extend(queue.tx_bufs_in_use.drain(..TX_CLEAN_BATCH))
                };

                tx_clean = (cleanup_to + 1) % queue.num_tx_descs as usize;
            } else {
                break;
            }
        }

        queue.tx_clean = tx_clean as u16;
    }

        /// Removes multiples of `TX_CLEAN_BATCH` packets from `queue`.    
    /// (code taken from https://github.com/ixy-languages/ixy.rs/blob/master/src/ixgbe.rs#L1016)
    fn tx_clean_pseudo(queue: &mut TxQueue<IxgbeTxQueueRegisters,AdvancedTxDescriptor>) {
        const TX_CLEAN_BATCH: usize = 32;
        let mut tx_clean = queue.tx_clean as usize;
        let tx_cur = queue.tx_cur;

        loop {
            let mut cleanable = tx_cur as i32 - tx_clean as i32;

            if cleanable < 0 {
                cleanable += queue.num_tx_descs as i32;
            }
    
            if cleanable < TX_CLEAN_BATCH as i32 {
                break;
            }
    
            let mut cleanup_to = tx_clean + TX_CLEAN_BATCH - 1;

            if cleanup_to >= queue.num_tx_descs as usize {
                cleanup_to -= queue.num_tx_descs as usize;
            }

            if queue.tx_descs[cleanup_to].desc_done() {
                tx_clean = (cleanup_to + 1) % queue.num_tx_descs as usize;
            } else {
                break;
            }
        }

        queue.tx_clean = tx_clean as u16;
    }
}

/// Possible link speeds of the 82599 NIC
#[derive(PartialEq)]
pub enum LinkSpeedMbps {
    LS100 = 100,
    LS1000 = 1000,
    LS10000 = 10000, 
    LSUnknown = 0,
}

impl LinkSpeedMbps {
    /// Converts between a u32 and a LinkSpeedMbps enum.
    /// The u32 number is the value in the links register that represents the link speed.
    fn from_links_register_value(value: u32) -> LinkSpeedMbps {
        if value == (1 << 28) {
            Self::LS100
        } else if value == (2 << 28) {
            Self::LS1000
        } else if value == (3 << 28) {
            Self::LS10000
        } else {
            Self::LSUnknown
        }
    }
}

/// The set of receive buffer sizes that are accepted by the 82599 device.
#[derive(Copy, Clone)]
pub enum RxBufferSizeKiB {
    Buffer1KiB = 1,
    Buffer2KiB = 2,
    Buffer3KiB = 3,
    Buffer4KiB = 4,
    Buffer5KiB = 5,
    Buffer6KiB = 6,
    Buffer7KiB = 7,
    Buffer8KiB = 8,
    Buffer9KiB = 9,
    Buffer10KiB = 10,
    Buffer11KiB = 11,
    Buffer12KiB = 12,
    Buffer13KiB = 13,
    Buffer14KiB = 14,
    Buffer15KiB = 15,
    Buffer16KiB = 16
}

/// Options for the filter protocol used in the 5-tuple filters.
pub enum FilterProtocol {
    Tcp = 0,
    Udp = 1,
    Sctp = 2,
    Other = 3
}

/// Creates a `TransmitBuffer` that contains a packet with only the ethernet header.
pub fn create_raw_packet(
    dest_mac_address: &[u8], 
    source_mac_address: &[u8], 
    message: &[u8]
) -> Result<PacketBuffer, &'static str> {
    
    const ETHER_TYPE_LEN: usize = 2;
    const MAC_ADDR_LEN: usize = 6;
    const ETHERNET_HEADER_LEN: usize = MAC_ADDR_LEN * 2 + ETHER_TYPE_LEN;
    const MIN_PACKET_LEN: usize = 46;

    let mut len = message.len() as u16;
    if len > 1500 {
        return Err("Too long for a raw packet");
    }
    if len < MIN_PACKET_LEN as u16 {
        len = 46;
    }

    let ether_type: [u8; ETHER_TYPE_LEN] = [(len >> 8) as u8, len as u8];

    let mut transmit_buffer = PacketBuffer::new(ETHERNET_HEADER_LEN as u16 + len)?;
    { 
        let buffer: &mut [u8] = transmit_buffer.as_slice_mut(0, MAC_ADDR_LEN)?;
        buffer.copy_from_slice(&dest_mac_address);
    }
    { 
        let buffer: &mut [u8] = transmit_buffer.as_slice_mut(6, MAC_ADDR_LEN)?;
        buffer.copy_from_slice(&source_mac_address);
    }
    { 
        let buffer: &mut [u8] = transmit_buffer.as_slice_mut(12, ETHER_TYPE_LEN)?;
        buffer.copy_from_slice(&ether_type);
    }
    { 
        let buffer: &mut [u8] = transmit_buffer.as_slice_mut(14, message.len())?;
        buffer.copy_from_slice(&message);
    }

    Ok(transmit_buffer)
}


#[derive(Default, Debug)]
pub struct IxgbeStats{
    pub rx_bytes: u64,
    pub tx_bytes: u64,
    pub rx_packets: u32,
    pub tx_packets: u32,
}
