#![no_std]
#![allow(dead_code)] //  to suppress warnings for unused functions/methods
#![allow(safe_packed_borrows)] // temporary, just to suppress unsafe packed borrows 

#[macro_use] extern crate log;
#[macro_use] extern crate lazy_static;
extern crate alloc;
extern crate spin;
extern crate sync_irq;
extern crate kernel_config;
extern crate memory;
extern crate pci; 
extern crate interrupts;

use core::ptr::{read_volatile, write_volatile};
use core::ops::DerefMut;
use spin::Once; 
use alloc::vec::Vec;
use sync_irq::IrqSafeMutex;

use memory::{VirtualAddress, MappedPages, create_contiguous_mapping, MMIO_FLAGS};
use pci::{PciDevice};
use kernel_config::memory::PAGE_SIZE;
use interrupts::InterruptNumber;

static INTEL_VEND:              u16 = 0x8086;  // Vendor ID for Intel 
static E1000_DEV:               u16 = 0x100E;  // Device ID for the e1000 Qemu, Bochs, and VirtualBox emmulated NICs
const E1000_I217:               u16 = 0x153A;  // Device ID for Intel I217
const E1000_82577LM:            u16 = 0x10EA;  // Device ID for Intel 82577LM
const PCI_BAR0:                 u16 = 0x10;
const PCI_INTERRUPT_LINE:       u16 = 0x3C;

const REG_CTRL:                 u32 = 0x0000;
const REG_STATUS:               u32 = 0x0008;
const REG_EEPROM:               u32 = 0x0014;
const REG_CTRL_EXT:             u32 = 0x0018;
const REG_IMASK:                u32 = 0x00D0;
const REG_RCTRL:                u32 = 0x0100;
const REG_RXDESCLO:             u32 = 0x2800;
const REG_RXDESCHI:             u32 = 0x2804;
const REG_RXDESCLEN:            u32 = 0x2808;
const REG_RXDESCHEAD:           u32 = 0x2810;
const REG_RXDESCTAIL:           u32 = 0x2818;

const REG_TCTRL:                u32 = 0x0400;
const REG_TXDESCLO:             u32 = 0x3800;
const REG_TXDESCHI:             u32 = 0x3804;
const REG_TXDESCLEN:            u32 = 0x3808;
const REG_TXDESCHEAD:           u32 = 0x3810;
const REG_TXDESCTAIL:           u32 = 0x3818;

const REG_RDTR:                 u32 = 0x2820;    // RX Delay Timer Register
const REG_RXDCTL:               u32 = 0x3828;    // RX Descriptor Control
const REG_RADV:                 u32 = 0x282C;    // RX Int. Absolute Delay Timer
const REG_RSRPD:                u32 = 0x2C00;    // RX Small Packet Detect Interrupt
 
const REG_MTA:                  u32 = 0x5200; 
const REG_CRCERRS:              u32 = 0x4000;        
 
const REG_TIPG:                 u32 = 0x0410;      // Transmit Inter Packet Gap
const ECTRL_SLU:                u32 = 0x40;        // set link up

///CTRL commands
const CTRL_LRST:                u32 = 1<<3; 
const CTRL_ILOS:                u32 = 1<<7; 
const CTRL_VME:                 u32 = 1<<30; 
const CTRL_PHY_RST:             u32 = 1<<31;

/// RCTL registers 
const RCTL_EN:                  u32 = 1 << 1;    // Receiver Enable
const RCTL_SBP:                 u32 = 1 << 2;    // Store Bad Packets
const RCTL_UPE:                 u32 = 1 << 3;    // Unicast Promiscuous Enabled
const RCTL_MPE:                 u32 = 1 << 4;    // Multicast Promiscuous Enabled
const RCTL_LPE:                 u32 = 1 << 5;    // Long Packet Reception Enable
const RCTL_LBM_NONE:            u32 = 0 << 6;    // No Loopback
const RCTL_LBM_PHY:             u32 = 3 << 6;    // PHY or external SerDesc loopback
const RTCL_RDMTS_HALF:          u32 = 0 << 8;    // Free Buffer Threshold is 1/2 of RDLEN
const RTCL_RDMTS_QUARTER:       u32 = 1 << 8;    // Free Buffer Threshold is 1/4 of RDLEN
const RTCL_RDMTS_EIGHTH:        u32 = 2 << 8;    // Free Buffer Threshold is 1/8 of RDLEN
const RCTL_MO_36:               u32 = 0 << 12;   // Multicast Offset - bits 47:36
const RCTL_MO_35:               u32 = 1 << 12;   // Multicast Offset - bits 46:35
const RCTL_MO_34:               u32 = 2 << 12;   // Multicast Offset - bits 45:34
const RCTL_MO_32:               u32 = 3 << 12;   // Multicast Offset - bits 43:32
const RCTL_BAM:                 u32 = 1 << 15;   // Broadcast Accept Mode
const RCTL_VFE:                 u32 = 1 << 18;   // VLAN Filter Enable
const RCTL_CFIEN:               u32 = 1 << 19;   // Canonical Form Indicator Enable
const RCTL_CFI:                 u32 = 1 << 20;   // Canonical Form Indicator Bit Value
const RCTL_DPF:                 u32 = 1 << 22;   // Discard Pause Frames
const RCTL_PMCF:                u32 = 1 << 23;   // Pass MAC Control Frames
const RCTL_SECRC:               u32 = 1 << 26;   // Strip Ethernet CRC
 
/// Buffer Sizes
const RCTL_BSIZE_256:           u32 = 3 << 16;
const RCTL_BSIZE_512:           u32 = 2 << 16;
const RCTL_BSIZE_1024:          u32 = 1 << 16;
const RCTL_BSIZE_2048:          u32 = 0 << 16;
const RCTL_BSIZE_4096:          u32 = (3 << 16) | (1 << 25);
const RCTL_BSIZE_8192:          u32 = (2 << 16) | (1 << 25);
const RCTL_BSIZE_16384:         u32 = (1 << 16) | (1 << 25);
 
 
/// Transmit Command
 
const CMD_EOP:                  u32 = 1 << 0;    // End of Packet
const CMD_IFCS:                 u32 = 1 << 1;   // Insert FCS
const CMD_IC:                   u32 = 1 << 2;    // Insert Checksum
const CMD_RS:                   u32 = 1 << 3;   // Report Status
const CMD_RPS:                  u32 = 1 << 4;   // Report Packet Sent
const CMD_VLE:                  u32 = 1 << 6;    // VLAN Packet Enable
const CMD_IDE:                  u32 = 1 << 7;    // Interrupt Delay Enable
 
 
/// TCTL Register
 
const TCTL_EN:                  u32 = 1 << 1;    // Transmit Enable
const TCTL_PSP:                 u32 = 1 << 3;    // Pad Short Packets
const TCTL_CT_SHIFT:            u32 = 4;          // Collision Threshold
const TCTL_COLD_SHIFT:          u32 = 12;          // Collision Distance
const TCTL_SWXOFF:              u32 = 1 << 22;   // Software XOFF Transmission
const TCTL_RTLC:                u32 = 1 << 24;   // Re-transmit on Late Collision
 
const TSTA_DD:                  u32 = 1 << 0;    // Descriptor Done
const TSTA_EC:                  u32 = 1 << 1;    // Excess Collisions
const TSTA_LC:                  u32 = 1 << 2;    // Late Collision
const LSTA_TU:                  u32 = 1 << 3;    // Transmit Underrun

const E1000_NUM_RX_DESC:        usize = 8;
const E1000_NUM_TX_DESC:        usize = 8;
const E1000_SIZE_RX_DESC:       usize = 256;
const E1000_SIZE_TX_DESC:       usize = 256;

/// to hold memory mappings
static NIC_PAGES: Once<MappedPages> = Once::new();
static NIC_DMA_PAGES: Once<MappedPages> = Once::new();

/// The single instance of the E1000 NIC.
/// TODO: in the future, we should support multiple NICs all stored elsewhere,
/// e.g., on the PCI bus or somewhere else.
static E1000_NIC: Once<IrqSafeMutex<Nic>> = Once::new();

/// struct to represent receive descriptors
#[repr(C,packed)]
pub struct e1000_rx_desc {
        addr: u64,      
        length: u16,
        checksum: u16,
        status: u8,
        errors: u8,
        special: u16,
}
// use core::fmt;
// impl fmt::Debug for e1000_rx_desc {
//         fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
//                 write!(f, "{{addr: {:#X}, length: {}, checksum: {}, status: {}, errors: {}, special: {}}}",
//                         self.addr, self.length, self.checksum, self.status, self.errors, self.special)
//         }
// }

/// struct to represent transmission descriptors
#[repr(C,packed)]
pub struct e1000_tx_desc {
        addr: u64,
        length: u16,
        cso: u8,
        cmd: u8,
        status: u8,
        css: u8,
        special : u16,
}
// impl fmt::Debug for e1000_tx_desc {
//         fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
//                 write!(f, "{{addr: {:#X}, length: {}, cso: {}, cmd: {}, status: {}, css: {}, special: {}}}",
//                         self.addr, self.length, self.cso, self.cmd, self.status, self.css, self.special)
//         }
// }

/// struct to hold information for the network card
pub struct Nic {
        /// Type of BAR0
        bar_type: u8,
        /// IO Base Address     
        io_base: u32,
        /// MMIO Base Address     
        mem_base: usize,   
        /// The interrupt vector number used by this device to trigger interrupts.
        interrupt_num: Option<InterruptNumber>,
        /// A flag indicating if eeprom exists
        eeprom_exists: bool,
        /// A buffer for storing the mac address  
        mac: [u8;6],       
        /// Receive Descriptors
        rx_descs: Vec<e1000_rx_desc>, 
        /// Transmit Descriptors 
        tx_descs: Vec<e1000_tx_desc>, 
        /// Current Receive Descriptor Buffer
        rx_cur: u16,      
        /// Current Transmit Descriptor Buffer
        tx_cur: u16,
        /// stores the virtual address of rx buffers
        rx_buf_addr: [usize;E1000_NUM_RX_DESC],
        /// The DMA allocator for the nic 
        nic_dma_allocator: DmaAllocator,
}

/// struct that stores addresses for memory allocated for DMA
pub struct DmaAllocator {
        /// starting address of physically contiguous memory
        start: usize,
        /// ending address of physically contiguous memory 
        end: usize, 
        /// starting address of available memory 
        current: usize, 
}

impl DmaAllocator {

    /// allocates DMA memory if amount is available, size is in bytes
    pub fn allocate_dma_mem(&mut self, size: usize) -> Option<usize> {
        let prev_current;                
                
        if self.end-self.current > size {
                prev_current = self.current;
                self.current = self.current + size;
                debug!("start: {:x} end: {:x} prev_current: {:x} current: {:x}", self.start,self.end,prev_current,self.current);
                return Some(prev_current);
        } else {
                return None;
        }
    }
}


/// translate virtual address to physical address
pub fn translate_v2p(v_addr : usize) -> Option<usize> {  
    let vaddr = VirtualAddress::new_canonical(v_addr);
    memory::translate(vaddr).map(|x| x.value())
}

/// functions that setup the NIC struct and handle the sending and receiving of packets
impl Nic{
    /// store required values from the devices PCI config space
    pub fn init(&mut self,ref dev:&PciDevice){
        use interrupts::IRQ_BASE_OFFSET;

        // Type of BAR0
        self.bar_type = (dev.bars[0] as u8) & 0x01;    
        // IO Base Address
        self.io_base = dev.bars[0] & !1;     
        // memory mapped base address
        self.mem_base = (dev.bars[0] as usize) & !3; //hard coded for 32 bit, need to make conditional     

        dev.pci_set_command_bus_master_bit();

        // Get interrupt number
        let interrupt_num = match dev.pci_get_intx_info() {
            Ok((Some(irq), _pin)) => (irq + IRQ_BASE_OFFSET) as InterruptNumber,
            _ => panic!("e1000: PCI device had no interrupt number (IRQ vector)"),
        };
        self.interrupt_num = Some(interrupt_num);
        dev.pci_enable_intx(true);         
    }

    /// allocates memory for the NIC, starting address and size taken from the PCI BAR0
    pub fn mem_map (&mut self,ref dev:&PciDevice) -> Result<(), &'static str>{
        let pages_nic = dev.pci_map_bar_mem(0)?;
        self.mem_base = pages_nic.start().start_address().value();
        debug!("new mem_base: {:#X}",self.mem_base);
        
        NIC_PAGES.call_once(|| pages_nic);      
        
        //checking device status register
        let val: u32 = unsafe { read_volatile((self.mem_base + REG_STATUS as usize) as *const u32) };
        debug!("DSR: {}",val);

        Ok(())
    }

    /// allocates memory for DMA, will be used by the rx and tx descriptors
    pub fn mem_map_dma(&mut self) -> Result<(), &'static str> {

        let (mp, _paddr) = create_contiguous_mapping(PAGE_SIZE * 4, MMIO_FLAGS)?;
        let virt_addr = mp.start().start_address().value();
        NIC_DMA_PAGES.call_once(|| mp);
            
        self.nic_dma_allocator.start = virt_addr;
        self.nic_dma_allocator.current = virt_addr;
        self.nic_dma_allocator.end = virt_addr + (PAGE_SIZE * 4); // 4 page is allocated
        trace!("head_dma_mem: {:#X}, tail_dma_mem: {:#X}", self.nic_dma_allocator.start, self.nic_dma_allocator.end);
            
        Ok(())
    }

    /// write to an NIC register
    /// p_address is register offset
    fn write_command(&self, p_address: u32, p_value: u32){
        unsafe { write_volatile((self.mem_base + p_address as usize) as *mut u32, p_value) };
    }

    /// read from an NIC register
    /// p_address is register offset
    fn read_command(&self, p_address: u32) -> u32 {
        let val = unsafe { read_volatile((self.mem_base + p_address as usize) as *const u32) };
        val
    }
 
    /// sets the eeprom_exists data member
    pub fn detect_eeprom(&mut self) {

        let mut val: u32;
        let mut i: u16 = 0;
        self.write_command(REG_EEPROM, 0x1);    
        
        while i < 1000 && ! self.eeprom_exists //???
        {
            val = self.read_command(REG_EEPROM);
            if (val & 0x10) == 0x10 {
                self.eeprom_exists = true;
            } else {
                self.eeprom_exists = false;
            }
            i = i+1;
        }
        debug!("eeprom_exists: {}",self.eeprom_exists);     
    } 
        
    /// Read 4 bytes from a specific EEProm Address
    pub fn eeprom_read( &self,addr: u16) -> u32 {
        let mut tmp: u32 = 0;
        if self.eeprom_exists {
            let x = ((addr) << 8) as u32;//addr bits are 15:8
            self.write_command( REG_EEPROM, (1) | x ); //write addr to eeprom read register and simulatenously write 1 to start read bit
            while (tmp & 0x10) != 0x10 { //check read done bit
                    tmp = self.read_command(REG_EEPROM);
            }
        }
        else { //why?
            let x = ((addr) << 2) as u32; //read 4 bytes (1 word)
            self.write_command( REG_EEPROM, (1) | x); 
            while (tmp & 0x02) != 0x02 {
                    tmp = self.read_command(REG_EEPROM);
            }
        }
        let data = (tmp >> 16) & 0x0000_FFFF; // data bits are 31:16
        data
    }

    /// Read MAC Address
    pub fn read_mac_addr(&mut self) -> bool {
        if self.eeprom_exists {
            let mut temp: u32 = self.eeprom_read(0);
            self.mac[0] = temp as u8 & 0xff;
            self.mac[1] = (temp >> 8) as u8;
            temp = self.eeprom_read(1);
            self.mac[2] = temp as u8 & 0xff;
            self.mac[3] = (temp >> 8) as u8;
            temp = self.eeprom_read(2);
            self.mac[4] = temp as u8 & 0xff;
            self.mac[5] = (temp >> 8) as u8;
        } else {
            let mac_32_low = self.read_command(0x5400);
            let mac_32_high = self.read_command(0x5404);
            if mac_32_low != 0 {
                self.mac[0] = mac_32_low as u8;
                self.mac[1] = (mac_32_low >> 8) as u8;
                self.mac[2] = (mac_32_low >> 16) as u8;
                self.mac[3] = (mac_32_low >> 24) as u8;
                self.mac[4] = mac_32_high as u8;
                self.mac[5] = (mac_32_high >> 8) as u8;
                    
            } else {
                    return false;
            }
        }
        debug!("MAC address: {:?}",self.mac);
        return true;

    }   

    /// Start up the network
    pub fn start_link (&self) -> bool { 
        //for i217 just check that bit1 is set of reg status
        let val = self.read_command(REG_CTRL);
        self.write_command(REG_CTRL, val | 0x40 | 0x20);

        let val = self.read_command(REG_CTRL);
        self.write_command(REG_CTRL, val & !(CTRL_LRST) & !(CTRL_ILOS) & !(CTRL_VME) & !(CTRL_PHY_RST));

        debug!("REG_CTRL: {:#X}", self.read_command(REG_CTRL));

        return true;           
    } 

    /// clear multicast registers
    pub fn clear_multicast (&self) {
        for i in 0..128 {
            self.write_command(REG_MTA + (i * 4), 0);
        }
    }

    /// clear statistic registers
    pub fn clear_statistics (&self) {
        for i in 0..64 {
            self.write_command(REG_CRCERRS + (i * 4), 0);
        }
    }      

    /// Initialize receive descriptors and rx buffers
    pub fn rx_init(&mut self) -> Result<(), &'static str> {
        const NO_BYTES : usize = 16 * (E1000_NUM_RX_DESC + 1);
        let dma_ptr = self.nic_dma_allocator.allocate_dma_mem(NO_BYTES);
        let ptr;

        match dma_ptr {
            Some(_x) => ptr = dma_ptr.unwrap(),
            None => return Err("e1000:rx_init Couldn't allocate DMA mem for rx descriptors"),
        }

        let ptr1 = ptr + (16-(ptr%16));
        debug!("pointers: {:x}, {:x}",ptr, ptr1);

        let raw_ptr = ptr1 as *mut e1000_rx_desc;
        debug!("size of e1000_rx_desc: {}, e1000_tx_desc: {}", 
                ::core::mem::size_of::<e1000_rx_desc>(), ::core::mem::size_of::<e1000_tx_desc>());
        
        unsafe{ self.rx_descs = Vec::from_raw_parts(raw_ptr, 0, E1000_NUM_RX_DESC);}
        //unsafe{debug!("Address of Rx desc: {:?}, value: {:?}",ptr, *pr1);}
        // debug!("rx_descs: {:?}, capacity: {}", self.rx_descs, self.rx_descs.capacity());

        for i in 0..E1000_NUM_RX_DESC
        {
            let dma_ptr = self.nic_dma_allocator.allocate_dma_mem(E1000_SIZE_RX_DESC);
            match dma_ptr{
                    Some(_x) => self.rx_buf_addr[i] = dma_ptr.unwrap(),
                    None => return Err("e1000:rx_init Couldn't allocate DMA mem for rx buffer"),
            } 
                                    
            let rx_buf = translate_v2p(self.rx_buf_addr[i]);
            let buf_addr;
            match rx_buf{
                    Some(_x) => buf_addr = rx_buf.unwrap() as u64,
                    None => return Err("e1000:rx_init Couldn't translate address for rx buffers"),
            }
            
            //let buf_addr = (translate_v2p(self.rx_buf_addr[i])).unwrap();
            let mut var = e1000_rx_desc {
                    addr: buf_addr as u64,
                    length: 0,
                    checksum: 0,
                    status: 0,
                    errors: 0,
                    special: 0,
            };
                            
            // debug!("packet buffer: {:x}",var.addr);
            self.rx_descs.push(var);
        }
        
        let slc = self.rx_descs.as_slice(); 
        let slc_ptr = slc.as_ptr();
        let v_addr = slc_ptr as usize;
        debug!("v address of rx_desc: {:x}",v_addr);
        
        let t_ptr = translate_v2p(v_addr);
        let ptr;
        match t_ptr{
            Some(_x) =>  ptr = t_ptr.unwrap(),
            None => return Err("e1000:rx_init Couldn't translate address for rx descriptor"),
        }

        //let ptr = (translate_v2p(v_addr)).unwrap();
        
        debug!("p address of rx_desc: {:x}",ptr);
        let ptr1 = (ptr & 0xFFFF_FFFF) as u32;
        let ptr2 = (ptr>>32) as u32;

        
        self.write_command(REG_RXDESCLO, ptr1);//lowers bits of 64 bit descriptor base address, 16 byte aligned
        self.write_command(REG_RXDESCHI, ptr2);//upper 32 bits
        
        self.write_command(REG_RXDESCLEN, (E1000_NUM_RX_DESC as u32)* 16);//number of bytes allocated for descriptors, 128 byte aligned
        
        self.write_command(REG_RXDESCHEAD, 0);//head pointer for reeive descriptor buffer, points to 16B
        self.write_command(REG_RXDESCTAIL, E1000_NUM_RX_DESC as u32);//Tail pointer for receive descriptor buffer, point to 16B
        self.rx_cur = 0;
        self.write_command(REG_RCTRL, RCTL_EN| RCTL_SBP | RCTL_LBM_NONE | RTCL_RDMTS_HALF | RCTL_BAM | RCTL_SECRC  | RCTL_BSIZE_256);
        //self.write_command(REG_RCTRL, RCTL_EN| RCTL_SBP| RCTL_UPE | RCTL_MPE | RCTL_LBM_NONE | RTCL_RDMTS_HALF | RCTL_BAM | RCTL_SECRC  | RCTL_BSIZE_256);
        Ok(())
    }               
        
    /// Initialize transmit descriptors 
    pub fn tx_init(&mut self) -> Result<(), &'static str>  {
        const NO_BYTES: usize = 16*(E1000_NUM_TX_DESC+1);
        
        let dma_ptr = self.nic_dma_allocator.allocate_dma_mem(NO_BYTES);
        let ptr;
        match dma_ptr{
            Some(_x) => ptr = dma_ptr.unwrap(),
            None => return Err("e1000:tx_init Couldn't allocate DMA mem for tx descriptor"),
        } 

        // make sure memory is 16 byte aligned
        let ptr1 = ptr + (16-(ptr%16));
        debug!("tx pointers: {:x}, {:x}",ptr, ptr1);

        let raw_ptr = ptr1 as *mut e1000_tx_desc;
        unsafe{ self.tx_descs = Vec::from_raw_parts(raw_ptr, 0, E1000_NUM_TX_DESC);}
        //unsafe{debug!("Address of Rx desc: {:?}, value: {:?}",ptr, *pr1);}
        
        for _i in 0..E1000_NUM_TX_DESC
        {
            let mut var = e1000_tx_desc {
                addr: 0,
                length: 0,
                cso: 0,
                cmd: 0,
                status: 0,
                css: 0,
                special : 0,
            };
            self.tx_descs.push(var);
        }

        //TODO: don't need this, use ptr1
        let slc = self.tx_descs.as_slice(); 
        let slc_ptr = slc.as_ptr();
        let v_addr = slc_ptr as usize;
        debug!("v address of tx_desc: {:x}",v_addr);

        let t_ptr = translate_v2p(v_addr);
        let ptr;
        match t_ptr{
            Some(_x) => ptr = t_ptr.unwrap(),
            None => return Err("e1000:tx_init Couldn't translate address for tx descriptor"),
        }
        //let ptr = (translate_v2p(v_addr)).unwrap();
        
        
        debug!("p address of tx_desc: {:x}",ptr);

        let ptr1 = (ptr & 0xFFFF_FFFF) as u32;
        let ptr2 = (ptr>>32) as u32;
        
        self.write_command(REG_TXDESCHI, ptr2 );
        self.write_command(REG_TXDESCLO, ptr1);                
        
        //now setup total length of descriptors
        self.write_command(REG_TXDESCLEN, (E1000_NUM_TX_DESC as u32) * 16);                
        
        //setup numbers
        self.write_command( REG_TXDESCHEAD, 0);
        self. write_command( REG_TXDESCTAIL,0);
        self.tx_cur = 0;
        self.write_command(REG_TCTRL,  TCTL_EN | TCTL_PSP);

        Ok(())
                
    }  

    /// Send a packet, called by a function higher in the network stack
    /// p_data is address of tranmit buffer, must be pointing to contiguous memory
    pub fn send_packet(&mut self, p_data: usize, p_len: u16) -> Result<(), &'static str> {
        //debug!("Value of tx descriptor address_translated: {:x}",ptr);
        let t_ptr = translate_v2p(p_data);
        let ptr;
        match t_ptr{
            Some(_x) => ptr = t_ptr.unwrap(),
            None => return Err("e1000:send_packet Couldn't translate address for tx buffer"),
        } 
        //let ptr = (translate_v2p(p_data)).unwrap();

        //debug!("Value of tx descriptor address_translated: {:x}",ptr);
        self.tx_descs[self.tx_cur as usize].addr = ptr as u64;
        self.tx_descs[self.tx_cur as usize].length = p_len;
        self.tx_descs[self.tx_cur as usize].cmd = (CMD_EOP | CMD_IFCS | CMD_RPS | CMD_RS ) as u8; //(1<<0)|(1<<1)|(1<<3)
        self.tx_descs[self.tx_cur as usize].status = 0;

        let old_cur: u8 = self.tx_cur as u8;
        self.tx_cur = (self.tx_cur + 1) % (E1000_NUM_TX_DESC as u16);
        debug!("THD {}",self.read_command(REG_TXDESCHEAD));
        debug!("TDT!{}",self.read_command(REG_TXDESCTAIL));
        self. write_command(REG_TXDESCTAIL, self.tx_cur as u32);   
        debug!("THD {}",self.read_command(REG_TXDESCHEAD));
        debug!("TDT!{}",self.read_command(REG_TXDESCTAIL));
        // debug!("post-write, tx_descs[{}] = {:?}", old_cur, self.tx_descs[old_cur as usize]);
        // debug!("Value of tx descriptor address: {:x}",self.tx_descs[old_cur as usize].addr);
        debug!("Waiting for packet to send!");
        
        while (self.tx_descs[old_cur as usize].status & 0xF) == 0 {
            //debug!("THD {}",self.read_command(REG_TXDESCHEAD));
            //debug!("status register: {}",self.tx_descs[old_cur as usize].status);
        }  //bit 0 should be set when done
        debug!("Packet is sent!");  
        Ok(())
    }        
        
    /// Enable Interrupts 
    pub fn enable_interrupts(&self) {
        //self.write_command(REG_IMASK ,0x1F6DC);
        //self.write_command(REG_IMASK ,0xff & !4);
        self.write_command(REG_IMASK ,0x84);//RXT and LSC
        self.read_command(0xc0); // clear all interrupts
    }      

    pub fn check_state(&self){
        debug!("REG_CTRL {:x}",self.read_command(REG_CTRL));
        debug!("REG_RCTRL {:x}",self.read_command(REG_RCTRL));
        debug!("REG_TCTRL {:x}",self.read_command(REG_TCTRL));

        // debug!("addr {:x}",self.tx_descs[0].addr);// as *const u64);
        // debug!("length {:?}",&self.tx_descs[0].length);// as *const u16);
        debug!("cso {:?}",&self.tx_descs[0].cso);// as *const u8);
        debug!("cmd {:?}",&self.tx_descs[0].cmd);// as *const u8);
        debug!("status {:?}",&self.tx_descs[0].status);// as *const u8);
        debug!("css {:?}",&self.tx_descs[0].css);// as *const u8);
        // debug!("special {:?}",&self.tx_descs[0].special);// as *const u16);
    }

    /// Handle a packet reception.
    pub fn handle_receive(&mut self) {
        //print status of all packets until EoP
        while(self.rx_descs[self.rx_cur as usize].status&0xF) !=0{
            debug!("rx desc status {}",self.rx_descs[self.rx_cur as usize].status);
            self.rx_descs[self.rx_cur as usize].status = 0;
            let old_cur = self.rx_cur as u32;
            self.rx_cur = (self.rx_cur + 1) % E1000_NUM_RX_DESC as u16;
            self.write_command(REG_RXDESCTAIL, old_cur );
        }
    }  


    /// Poll for recieved messages
    /// Can be used as analternative to interrupts
    pub fn rx_poll(&mut self){
        //detect if a packet has beem received
        if (self.rx_descs[self.rx_cur as usize].status&0xF) != 0 {                        
            self.handle_receive();
        }   
    }

    //Interrupt handler for nic
    pub fn handle_interrupt(&mut self) {
        debug!("e1000 handler");
        let status = self.read_command(0xc0); //reads status and clears interrupt
        if (status & 0x04 ) == 0x04 //link status change
        {
            debug!("Interrupt:link status changed");
            self.start_link();
        }
        else if (status & 0x80 ) == 0x80 { //receiver timer interrupt
            debug!("Interrupt: RXT");
            self.handle_receive();
        }
        else {
            debug!("Unhandled interrupt!");
        }
        self.read_command(0xc0); //clear interrupt
    }

}

/// initialize the nic
pub fn init_nic(e1000_pci: &mut PciDevice) -> Result<(), &'static str>{
    let mut e1000_nc = Nic {
        bar_type : 0, 
        io_base : 0,   
        mem_base : 0, 
        interrupt_num: None,
        eeprom_exists: false,
        mac: [0,0,0,0,0,0],
        rx_descs: Vec::with_capacity(E1000_NUM_RX_DESC),
        tx_descs: Vec::with_capacity(E1000_NUM_TX_DESC),
        rx_cur: 0,
        tx_cur: 0,
        rx_buf_addr: [0;E1000_NUM_RX_DESC],
        nic_dma_allocator: DmaAllocator{
                                start: 0,
                                end: 0,
                                current: 0,
                        }
    };
    e1000_nc.init(e1000_pci);
    e1000_nc.mem_map(e1000_pci)?;
    e1000_nc.mem_map_dma()?;
    e1000_nc.detect_eeprom();
    e1000_nc.read_mac_addr();
    e1000_nc.start_link();
    e1000_nc.clear_multicast();
    e1000_nc.clear_statistics();
    e1000_nc.enable_interrupts();
    e1000_nc.rx_init()?;
    e1000_nc.tx_init()?;

    Ok(())
}

