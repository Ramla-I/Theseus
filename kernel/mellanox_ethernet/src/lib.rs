 //! Note: Mellanox manual refers to the NIC as HCA.

 #![no_std]
 #![feature(slice_pattern)]
 #![feature(core_intrinsics)]

#[macro_use]extern crate log;
extern crate memory;
extern crate volatile;
extern crate bit_field;
extern crate zerocopy;
#[macro_use] extern crate alloc;
#[macro_use] extern crate static_assertions;
extern crate owning_ref;
extern crate byteorder;
extern crate nic_initialization;
extern crate kernel_config;
extern crate libm;

use core::{num, slice::SlicePattern};

use alloc::vec::Vec;
use memory::{PhysicalAddress, PhysicalMemoryRegion, MappedPages, create_contiguous_mapping};
use volatile::{Volatile, ReadOnly, WriteOnly};
use bit_field::BitField;
use zerocopy::*;
// {FromBytes, AsBytes, Unaligned};
use byteorder::BigEndian;
use owning_ref:: BoxRefMut;
use nic_initialization::NIC_MAPPING_FLAGS;
use kernel_config::memory::PAGE_SIZE;
use core::fmt;

// Taken from HCA BAR 
const MAX_CMND_QUEUE_ENTRIES: usize = 64;
#[derive(FromBytes)]
#[repr(C,packed)]
/// The fields are stored in BE ordering, so that's why every 32 bits field seems to be opposite to the diagram in the manual
pub struct InitializationSegment {
    fw_rev_minor:               ReadOnly<U16<BigEndian>>,
    fw_rev_major:               ReadOnly<U16<BigEndian>>,
    cmd_interface_rev:          ReadOnly<U16<BigEndian>>,
    fw_rev_subminor:            ReadOnly<U16<BigEndian>>,
    _padding1:                  [u8; 8],
    cmdq_phy_addr_high:         Volatile<U32<BigEndian>>,
    cmdq_phy_addr_low:          Volatile<U32<BigEndian>>,
    command_doorbell_vector:    Volatile<U32<BigEndian>>,
    _padding2:                  [u8; 390],
    initializing_state:         ReadOnly<U32<BigEndian>>,
    health_buffer:              Volatile<[u8; 64]>,
    no_dram_nic_offset:         ReadOnly<U32<BigEndian>>,
    _padding3:                  [u8; 3516],
    internal_timer_h:           ReadOnly<U32<BigEndian>>,
    internal_timer_l:           ReadOnly<U32<BigEndian>>,
    _padding4:                  [u8; 8],
    health_counter:             ReadOnly<U32<BigEndian>>,
    _padding5:                  [u8; 44],
    real_time:                  ReadOnly<U64<BigEndian>>,
    _padding6:                  [u8; 12228],
}

// const_assert_eq!(core::mem::size_of::<InitializationSegment>(), 16400);

impl InitializationSegment {
    pub fn num_cmdq_entries(&self) -> u8 {
        let log = (self.cmdq_phy_addr_low.read().get() >> 4) & 0x0F;
        2_u8.pow(log)
    }

    pub fn cmdq_entry_stride(&self) -> u8 {
        let val = self.cmdq_phy_addr_low.read().get() & 0x0F;
        2_u8.pow(val)
    }
    
    pub fn set_physical_address_of_cmdq(&mut self, pa: PhysicalAddress) -> Result<(), &'static str> {
        if pa.value() & 0xFFF != 0 {
            return Err("cmdq physical address lower 12 bits must be zero.");
        }

        self.cmdq_phy_addr_high.write(U32::new((pa.value() >> 32) as u32));
        let val = self.cmdq_phy_addr_low.read().get() & 0xFFF;
        self.cmdq_phy_addr_low.write(U32::new(pa.value() as u32 | val));
        Ok(())
    }

    pub fn device_is_initializing(&self) -> bool {
        self.initializing_state.read().get().get_bit(31)
    }

    pub fn post_command(&mut self, command_bit: usize) {
        let val = self.command_doorbell_vector.read().get();
        self.command_doorbell_vector.write(U32::new(val | (1 << command_bit)));
    }

    pub fn print(&self) {
        trace!("{}.{}.{}, {}", self.fw_rev_major.read().get(), self.fw_rev_minor.read().get(), self.fw_rev_subminor.read().get(), self.cmd_interface_rev.read().get());
        trace!("{:#X} {:#X}", self.cmdq_phy_addr_high.read().get(), self.cmdq_phy_addr_low.read().get());
        trace!("{:#X}", self.command_doorbell_vector.read().get());
        trace!("{:#X}", self.initializing_state.read().get());
    }
}

pub enum InitializingState {
    NotAllowed = 0,
    WaitingPermetion = 1, // Is this a typo?
    WaitingResources = 2,
    Abort = 3
}

#[derive(FromBytes, Default)]
#[repr(C)]
pub struct CommandQueueEntry {
    type_of_transport:              Volatile<U32<BigEndian>>,
    input_length:                   Volatile<U32<BigEndian>>,
    input_mailbox_pointer_h:        Volatile<U32<BigEndian>>,
    input_mailbox_pointer_l:        Volatile<U32<BigEndian>>,
    command_input_opcode:           Volatile<U32<BigEndian>>,
    command_input_op_mod:           Volatile<U32<BigEndian>>,
    command_input_inline_data_0:    Volatile<U32<BigEndian>>,
    command_input_inline_data_1:    Volatile<U32<BigEndian>>,
    command_output_status:          Volatile<U32<BigEndian>>,
    command_output_syndrome:        Volatile<U32<BigEndian>>,
    command_output_inline_data_0:   Volatile<U32<BigEndian>>,
    command_output_inline_data_1:   Volatile<U32<BigEndian>>,
    output_mailbox_pointer_h:       Volatile<U32<BigEndian>>,
    output_mailbox_pointer_l:       Volatile<U32<BigEndian>>,
    output_length:                  Volatile<U32<BigEndian>>,
    token_signature_status_own:     Volatile<U32<BigEndian>>
}


const_assert_eq!(core::mem::size_of::<CommandQueueEntry>(), 64);

pub enum CommandTransportType {
    PCIe = 0x7 << 24
}

impl fmt::Debug for CommandQueueEntry {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "CQE\n")?;
        write!(f, "CQE:type of transport: {:#X} \n", self.type_of_transport.read().get())?;
        write!(f, "CQE:input length: {} \n", self.input_length.read().get())?;
        write!(f, "CQE:input_mailbox_ptr_h: {:#X} \n", self.input_mailbox_pointer_h.read().get())?;
        write!(f, "CQE:input_mailbox_ptr_l: {:#X} \n", self.input_mailbox_pointer_l.read().get())?;
        write!(f, "CQE:command_input_opcode: {:#X} \n", self.command_input_opcode.read().get())?;
        write!(f, "CQE:command_input_op_mod: {:#X} \n", self.command_input_op_mod.read().get())?;
        write!(f, "CQE:command_input_inline_data_0: {:#X} \n", self.command_input_inline_data_0.read().get())?;
        write!(f, "CQE:command_input_inline_data_1: {:#X} \n", self.command_input_inline_data_1.read().get())?;
        write!(f, "CQE:command_output_status: {:#X} \n", self.command_output_status.read().get())?;
        write!(f, "CQE:command_output_syndrome: {:#X} \n", self.command_output_syndrome.read().get())?;
        write!(f, "CQE:command_output_inline_data_0: {:#X} \n", self.command_output_inline_data_0.read().get())?;
        write!(f, "CQE:command_output_inline_data_1: {:#X} \n", self.command_output_inline_data_1.read().get())?;
        write!(f, "CQE:output_mailbox_pointer_h: {:#X} \n", self.output_mailbox_pointer_h.read().get())?;
        write!(f, "CQE:output_mailbox_pointer_l: {:#X} \n", self.output_mailbox_pointer_l.read().get())?;
        write!(f, "CQE:output_length: {} \n", self.output_length.read().get())?;
        write!(f, "CQE:token_signature_status_own: {:#X} \n", self.token_signature_status_own.read().get())
    }
}

impl CommandQueueEntry {
    fn set_type_of_transport(&mut self, transport: CommandTransportType) {
        self.type_of_transport.write(U32::new(CommandTransportType::PCIe as u32));
    }

    fn set_input_length_in_bytes(&mut self, length: u32) {
        self.input_length.write(U32::new(length))
    }

    fn set_input_mailbox_pointer(&mut self, pa: PhysicalAddress) -> Result<(), &'static str> {
        error!("Don't use this function yet! We don't fill out the mailbox fields");
        
        if pa.value() & 0x1FF != 0 {
            return Err("input mailbox pointer physical address lower 9 bits must be zero.");
        }

        self.input_mailbox_pointer_h.write(U32::new((pa.value() >> 32) as u32));
        let val = self.input_mailbox_pointer_l.read().get() & 0x1FF;
        self.input_mailbox_pointer_l.write(U32::new(pa.value() as u32 | val));
        Ok(())
    }

    // right now assume only 8 bytes of commands are passed
    fn set_input_inline_data(&mut self, opcode: CommandOpcode, op_mod: Option<u16>, command0: Option<u32>, command1: Option<u32>) {
        self.command_input_opcode.write(U32::new((opcode as u32) << 16));
        self.command_input_op_mod.write(U32::new(op_mod.unwrap_or(0) as u32));
        self.command_input_inline_data_0.write(U32::new(command0.unwrap_or(0)));
        self.command_input_inline_data_1.write(U32::new(command1.unwrap_or(0)));

    }

    fn get_command_opcode(&self) -> CommandOpcode {
        match self.command_input_opcode.read().get() >> 16 {
            0x100 => {CommandOpcode::QueryHcaCap},
            0x101 => {CommandOpcode::QueryAdapter}, 
            0x102 => {CommandOpcode::InitHca}, 
            0x103 => {CommandOpcode::TeardownHca}, 
            0x104 => {CommandOpcode::EnableHca}, 
            0x105 => {CommandOpcode::DisableHca}, 
            0x107 => {CommandOpcode::QueryPages}, 
            0x108 => {CommandOpcode::ManagePages}, 
            0x10A => {CommandOpcode::QueryIssi}, 
            0x10B => {CommandOpcode::SetIssi}, 
            _ => {CommandOpcode::Unknown}
        }
    }

    // right now assume only 8 bytes of commands are passed
    fn get_output_inline_data(&self) -> (u8, u32, u32, u32) {
        (
            (self.command_output_status.read().get() >> 24) as u8,
            self.command_output_syndrome.read().get(),
            self.command_output_inline_data_0.read().get(),
            self.command_output_inline_data_1.read().get()
        )
    }

    fn set_output_mailbox_pointer(&mut self, mp: &mut MappedPages, pa: PhysicalAddress) -> Result<(), &'static str> {
        error!("Don't use this function yet! We don't fill out the mailbox fields");

        if pa.value() & 0x1FF != 0 {
            return Err("output mailbox pointer physical address lower 9 bits must be zero.");
        }

        self.output_mailbox_pointer_h.write(U32::new((pa.value() >> 32) as u32));
        let val = self.output_mailbox_pointer_l.read().get() & 0x1FF;
        self.output_mailbox_pointer_l.write(U32::new(pa.value() as u32 | val));
        Ok(())
    }
    
    fn set_output_length_in_bytes(&mut self, length: u32) {
        self.output_length.write(U32::new(length));
    }

    fn get_output_length_in_bytes(&self) -> u32 {
        self.output_length.read().get()
    }

    fn set_token(&mut self, token: u8) {
        let val = self.token_signature_status_own.read().get();
        self.token_signature_status_own.write(U32::new(val | ((token as u32) << 24)));
    }

    fn get_token(&self) -> u8 {
        (self.token_signature_status_own.read().get() >> 24) as u8
    }

    fn get_signature(&self) -> u8 {
        (self.token_signature_status_own.read().get() >> 16) as u8
    }

    pub fn get_delivery_status(&self) -> CommandDeliveryStatus {
        let status = self.token_signature_status_own.read().get() & 0xFE;
        if status == 0 {
            CommandDeliveryStatus::Success
        } else if status == 1 {
            CommandDeliveryStatus::SignatureErr
        } else if status == 2 {
            CommandDeliveryStatus::TokenErr
        } else if status == 3 {
            CommandDeliveryStatus::BadBlockNumber
        } else if status == 4 {
            CommandDeliveryStatus::BadOutputPointer
        } else if status == 5 {
            CommandDeliveryStatus::BadInputPointer
        } else if status == 6 {
            CommandDeliveryStatus::InternalErr
        } else if status == 7 {
            CommandDeliveryStatus::InputLenErr
        } else if status == 8 {
            CommandDeliveryStatus::OutputLenErr
        } else if status == 9 {
            CommandDeliveryStatus::ReservedNotZero
        } else if status == 10 {
            CommandDeliveryStatus::BadCommandType
        } else {
            CommandDeliveryStatus::Unknown
        }
    }

    pub fn change_ownership_to_hw(&mut self) {
        let ownership = self.token_signature_status_own.read().get() | 0x1;
        self.token_signature_status_own.write(U32::new(ownership));
    }

    pub fn owned_by_hw(&self) -> bool {
        self.token_signature_status_own.read().get().get_bit(0)
    }

    pub fn get_return_status(&self) -> CommandReturnStatus {
        let (status, _syndrome, _, _) = self.get_output_inline_data();

        if status == 0x0 {
            CommandReturnStatus::OK
        } else if status == 0x1 {
            CommandReturnStatus::InternalError
        } else if status == 0x2 {
            CommandReturnStatus::BadOp
        } else if status == 0x3 {
            CommandReturnStatus::BadParam
        } else if status == 0x4 {
            CommandReturnStatus::BadSysState
        } else if status == 0x5 {
            CommandReturnStatus::BadResource
        } else if status == 0x6 {
            CommandReturnStatus::ResourceBusy
        } else if status == 0x8 {
            CommandReturnStatus::ExceedLim
        } else if status == 0x9 {
            CommandReturnStatus::BadResState
        } else if status == 0xA {
            CommandReturnStatus::BadIndex
        } else if status == 0xF {
            CommandReturnStatus::NoResources
        } else if status == 0x50 {
            CommandReturnStatus::BadInputLen
        } else if status == 0x51 {
            CommandReturnStatus::BadOutputLen
        } else if status == 0x10 {
            CommandReturnStatus::BadResourceState
        } else if status == 0x30 {
            CommandReturnStatus::BadPkt
        } else if status == 0x40 {
            CommandReturnStatus::BadSize
        } else {
            CommandReturnStatus::Unknown
        }
    }
}

#[derive(FromBytes)]
#[repr(C)]
struct CommandInterfaceMailbox {
    mailbox_data:           Volatile<[u8; 512]>,
    _padding:               ReadOnly<[u8; 48]>,
    next_pointer_h:         Volatile<U32<BigEndian>>,
    next_pointer_l:         Volatile<U32<BigEndian>>,
    block_number:           Volatile<U32<BigEndian>>,
    token_ctrl_signature:   Volatile<U32<BigEndian>>
}
const_assert_eq!(core::mem::size_of::<CommandInterfaceMailbox>(), 576);

impl CommandInterfaceMailbox {
    fn clear_all_fields(&mut self) {
        self. mailbox_data.write([0;512]);
        self.next_pointer_h.write(U32::new(0));
        self.next_pointer_l.write(U32::new(0));
        self.block_number.write(U32::new(0));
        self.token_ctrl_signature.write(U32::new(0));
    }
}

impl fmt::Debug for CommandInterfaceMailbox {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        for i in 0..(512/4) {
            let data = self.mailbox_data.read();
            write!(f, "mailbox data: {:#X} {:#X} {:#X} {:#X} \n", data[i*4], data[i*4+1], data[i*4+2], data[i*4 +3])?;
        }
        
        // write!(f, "padding: {}", self._padding.read())?;
        writeln!(f, "next pointer h: {:#X} \n", self.next_pointer_h.read().get())?;
        writeln!(f, "next pointer l: {:#X} \n", self.next_pointer_l.read().get())?;
        writeln!(f, "block number: {:#X} \n", self.block_number.read().get())?;
        writeln!(f, "token ctrl signature: {:#X} \n", self.token_ctrl_signature.read().get())
    }
}

#[derive(Debug)]
pub enum CommandDeliveryStatus {
    Success = 0,
    SignatureErr = 1,
    TokenErr = 2,
    BadBlockNumber = 3,
    BadOutputPointer = 4,
    BadInputPointer = 5,
    InternalErr = 6,
    InputLenErr = 7,
    OutputLenErr = 8,
    ReservedNotZero = 9,
    BadCommandType = 10, //Should this be 10 or 16??
    Unknown,
}

#[derive(PartialEq, Debug)]
pub enum CommandOpcode {
    QueryHcaCap = 0x100,
    QueryAdapter = 0x101,
    InitHca = 0x102,
    TeardownHca = 0x103,
    EnableHca = 0x104,
    DisableHca = 0x105,
    QueryPages = 0x107,
    ManagePages = 0x108,
    QueryIssi = 0x10A,
    SetIssi = 0x10B,
    QuerySpecialContexts = 0x203,
    CreateEq = 0x301,
    CreateCq = 0x400,
    QueryVportState = 0x751,
    QueryNicVportContext = 0x754,
    AllocUar = 0x802,
    AllocPd = 0x800,
    AllocTransportDomain = 0x816,
    CreateTis = 0x912,
    CreateSq = 0x904,
    ModifySq = 0x905,
    CreateRq = 0x908,
    ModifyRq = 0x909,
    Unknown
}


#[derive(Debug)]
pub enum CommandReturnStatus {
    OK = 0x00,
    InternalError = 0x01,
    BadOp = 0x02,
    BadParam = 0x03,
    BadSysState = 0x04,
    BadResource = 0x05,
    ResourceBusy = 0x06,
    ExceedLim = 0x08,
    BadResState = 0x09,
    BadIndex = 0x0A,
    NoResources = 0x0F,
    BadInputLen = 0x50,
    BadOutputLen = 0x51,
    BadResourceState = 0x10,
    BadPkt = 0x30,
    BadSize = 0x40,
    Unknown
}
/// Section 8.24.1
/// A buffer of fixed-size entries that is used to pass commands to the HCA.
/// The number of enties and the entry stride is retrieved from the initialization segment of the HCA BAR.
/// It resides in a physically contiguous 4 KiB memory chunk.
#[repr(C)]
pub struct CommandQueue {
    entries: BoxRefMut<MappedPages, [CommandQueueEntry]>,
    available_entries: [bool; MAX_CMND_QUEUE_ENTRIES],
    token: u8, //taken from snabb, my assumption is that it is a random number that needs to be different for every command
    mailbox_buffers_input: Vec<Vec<(MappedPages, PhysicalAddress)>>, // A page, physical address of the page, and if it's in use
    mailbox_buffers_output: Vec<Vec<(MappedPages, PhysicalAddress)>> // A page, physical address of the page, and if it's in use
}

const MAILBOX_SIZE_IN_BYTES: usize = 576;
const MAILBOX_DATA_SIZE_IN_BYTES: usize = 512;

const SIZE_PADDR_IN_BYTES: usize = 8; // each physical address takes 8 bytes in the mailbox
const SIZE_PADDR_H_IN_BYTES: usize = 4; // each physical address takes 8 bytes in the mailbox
const SIZE_PADDR_L_IN_BYTES: usize = 4; // each physical address takes 8 bytes in the mailbox
pub enum ManagePagesOpmod {
    AllocationFail = 0,
    AllocationSuccess = 1,
    HcaReturnPages = 2
}

pub enum QueryPagesOpmod {
    BootPages = 1,
    InitPages = 2,
    RegularPages = 3
}

#[derive(PartialEq)]
enum MailboxType {
    Input,
    Output
}

enum QueryVportStateOpMod {
    VnicVport = 0,
    EswVport = 1,
    Uplink = 2
}

impl CommandQueue {

    pub fn create(entries: BoxRefMut<MappedPages, [CommandQueueEntry]>, num_cmdq_entries: usize) -> Result<CommandQueue, &'static str> {
        let mut available_entries = [false; MAX_CMND_QUEUE_ENTRIES];
        for i in 0..num_cmdq_entries { available_entries[i] = true; }

        // allocate one page to be the mailbox buffer per entry
        let mut mailbox_buffers_input = Vec::with_capacity(num_cmdq_entries);
        let mut mailbox_buffers_output = Vec::with_capacity(num_cmdq_entries);
        for _ in 0..num_cmdq_entries {
            let (mailbox_mp, mailbox_pa) = create_contiguous_mapping(PAGE_SIZE, NIC_MAPPING_FLAGS)?;
            mailbox_buffers_input.push(vec!((mailbox_mp, mailbox_pa)));

            let (mailbox_mp, mailbox_pa) = create_contiguous_mapping(PAGE_SIZE, NIC_MAPPING_FLAGS)?;
            mailbox_buffers_output.push(vec!((mailbox_mp, mailbox_pa)));
        }

        Ok(CommandQueue{ entries, available_entries, token: 0xAA, mailbox_buffers_input, mailbox_buffers_output })
    }

    fn find_free_command_entry(&self) -> Option<usize> {
        self.available_entries.iter().position(|&x| x == true)
    }

    pub fn create_command(
        &mut self, opcode: CommandOpcode, 
        op_mod: Option<u16>, 
        allocated_pages: Option<Vec<PhysicalAddress>>,
        uar: Option<u32>,
        log_queue_size: Option<u8>
    ) -> Result<usize, &'static str> 
    {
        let entry_num = self.find_free_command_entry().ok_or("No command entry available")?; 

        let mut cmdq_entry = CommandQueueEntry::default();
        cmdq_entry.set_type_of_transport(CommandTransportType::PCIe);
        cmdq_entry.set_token(self.token);
        cmdq_entry.change_ownership_to_hw();

        match opcode {
            CommandOpcode::EnableHca => {
                cmdq_entry.set_input_length_in_bytes(12);
                cmdq_entry.set_output_length_in_bytes(8);
                cmdq_entry.set_input_inline_data(opcode, op_mod, None, None);
            }
            CommandOpcode::QueryIssi => {
                warn!("running query issi with smaller output length, may be an error");
                cmdq_entry.set_input_length_in_bytes(8);
                cmdq_entry.set_output_length_in_bytes(112);
                cmdq_entry.set_input_inline_data(opcode, op_mod, None, None);
                
                self.init_query_issi_output_mailbox_buffers(entry_num)?;
                self.set_mailbox_pointer_in_cmd_entry(&mut cmdq_entry, entry_num, MailboxType::Output);

            }
            CommandOpcode::SetIssi => {
                warn!("setting to 1 by default, could be wrong");
                cmdq_entry.set_input_length_in_bytes(12);
                cmdq_entry.set_output_length_in_bytes(8);
                cmdq_entry.set_input_inline_data(opcode, op_mod, Some(1), None);
            }
            CommandOpcode::InitHca => {
                cmdq_entry.set_input_length_in_bytes(12);
                cmdq_entry.set_output_length_in_bytes(8);
                cmdq_entry.set_input_inline_data(opcode, op_mod, None, None);
            }
            CommandOpcode::QuerySpecialContexts => {
                cmdq_entry.set_input_length_in_bytes(8);
                cmdq_entry.set_output_length_in_bytes(16);
                cmdq_entry.set_input_inline_data(opcode, op_mod, None, None);
            }
            CommandOpcode::QueryPages => {
                cmdq_entry.set_input_length_in_bytes(12);
                cmdq_entry.set_output_length_in_bytes(16);
                cmdq_entry.set_input_inline_data(opcode, op_mod, None, None);
            }
            CommandOpcode::ManagePages => {
                let pages_pa = allocated_pages.ok_or("No pages were passed to the manage pages command")?;
                cmdq_entry.set_input_length_in_bytes(0x10 + pages_pa.len() as u32 *8); // taken from snabb
                cmdq_entry.set_output_length_in_bytes(16);
                cmdq_entry.set_input_inline_data(
                    opcode, 
                    op_mod, 
                    None, 
                    Some(pages_pa.len() as u32)
                );

                self.init_manage_pages_input_mailbox_buffers(entry_num, pages_pa)?;
                self.set_mailbox_pointer_in_cmd_entry(&mut cmdq_entry, entry_num, MailboxType::Input);
            }
            CommandOpcode::AllocUar => {
                cmdq_entry.set_input_length_in_bytes(8);
                cmdq_entry.set_output_length_in_bytes(12);
                cmdq_entry.set_input_inline_data(opcode, op_mod, None, None);
            },
            CommandOpcode::CreateEq => {
                let pages_pa = allocated_pages.ok_or("No pages were passed to the create EQ command")?;

                cmdq_entry.set_input_length_in_bytes((0x110 + pages_pa.len()*8) as u32);
                cmdq_entry.set_output_length_in_bytes(12);
                cmdq_entry.set_input_inline_data(opcode, op_mod, None, None);
                
                self.create_page_request_event_queue(
                    entry_num, 
                    pages_pa,
                    uar.ok_or("uar not specified in EQ creation")?,
                    log_queue_size.ok_or("queue size not specified in EQ creation")?
                )?;

                self.set_mailbox_pointer_in_cmd_entry(&mut cmdq_entry, entry_num, MailboxType::Input);
            },
            CommandOpcode::QueryVportState => { // only accesses your own vport
                cmdq_entry.set_input_length_in_bytes(12);
                cmdq_entry.set_output_length_in_bytes(16);
                cmdq_entry.set_input_inline_data(opcode, op_mod, None, None);
            },
            CommandOpcode::QueryNicVportContext => { // only accesses your own vport
                cmdq_entry.set_input_length_in_bytes(16);
                cmdq_entry.set_output_length_in_bytes(16 + 0x100);
                cmdq_entry.set_input_inline_data(opcode, op_mod, None, None);

                self.init_nic_vport_context_output_mailbox_buffers(entry_num)?;
                self.set_mailbox_pointer_in_cmd_entry(&mut cmdq_entry, entry_num, MailboxType::Output);
            },
            CommandOpcode::AllocPd => { 
                cmdq_entry.set_input_length_in_bytes(8);
                cmdq_entry.set_output_length_in_bytes(12);
                cmdq_entry.set_input_inline_data(opcode, op_mod, None, None);
            },
            CommandOpcode::AllocTransportDomain => { 
                cmdq_entry.set_input_length_in_bytes(8);
                cmdq_entry.set_output_length_in_bytes(12);
                cmdq_entry.set_input_inline_data(opcode, op_mod, None, None);
            },
            CommandOpcode::QuerySpecialContexts => { 
                cmdq_entry.set_input_length_in_bytes(8);
                cmdq_entry.set_output_length_in_bytes(12);
                cmdq_entry.set_input_inline_data(opcode, op_mod, None, None);
            },
            _=> {
                debug!("unimplemented opcode");
            }
        }        
        core::mem::swap(&mut cmdq_entry, &mut self.entries[entry_num]);
        warn!("{:?}", &mut self.entries[entry_num]);
        self.token = self.token.wrapping_add(1);
        self.available_entries[entry_num] = false;
        Ok(entry_num)
    }

    fn set_mailbox_pointer_in_cmd_entry(&mut self, cmdq_entry: &mut CommandQueueEntry, entry_num: usize, mailbox_type: MailboxType) {
        if mailbox_type == MailboxType::Input {
            let mailbox_ptr = self.mailbox_buffers_input[entry_num][0].1.value();
            warn!("mailbox ptr in command entry: {:#X}", mailbox_ptr);
            cmdq_entry.input_mailbox_pointer_h.write(U32::new((mailbox_ptr >> 32) as u32));
            cmdq_entry.input_mailbox_pointer_l.write(U32::new((mailbox_ptr & 0xFFFF_FFFF) as u32));
        } else {
            let mailbox_ptr = self.mailbox_buffers_output[entry_num][0].1.value();
            cmdq_entry.output_mailbox_pointer_h.write(U32::new((mailbox_ptr >> 32) as u32));
            cmdq_entry.output_mailbox_pointer_l.write(U32::new((mailbox_ptr & 0xFFFF_FFFF) as u32));
        }
    }

    fn init_query_issi_output_mailbox_buffers(&mut self, entry_num: usize) -> Result<(), &'static str> {
        const NUM_MAILBOXES_QUERY_ISSI: usize = 1;
        self.initialize_mailboxes(entry_num, NUM_MAILBOXES_QUERY_ISSI, MailboxType::Output)?;
        Ok(())
    }

    fn init_manage_pages_input_mailbox_buffers(&mut self, entry_num: usize, mut pages: Vec<PhysicalAddress>) -> Result<(), &'static str> {
        
        let num_mailboxes = libm::ceilf((pages.len() * SIZE_PADDR_IN_BYTES) as f32 / MAILBOX_DATA_SIZE_IN_BYTES as f32) as usize;
        self.initialize_mailboxes(entry_num, num_mailboxes, MailboxType::Input)?;

        let mailbox_pages = &mut self.mailbox_buffers_input[entry_num];
        let paddr_per_mailbox = MAILBOX_DATA_SIZE_IN_BYTES / SIZE_PADDR_IN_BYTES;

        for block_num in 0..num_mailboxes {  
            let (mb_page, mb_page_starting_addr) = &mut mailbox_pages[block_num];

            let mailbox = mb_page.as_type_mut::<CommandInterfaceMailbox>(0)?;

            let mut data = [0; 512];

            for page in 0..paddr_per_mailbox {
                let paddr = pages.pop();
                match paddr {
                    Some(paddr) => {
                        Self::write_paddr_in_mailbox_data(page*SIZE_PADDR_IN_BYTES, paddr, &mut data);
                    },
                    None => { 
                        trace!("breaking out of loop on mailbox: {} and paddr: {}", block_num, page);
                        break; 
                    }
                }
            }

            mailbox.mailbox_data.write(data);
            debug!("Mailbox {}", block_num);
            debug!("{:?}", mailbox);
        }

        Ok(())
    }

    pub fn wait_for_command_completion(&mut self, entry_num: usize) -> (CommandDeliveryStatus, CommandReturnStatus) {
        while self.entries[entry_num].owned_by_hw() {}
        self.available_entries[entry_num] = true;
        debug!("{:?}", self.entries[entry_num]);
        (self.entries[entry_num].get_delivery_status(), self.entries[entry_num].get_return_status())
    }

    fn check_command_output_validity(&self, entry_num: usize, cmd_opcode: CommandOpcode) -> Result<(), &'static str> {
        if self.entries[entry_num].owned_by_hw() {
            error!("the command hasn't completed yet!");
            return Err("the command hasn't completed yet!");
        }

        if self.entries[entry_num].get_command_opcode() != cmd_opcode {
            error!("Incorrect Command!: {:?}", self.entries[entry_num].get_command_opcode());
            return Err("Incorrect Command!");
        }

        Ok(())
    }

    pub fn get_query_issi_command_output(&self, entry_num: usize) -> Result<(u16, u8), &'static str> {
        self.check_command_output_validity(entry_num, CommandOpcode::QueryIssi)?;

        // This offset is not correct! need to check!
        const DATA_OFFSET_IN_MAILBOX: usize = 0x20 - 0x10;
        let mailbox = &self.mailbox_buffers_output[entry_num][0].0;
        debug!("Query ISSI mailbox {:?}", mailbox.as_type::<CommandInterfaceMailbox>(0)?);
        let supported_issi = (mailbox.as_type::<u32>(DATA_OFFSET_IN_MAILBOX)?).to_le();

        let (_status, _syndrome, current_issi , _command1) = self.entries[entry_num].get_output_inline_data();
        Ok((current_issi as u16, supported_issi as u8))
    }

    pub fn get_query_pages_command_output(&self, entry_num: usize) -> Result<u32, &'static str> {
        self.check_command_output_validity(entry_num, CommandOpcode::QueryPages)?;

        let (_status, _syndrome, _function_id, num_pages) = self.entries[entry_num].get_output_inline_data();
        Ok(num_pages)
    }

    pub fn get_uar(&self, entry_num: usize) -> Result<u32, &'static str> {
        debug!("{:?}",self.entries[entry_num]);
        self.check_command_output_validity(entry_num, CommandOpcode::AllocUar)?;
        let (_status, _syndrome, uar, _reserved) = self.entries[entry_num].get_output_inline_data();
        Ok(uar & 0xFF_FFFF)
    }

    fn create_page_request_event_queue(&mut self, entry_num: usize, mut pages: Vec<PhysicalAddress>, uar: u32, log_eq_size: u8) -> Result<(), &'static str> {
        
        let size_of_mailbox_data = (0x110 - 0x10) + SIZE_PADDR_IN_BYTES * pages.len();

        let num_mailboxes = libm::ceilf(size_of_mailbox_data as f32 / MAILBOX_DATA_SIZE_IN_BYTES as f32) as usize;
        self.initialize_mailboxes(entry_num, num_mailboxes, MailboxType::Input)?;

        let mailbox_pages = &mut self.mailbox_buffers_input[entry_num];
        
        for block_num in 0..num_mailboxes {
            let (mb_page, _mb_page_starting_addr) = &mut mailbox_pages[block_num];
            
            if block_num == 0 {
                // initialize the event queue context
                let eq_context = mb_page.as_type_mut::<EventQueueContext>(0)?;
                eq_context.init(uar, log_eq_size);

                // initialize the bitmask. this function only activates the page request event
                let bitmask_offset_in_mailbox  = 0x58 - 0x10;
                let eq_bitmask = mb_page.as_type_mut::<u64>(bitmask_offset_in_mailbox)?;
                const PAGE_REQUEST_BIT: u64 = 1 << 0xB;
                *eq_bitmask = PAGE_REQUEST_BIT;

                // Now use the remainder of the mailbox for page entries
                let eq_pa_offset = 0x110 - 0x10;
                let data = mb_page.as_type_mut::<[u8;256]>(eq_pa_offset)?;
                let pages_in_mailbox_0 = (MAILBOX_DATA_SIZE_IN_BYTES - eq_pa_offset) / SIZE_PADDR_IN_BYTES;
 
                for page in 0..pages_in_mailbox_0 {
                    let paddr = pages.pop();
                    match paddr {
                        Some(paddr) => {
                            Self::write_paddr_in_mailbox_data(page*SIZE_PADDR_IN_BYTES, paddr, data);
                        },
                        None => { 
                            trace!("breaking out of loop on mailbox: {} and page: {}", block_num, page);
                            break; 
                        }
                    }
                }

                debug!("Mailbox {}", block_num);
                debug!("{:?}", mb_page.as_type_mut::<CommandInterfaceMailbox>(0)?);
                    
            } else {
                let mailbox = mb_page.as_type_mut::<CommandInterfaceMailbox>(0)?;
                let paddr_per_mailbox = MAILBOX_DATA_SIZE_IN_BYTES / SIZE_PADDR_IN_BYTES;
                let mut data = [0; 512];

                for page in 0..paddr_per_mailbox {
                    let paddr = pages.pop();
                    match paddr {
                        Some(paddr) => {
                            Self::write_paddr_in_mailbox_data(page*SIZE_PADDR_IN_BYTES, paddr, &mut data);
                        },
                        None => { 
                            trace!("breaking out of loop on mailbox: {} and paddr: {}", block_num, page);
                            break; 
                        }
                    }
                }

                mailbox.mailbox_data.write(data);

                debug!("Mailbox {}", block_num);
                debug!("{:?}", mailbox);
            }
        }
        Ok(())
    }

    pub fn get_eq_number(&self, entry_num: usize) -> Result<u8, &'static str> {
        self.check_command_output_validity(entry_num, CommandOpcode::CreateEq)?;

        let (_status, _syndrome, eq_number, _reserved) = self.entries[entry_num].get_output_inline_data();
        Ok(eq_number as u8)
    }

    fn get_vport_state(&self, entry_num: usize)-> Result<(u16, u8, u8), &'static str> {
        self.check_command_output_validity(entry_num, CommandOpcode::QueryVportState)?;

        let (_status, _syndrome, _reserved, output) = self.entries[entry_num].get_output_inline_data();
        let max_tx_speed = (output >> 16) as u16;
        let admin_state = (output as u8 & 0xF0) >> 4;
        let state = (output as u8) & 0xF;

        Ok((max_tx_speed, admin_state, state))      
    }

    fn init_nic_vport_context_output_mailbox_buffers(&mut self, entry_num: usize) -> Result<(), &'static str> {
        const NUM_MAILBOXES_NIC_VPORT_CONTEXT: usize = 1;
        self.initialize_mailboxes(entry_num, NUM_MAILBOXES_NIC_VPORT_CONTEXT, MailboxType::Output)?;
        Ok(())
    }

    fn get_vport_context(&self, entry_num: usize)-> Result<(u16, [u8;6]), &'static str> {
        self.check_command_output_validity(entry_num, CommandOpcode::QueryNicVportContext)?;

        let context = self.mailbox_buffers_output[entry_num][0].0.as_type::<NicVportContext>(0)?;
        let mac_addr_h = context.permanent_address_h.read().get();
        let mac_addr_l = context.permanent_address_l.read().get();

        let mut mac_addr = [0; 6]; 
        mac_addr[0] =  mac_addr_l as u8;
        mac_addr[1] = (mac_addr_l >> 8) as u8;
        mac_addr[2] = (mac_addr_l >> 16) as u8;
        mac_addr[3] = (mac_addr_l >> 24) as u8;
        mac_addr[4] =  mac_addr_h as u8;
        mac_addr[5] = (mac_addr_h >> 8) as u8;


        Ok((context.mtu.read().get() as u16, mac_addr))
    }

    pub fn get_protection_domain(&self, entry_num: usize) -> Result<u32, &'static str> {
        self.check_command_output_validity(entry_num, CommandOpcode::AllocPd)?;
        let (_status, _syndrome, pd, _reserved) = self.entries[entry_num].get_output_inline_data();

        Ok(pd & 0xFF_FFFF)
    }

    pub fn get_transport_domain(&self, entry_num: usize) -> Result<u32, &'static str> {
        self.check_command_output_validity(entry_num, CommandOpcode::AllocTransportDomain)?;
        let (_status, _syndrome, td, _reserved) = self.entries[entry_num].get_output_inline_data();

        Ok(td & 0xFF_FFFF)
    }

    pub fn get_reserved_lkey(&self, entry_num: usize) -> Result<u32, &'static str> {
        self.check_command_output_validity(entry_num, CommandOpcode::QuerySpecialContexts)?;
        let (_status, _syndrome, _dump_fill_mkey, resd_lkey) = self.entries[entry_num].get_output_inline_data();

        Ok(resd_lkey)
    }

    fn write_paddr_in_mailbox_data(start_offset: usize, paddr: PhysicalAddress, data: &mut [u8]) {
        let start_offset_h = start_offset;
        let end_offset_h = start_offset_h + SIZE_PADDR_H_IN_BYTES;
        let addr = (paddr.value() >> 32) as u32;
        data[start_offset_h..end_offset_h].copy_from_slice(&addr.to_be_bytes());

        let start_offset_l = end_offset_h;
        let end_offset_l = start_offset_l + SIZE_PADDR_L_IN_BYTES;
        let addr = (paddr.value() & 0xFFFF_FFFF) as u32;
        data[start_offset_l..end_offset_l].copy_from_slice(&addr.to_be_bytes());
    }

    /// Clears all mailboxes, then sets the token, block number and next address fields for all mailboxes
    fn initialize_mailboxes(&mut self, entry_num: usize, num_mailboxes: usize, mailbox_type: MailboxType) -> Result<(), &'static str> {
        let mailbox_pages = if mailbox_type == MailboxType::Input{
            &mut self.mailbox_buffers_input[entry_num]
        } else {
            &mut self.mailbox_buffers_output[entry_num]
        };

        // Adding extra mailbox pages
        let available_mailboxes = mailbox_pages.len(); 
        if num_mailboxes > available_mailboxes {
            let num_mailbox_pages_required = num_mailboxes - available_mailboxes;
            trace!("Adding {} mailbox pages", num_mailbox_pages_required);
            
            for _ in 0..num_mailbox_pages_required {
                mailbox_pages.push(create_contiguous_mapping(PAGE_SIZE, NIC_MAPPING_FLAGS)?);
            }
        }

        for block_num in 0..num_mailboxes {
            // record the next page address to set pointer for last mailbox (can't have two borrows in the same loop)
            let next_mb_addr = if block_num < (num_mailboxes - 1) {
                mailbox_pages[block_num + 1].1.value()
            } else {
                0
            };

            let (mb_page, mb_page_starting_addr) = &mut mailbox_pages[block_num];

            trace!("Initializing mb: {}", block_num);
            let mailbox = mb_page.as_type_mut::<CommandInterfaceMailbox>(0)?;
            mailbox.clear_all_fields();

            mailbox.block_number.write(U32::new(block_num as u32));
            mailbox.token_ctrl_signature.write(U32::new((self.token as u32) << 16));

            mailbox.next_pointer_h.write(U32::new((next_mb_addr >> 32) as u32));
            mailbox.next_pointer_l.write(U32::new((next_mb_addr & 0xFFFF_FFFF) as u32));
               
        }

        Ok(())
    }

    fn create_completion_queue() {

    }
}

#[derive(FromBytes)]
#[repr(C)]
struct CompletionQueueContext {
    status:                 Volatile<U32<BigEndian>>,
    _padding1:              u32,
    page_offset:            Volatile<U32<BigEndian>>,
    uar_log_cq_size:        Volatile<U32<BigEndian>>,
    cq_max_count_period:    Volatile<U32<BigEndian>>,
    c_eqn:                  Volatile<U32<BigEndian>>,
    log_page_size:          Volatile<U32<BigEndian>>,
    _padding2:              u32,
    last_notified_index:    Volatile<U32<BigEndian>>,
    last_solicit_index:     Volatile<U32<BigEndian>>,
    consumer_counter:       Volatile<U32<BigEndian>>,
    producer_counter:       Volatile<U32<BigEndian>>,
    _padding3:              u64,
    dbr_addr:               Volatile<u64>,
}

const_assert_eq!(core::mem::size_of::<CompletionQueueContext>(), 64);

impl CompletionQueueContext {
    pub fn init(&mut self, uar_page: u32, log_cq_size: u8) {
    }
}

#[derive(FromBytes)]
#[repr(C)]
struct SendQueueContext {
    state:                              Volatile<U32<BigEndian>>,
    user_index:                         Volatile<U32<BigEndian>>,
    cqn:                                Volatile<U32<BigEndian>>,
    hairpin_peer_rq:                    Volatile<U32<BigEndian>>,
    _padding1:                          u64,
    packet_pacing_rate_limit_index:     Volatile<U32<BigEndian>>,
    tis_lst_sz:                         Volatile<U32<BigEndian>>,
    _padding2:                          u64,
    tis_num:                            Volatile<U32<BigEndian>>,
}

const_assert_eq!(core::mem::size_of::<SendQueueContext>(), 48);

impl SendQueueContext {
    pub fn init(&mut self, uar_page: u32, log_cq_size: u8) {
    }
}

#[derive(FromBytes)]
#[repr(C)]
struct WorkQueue {
    wq_type:                    Volatile<U32<BigEndian>>,
    page_offset_lwm:            Volatile<U32<BigEndian>>,
    pd:                         Volatile<U32<BigEndian>>,
    uar_page:                   Volatile<U32<BigEndian>>,
    dbr_addr_h:                 Volatile<U32<BigEndian>>,
    dbr_addr_l:                 Volatile<U32<BigEndian>>,
    hw_counter:                 Volatile<U32<BigEndian>>,
    sw_counter:                 Volatile<U32<BigEndian>>,
    log_wq_size_stride:         Volatile<U32<BigEndian>>,
    log_hairpin_num_packets:    Volatile<U32<BigEndian>>,
    _padding1:                  [u8; 152],
}

const_assert_eq!(core::mem::size_of::<WorkQueue>(), 192);

impl WorkQueue {
    pub fn init(&mut self, uar_page: u32, log_cq_size: u8) {
    }
}

#[derive(FromBytes)]
#[repr(C)]
struct EventQueueContext {
    status:             Volatile<U32<BigEndian>>,
    _padding1:          ReadOnly<u32>,
    page_offset:        Volatile<U32<BigEndian>>,
    uar_log_eq_size:    Volatile<U32<BigEndian>>,
    _padding2:          ReadOnly<u32>,
    intr:               Volatile<U32<BigEndian>>,
    log_pg_size:        Volatile<U32<BigEndian>>,
    _padding3:          ReadOnly<u64>,
    consumer_counter:   Volatile<U32<BigEndian>>,
    producer_counter:   Volatile<U32<BigEndian>>,
    _padding4:          ReadOnly<[u8;12]>,
}

const_assert_eq!(core::mem::size_of::<EventQueueContext>(), 64);

impl EventQueueContext {
    pub fn init(&mut self, uar_page: u32, log_eq_size: u8) {
        let uar = uar_page & 0xFF_FFFF;
        let size = ((log_eq_size & 0x1F) as u32) << 24;
        self.uar_log_eq_size.write(U32::new(uar | size));
        self.log_pg_size.write(U32::new(0));
    }
}

#[derive(FromBytes)]
#[repr(C)]
struct NicVportContext {
    data1:                  Volatile<U32<BigEndian>>,
    data2:                  Volatile<U32<BigEndian>>,
    data3:                  Volatile<U32<BigEndian>>,
    _padding0:              ReadOnly<[u8; 24]>,
    mtu:                    Volatile<U32<BigEndian>>,
    system_image_guid:      Volatile<U64<BigEndian>>,
    port_guid:              Volatile<U64<BigEndian>>,
    node_guid:              Volatile<U64<BigEndian>>,
    max_qp_retry:           Volatile<U32<BigEndian>>,
    _padding1:              ReadOnly<[u8; 36]>,
    qkey_violation_counter: Volatile<U32<BigEndian>>,
    _padding2:              ReadOnly<[u8; 132]>,
    list_info:              Volatile<U32<BigEndian>>,
    permanent_address_h:    Volatile<U32<BigEndian>>,
    permanent_address_l:    Volatile<U32<BigEndian>>,
    sw_network_metadata:    Volatile<U32<BigEndian>>,
}

const_assert_eq!(core::mem::size_of::<NicVportContext>(), 256);
