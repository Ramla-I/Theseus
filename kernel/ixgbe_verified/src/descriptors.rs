use memory::PhysicalAddress;
use volatile::Volatile;
use zerocopy::FromBytes;

// Transmit descriptor bits
/// Tx Command: End of Packet
pub const TX_CMD_EOP:                      u8 = 1 << 0;     
/// Tx Command: Insert MAC FCS
pub const TX_CMD_IFCS:                     u8 = 1 << 1;     
/// Tx Command: Insert Checksum
pub const TX_CMD_IC:                       u8 = 1 << 2;     
/// Tx Command: Report Status
pub const TX_CMD_RS:                       u8 = 1 << 3;     
/// Tx Command: Report Packet Sent
pub const TX_CMD_RPS:                      u8 = 1 << 4;     
/// Tx Command: Descriptor Extension (Advanced format)
pub const TX_CMD_DEXT:                     u8 = 1 << 5;  
/// Tx Command: VLAN Packet Enable
pub const TX_CMD_VLE:                      u8 = 1 << 6;     
/// Tx Command: Interrupt Delay Enable
pub const TX_CMD_IDE:                      u8 = 1 << 7;     
/// Tx Status: descriptor Done
pub const TX_STATUS_DD:                    u8 = 1 << 0;
/// Tx Descriptor Type: advanced
pub const TX_DTYP_ADV:                     u8 = 0x3 << 4;
/// Tx Descriptor paylen shift
/// The paylen is located at bit 46 in the upper 64 bits of the advanced Tx descriptor.
/// Since we have divided the upper 64 bits into 4 parts (u16,u8,u8,u32),
/// the paylen is then located at bit 14 of the upper 32 bits of the descriptor.
pub const TX_PAYLEN_SHIFT:                 u8 = 46 - 32; //(actual offset - offset of variable) 

/// Advanced Transmit Descriptor used by the `ixgbe` NIC driver.
///
/// # Two usage modes
/// It has 2 modes: Read and Write Back, both of which use the whole 128 bits. 
/// There is one transmit descriptor per transmit buffer; it can be converted between these 2 modes.
///
/// Read contains the addresses that the driver writes.
/// Write Back contains information the hardware writes on receiving a packet.
///
/// More information can be found in the 82599 datasheet.
#[derive(FromBytes)]
#[repr(C)]
pub struct AdvancedTxDescriptor {
    /// Starting physical address of the receive buffer for the packet.
    pub packet_buffer_address:  Volatile<u64>,
    /// Length of data buffer
    pub data_len: Volatile<u16>,
    /// A multi-part field:
    /// * `dtyp`: Descriptor Type, occupies bits `[7:4]`,
    /// * `mac`: options to apply LinkSec and time stamp, occupies bits `[3:2]`.
    pub dtyp_mac_rsv : Volatile<u8>,
    /// Command bits
    pub dcmd:  Volatile<u8>,
    /// A multi-part field:
    /// * `paylen`: the size in bytes of the data buffer in host memory.
    ///   not including the fields that the hardware adds), occupies bits `[31:14]`.
    /// * `popts`: options to offload checksum calculation, occupies bits `[13:8]`.
    /// * `sta`: status of the descriptor (whether it's in use or not), occupies bits `[3:0]`.
    pub paylen_popts_cc_idx_sta: Volatile<u32>,
}

impl AdvancedTxDescriptor {
    pub fn init(&mut self) {
        self.packet_buffer_address.write(0);
        self.paylen_popts_cc_idx_sta.write(0);
        self.dcmd.write(0);
        self.dtyp_mac_rsv.write(0);
        self.data_len.write(0);
    }

    pub fn send(&mut self, transmit_buffer_addr: PhysicalAddress, transmit_buffer_length: u16) {
        self.packet_buffer_address.write(transmit_buffer_addr.value() as u64);
        self.data_len.write(transmit_buffer_length);
        self.dtyp_mac_rsv.write(TX_DTYP_ADV);
        self.paylen_popts_cc_idx_sta.write((transmit_buffer_length as u32) << TX_PAYLEN_SHIFT);
        self.dcmd.write(TX_CMD_DEXT | TX_CMD_RS | TX_CMD_IFCS | TX_CMD_EOP);
    }

    pub fn wait_for_packet_tx(&self) {
        while (self.paylen_popts_cc_idx_sta.read() as u8 & TX_STATUS_DD) == 0 {
            // error!("tx desc status: {:#X}", self.desc.read());
        } 
    }
}

