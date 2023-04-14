use volatile::Volatile;
use zerocopy::FromBytes;

/// This struct is a Legacy Transmit Descriptor. 
/// It's the descriptor type used in older Intel NICs and the E1000 driver.
#[derive(FromBytes)]
#[repr(C)]
pub struct LegacyDescriptor {
    /// The starting physical address of the transmit buffer
    pub(crate) phys_addr:  Volatile<u64>,
    other: Volatile<u64>
}

#[derive(Debug, Copy, Clone)]
pub struct PacketLength(u16);

impl PacketLength {
    pub fn zero() -> PacketLength {
        PacketLength(0)
    }
}
impl LegacyDescriptor {
    #[inline(always)]
    /// Returns (descriptor done bit, packet length)
    pub fn rx_metadata(&self) -> (bool, PacketLength) {
        let metadata = self.other.read();
        ((metadata & (1 << 32)) == (1 << 32), PacketLength(metadata as u16 & 0xFFFF))
    }

    #[inline(always)]
    pub fn send(&mut self, packet_length: PacketLength, rs_bit: u64) {
        self.other.write(packet_length.0 as u64 | rs_bit | (1 << (24 + 1)) | (1 << 24));
    }
}