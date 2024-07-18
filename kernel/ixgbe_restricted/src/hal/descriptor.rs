use volatile::Volatile;
use zerocopy::FromBytes;
use core::ops::Deref;

/// The TinyNF driver uses a combined receive and transmit descriptor ring, 
/// so interpretation of the bits depends on if we are accessing on receive or transmit.
#[derive(FromBytes)]
#[repr(C)]
pub struct LegacyDescriptor {
    /// The starting physical address of the packet buffer
    phys_addr:  Volatile<u64>,
    metadata: Volatile<u64>
}

impl LegacyDescriptor {
    #[inline(always)]
    pub fn rx_metadata(&self) -> (DescriptorDone, PacketLength) {
        const DESC_DONE: u64 = 1 << 32;
        let metadata = self.metadata.read();
        (DescriptorDone((metadata & DESC_DONE) == DESC_DONE), PacketLength(metadata as u16 & 0xFFFF))
    }

    #[inline(always)]
    pub fn send(&mut self, packet_length: PacketLength, rs_bit: u64) {
        const CMD_EOP:  u64 = 1 << 24;
        const CMD_IFCS: u64 = 1 << 25;
        const CMD_RS:   u64 = 1 << 27;
        self.metadata.write(*packet_length as u64 | rs_bit | CMD_IFCS | CMD_EOP);
    }
}


#[derive(Debug, Copy, Clone)]
pub struct PacketLength(u16);
impl PacketLength {
    pub fn zero() -> PacketLength {
        PacketLength(0)
    }
}
impl Deref for PacketLength {
    type Target = u16;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

 #[derive(Debug, Copy, Clone)]
pub struct DescriptorDone(bool);
impl Deref for DescriptorDone {
    type Target = bool;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
