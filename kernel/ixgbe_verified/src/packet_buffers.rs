
/// A buffer that stores a packet to be transmitted or received through the NIC
/// and is guaranteed to be contiguous in physical memory. 
/// Auto-dereferences into a `MappedPages` object that represents its underlying memory. 
/// 
/// This is a combined packet buffer without any drop handler to make it easy to use buffers in a network function loop.
/// Network functions receive a packet, process it, and then transmit it.
pub struct PacketBuffer {
    pub mp: MappedPages,
    pub phys_addr: PhysicalAddress,
    pub length: u16,
    pub buffer: *mut EthernetFrame //look into ouborous or pinned. should be able to store reference to MappedPages
}

const MAX_ETHERNET_FRAME_IN_BYTES: usize = 1518;
impl PacketBuffer {
    /// Creates a new `PacketBuffer` with the specified size in bytes.
    /// The size is a `u16` because that is the maximum size of a NIC buffer. 
    pub fn new(size_in_bytes: u16) -> Result<PacketBuffer, &'static str> {
        let (mut mp, starting_phys_addr) = create_contiguous_mapping(
            size_in_bytes as usize,
            EntryFlags::WRITABLE | EntryFlags::NO_CACHE | EntryFlags::NO_EXECUTE,
        )?;
        
        let buffer = mp.as_type_mut::<EthernetFrame>(0)? as *mut EthernetFrame;

        Ok(PacketBuffer {
            mp: mp,
            phys_addr: starting_phys_addr,
            length: size_in_bytes,
            buffer
        })
    }
}

impl Deref for PacketBuffer {
    type Target = MappedPages;
    fn deref(&self) -> &MappedPages {
        &self.mp
    }
}
impl DerefMut for PacketBuffer {
    fn deref_mut(&mut self) -> &mut MappedPages {
        &mut self.mp
    }
}

/// A struct that makes it easy to access different fields of an ethernet frame
/// Note: Tried to use const generics for the payload, but it fails when trying to derive FromBytes, works otherwise.
#[derive(FromBytes)]
pub struct EthernetFrame {
    pub dest_addr:  [u8; 6],
    pub src_addr:   [u8; 6],
    pub length:     u16,
    pub payload:    [u8; 1504]
}