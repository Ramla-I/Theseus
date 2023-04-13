use core::ops::{Deref, DerefMut};
use memory::{PhysicalAddress, BorrowedSliceMappedPages, Mutable, MappedPages, create_contiguous_mapping, VirtualAddress};
use packet_buffers::EthernetFrame;
use alloc::vec::Vec;
use zerocopy::FromBytes;
use core::ptr::Unique;
use crate::allocator::NIC_MAPPING_FLAGS_CACHED;

#[derive(FromBytes)]
pub struct PacketBuffer(usize);
const_assert_eq!(core::mem::size_of::<PacketBuffer>(), 8);

struct BufferMetadata {
    frame: Unique<EthernetFrame>,
    paddr: PhysicalAddress,
    length: u16
}

impl Deref for BufferMetadata {
    type Target = EthernetFrame;

    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        unsafe { self.frame.as_ref()}
    }
}

impl DerefMut for BufferMetadata {

    #[inline(always)]
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { self.frame.as_mut()}
    }
}

pub struct Mempool {
    // buffers: BorrowedSliceMappedPages<EthernetFrame, Mutable>,
    pub buffer_indices: Vec<PacketBuffer>,
    buffers: Vec<BufferMetadata>,
    buffers_mp: MappedPages,
}
// const_assert_eq!(core::mem::size_of::<MappedPages>(), 40);
// const_assert_eq!(core::mem::size_of::<Mempool>(), 80);


impl Mempool {
    pub(crate) fn new(num_buffers: usize) -> Result<Mempool, &'static str> {
        let (buffers_mp, buffers_paddr) = create_contiguous_mapping(num_buffers * core::mem::size_of::<EthernetFrame>(), NIC_MAPPING_FLAGS_CACHED)?;
        let mut buffers = Vec::with_capacity(num_buffers);
        let mut buffer_indices = Vec::with_capacity(num_buffers);


        for i in 0..num_buffers {
            let buffer = BufferMetadata {
                frame: Unique::new((buffers_mp.start_address() + (i * core::mem::size_of::<EthernetFrame>())).value() as *mut EthernetFrame).ok_or("pointer passed to Unique was invalid")?,
                paddr: buffers_paddr + (i * core::mem::size_of::<EthernetFrame>()),
                length: 0,
            };
            buffers.push(buffer);
            buffer_indices.push(PacketBuffer(i));
        }

        Ok(Mempool{ buffer_indices, buffers, buffers_mp })
    }

    #[inline(always)]
    pub fn phys_addr(&self, buffer: &PacketBuffer) -> PhysicalAddress {
        self.buffers[buffer.0].paddr
    }

    #[inline(always)]
    pub fn buffer_metadata(&self, buffer: &PacketBuffer) -> (PhysicalAddress, u16) {
        (self.buffers[buffer.0].paddr, self.buffers[buffer.0].length)
    }

    #[inline(always)]
    pub fn set_length(&mut self, buffer: &PacketBuffer, length: u16) {
        self.buffers[buffer.0].length = length;
    }

    #[inline(always)]
    pub fn get_length(&mut self, buffer: &PacketBuffer) -> u16 {
        self.buffers[buffer.0].length
    }

    #[inline(always)]
    pub fn frame(&mut self, buffer: &PacketBuffer) -> &mut EthernetFrame {
        &mut self.buffers[buffer.0]
    }
}

impl Deref for Mempool {
    type Target = Vec<PacketBuffer>;

    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.buffer_indices
    }
}

impl DerefMut for Mempool {

    #[inline(always)]
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buffer_indices
    }
}