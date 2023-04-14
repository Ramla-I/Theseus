use core::num;
use core::ptr::{NonNull, Unique};
use core::ops::{Deref, DerefMut};

use memory::{PhysicalAddress,BorrowedSliceMappedPages, Mutable, MappedPages, create_contiguous_mapping};
use packet_buffers::{EthernetFrame};
use alloc::vec::Vec;
use zerocopy::FromBytes;

use crate::allocator::NIC_MAPPING_FLAGS_CACHED;

// #[derive(FromBytes)]
pub struct Buffer {
    pub(crate) buffer: Unique<EthernetFrame>,
    pub length: u16,
    pub(crate) paddr: PhysicalAddress
}

const_assert_eq!(core::mem::size_of::<Buffer>(), 24);

impl Deref for Buffer {
    type Target = EthernetFrame;

    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        unsafe { self.buffer.as_ref()}
    }
}

impl DerefMut for Buffer {

    #[inline(always)]
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { self.buffer.as_mut()}
    }
}

pub type PacketBuffer = OwnedPtr<Buffer>;
const_assert_eq!(core::mem::size_of::<PacketBuffer>(), 8);

pub struct OwnedPtr<T>(Unique<T>);

impl Deref for PacketBuffer {
    type Target = Buffer;

    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        unsafe { self.0.as_ref()}
    }
}

impl DerefMut for PacketBuffer {

    #[inline(always)]
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { self.0.as_mut()}
    }
}

pub struct Mempool {
    buffers: Vec<OwnedPtr<Buffer>>,
    buffers_paddr: PhysicalAddress,
    buffers_mp: MappedPages,
    pool_mp: MappedPages,
}
const_assert_eq!(core::mem::size_of::<MappedPages>(), 40);
const_assert_eq!(core::mem::size_of::<Mempool>(), 112);


impl Mempool {
    pub fn new(num_buffers: usize) -> Result<Mempool, &'static str> {
        let (buffers_mp, buffers_paddr) = create_contiguous_mapping(num_buffers * core::mem::size_of::<EthernetFrame>(), NIC_MAPPING_FLAGS_CACHED)?;
        let (pool_mp, pool_paddr) = create_contiguous_mapping(num_buffers * core::mem::size_of::<Buffer>(), NIC_MAPPING_FLAGS_CACHED)?;

        let pool_start_addr = pool_mp.start_address();
        let mut buffers = Vec::with_capacity(num_buffers);

        for i in 0..num_buffers {
            let mut pool_elem = Unique::new((pool_start_addr + i * core::mem::size_of::<Buffer>()).value() as *mut Buffer).ok_or("failed to create a Unique")?;
            unsafe{
                *pool_elem.as_mut() = Buffer {
                    buffer: Unique::new((buffers_mp.start_address() + (i * core::mem::size_of::<EthernetFrame>())).value() as *mut EthernetFrame).ok_or("pointer passed to Unique was invalid")?,
                    length: 0,
                    paddr: buffers_paddr + (i * core::mem::size_of::<EthernetFrame>())
                };
            }
            buffers.push(OwnedPtr(pool_elem));
        }

        Ok(Mempool{ buffers, buffers_paddr, buffers_mp, pool_mp})
    }

    pub fn get_buffer(&mut self) -> Option<PacketBuffer> {
        self.buffers.pop()
    }

    pub fn return_buffer(&mut self, buffer: PacketBuffer) {
        self.buffers.push(buffer);
    }
}


impl Deref for Mempool {
    type Target = Vec<PacketBuffer>;

    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.buffers
    }
}

impl DerefMut for Mempool {
    #[inline(always)]
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buffers
    }
}