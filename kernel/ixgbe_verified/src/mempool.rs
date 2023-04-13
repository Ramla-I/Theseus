use core::ops::{Deref, DerefMut};

use memory::{PhysicalAddress, BorrowedSliceMappedPages, Mutable, MappedPages, create_contiguous_mapping, VirtualAddress};
use packet_buffers::*;
use alloc::vec::Vec;
use zerocopy::FromBytes;

use crate::allocator::NIC_MAPPING_FLAGS_CACHED;

// #[derive(FromBytes)]
// pub struct Buffer {
//     index: usize,
// }
// const_assert_eq!(core::mem::size_of::<Buffer>(), 8);


// pub type PacketBuffer = Buffer;

// pub type PacketBuffer = OwnedPtr<Buffer>;
// const_assert_eq!(core::mem::size_of::<PacketBuffer>(), 8);

// pub struct OwnedPtr<T>(Unique<T>);

// impl Deref for PacketBuffer {
//     type Target = Buffer;

//     #[inline(always)]
//     fn deref(&self) -> &Self::Target {
//         unsafe { self.0.as_ref()}
//     }
// }

// impl DerefMut for PacketBuffer {

//     #[inline(always)]
//     fn deref_mut(&mut self) -> &mut Self::Target {
//         unsafe { self.0.as_mut()}
//     }
// }

// pub struct Mempool {
//     buffers: Vec<PacketBuffer>,
//     buffers_paddr: PhysicalAddress,
//     buffers_vaddr: VirtualAddress,
//     buffers_mp: MappedPages,
// }
// const_assert_eq!(core::mem::size_of::<MappedPages>(), 40);
// const_assert_eq!(core::mem::size_of::<Mempool>(), 80);


// impl Mempool {
//     pub fn new(num_buffers: usize) -> Result<Mempool, &'static str> {
//         let (buffers_mp, buffers_paddr) = create_contiguous_mapping(num_buffers * core::mem::size_of::<EthernetFrame>(), NIC_MAPPING_FLAGS_CACHED)?;
//         // let (pool_mp, pool_paddr) = create_contiguous_mapping(num_buffers * core::mem::size_of::<Buffer>(), NIC_MAPPING_FLAGS_CACHED)?;

//         // let pool_start_addr = pool_mp.start_address();
//         let mut buffers = Vec::with_capacity(num_buffers);

//         for i in 0..num_buffers {
//             // let mut pool_elem = Unique::new((pool_start_addr + i * core::mem::size_of::<Buffer>()).value() as *mut Buffer).ok_or("failed to create a Unique")?;
//             // unsafe{
//                 let buffer = Buffer {
//                     vaddr: (buffers_mp.start_address() + (i * core::mem::size_of::<EthernetFrame>())).value(),
//                     // buffer: Unique::new((buffers_mp.start_address() + (i * core::mem::size_of::<EthernetFrame>())).value() as *mut EthernetFrame).ok_or("pointer passed to Unique was invalid")?,
//                     // length: 0,
//                     // paddr: buffers_paddr + (i * core::mem::size_of::<EthernetFrame>())
//                 };
//             // }
//             buffers.push(buffer);
//         }

//         Ok(Mempool{ buffers, buffers_paddr, buffers_vaddr: buffers_mp.start_address(), buffers_mp})
//     }

//     pub fn get_buffer(&mut self) -> Option<PacketBuffer> {
//         self.buffers.pop()
//     }

//     pub fn return_buffer(&mut self, buffer: PacketBuffer) {
//         self.buffers.push(buffer);
//     }

//     #[inline(always)]
//     pub fn paddr(&self, buffer: &Buffer) -> PhysicalAddress {
//         self.buffers_paddr + (buffer.vaddr - self.buffers_vaddr.value()) / core::mem::size_of::<EthernetFrame>()
//     }
// }


// impl Deref for Mempool {
//     type Target = Vec<PacketBuffer>;

//     #[inline(always)]
//     fn deref(&self) -> &Self::Target {
//         &self.buffers
//     }
// }

// impl DerefMut for Mempool {
//     #[inline(always)]
//     fn deref_mut(&mut self) -> &mut Self::Target {
//         &mut self.buffers
//     }
// }

pub struct Mempool {
    buffers: Vec<PacketBufferS>,
}