//! A memory pool with packet buffers.
//! The ethernet frames are allocated from contiguous memory
//! and the per-frame PacketBuffer is also allocated from contiguous memory.
//! So the PacketBuffer cannot derive FromBytes as it stores a pointer, which is why the Buffer<T>
//! doesn't require the FromBytes trait... it's a bit unsafe... may be we should mark it as such.
//!
//! To Do: Drop handler that returns buffers to their backing store
//! To Do: Ideally some id field that relates buffer to the mempool to make it easier to return.

use core::ops::DerefMut;

use memory::{create_contiguous_mapping, DMA_FLAGS, PhysicalAddress};
use crate::ethernet_frame::EthernetFrame;
// use zerocopy::FromBytes;
use prusti_memory_buffer::{Buffer, BufferBackingStore, create_buffers_from_mp};
use prusti_external_spec::vecdeque_wrapper::VecDequeWrapper;
use static_assertions::const_assert_eq;

pub struct PacketBuffer {
    pub(crate) frame: Buffer<EthernetFrame>,
    pub(crate) length: u16,
    pub(crate) paddr: PhysicalAddress
}

impl PacketBuffer {
    pub fn frame(&mut self) -> &mut EthernetFrame {
        self.frame.deref_mut()
    }

    pub fn length(&self) -> u16 {
        self.length
    }
}

pub type PktBuff = Buffer<PacketBuffer>; // Just for convenience, so we don't have to write Buffer<> everywhere
const_assert_eq!(core::mem::size_of::<PktBuff>(), 8); // A key to good performance is that this should not be >8 bytes as this is the value that's copied between driver and application.

pub struct Mempool {
    pub(crate) buffers: VecDequeWrapper<PktBuff>,
    frames_paddr: PhysicalAddress,
    pkt_buffers_paddr: PhysicalAddress,
    frames_mp: BufferBackingStore<EthernetFrame>,
    pkt_buffers_mp: BufferBackingStore<PacketBuffer>
}

// // const_assert_eq!(core::mem::size_of::<MappedPages>(), 40);
// // const_assert_eq!(core::mem::size_of::<Mempool>(), 112);


impl Mempool {
    pub fn new(num_buffers: usize) -> Result<Mempool, &'static str> {
        let (frames_mp, frames_paddr) = 
            create_contiguous_mapping(num_buffers as usize * core::mem::size_of::<EthernetFrame>(), DMA_FLAGS)?;
        let (pkt_buffers_mp, pkt_buffers_paddr) = 
            create_contiguous_mapping(num_buffers * core::mem::size_of::<PacketBuffer>(), DMA_FLAGS)?;

        let mut frames_backing_store: BufferBackingStore<EthernetFrame> = create_buffers_from_mp(frames_mp, num_buffers)
            .map_err(|(_mp, e)| e.into_str())?; // just drop the MappedPages
        let mut pkt_buffers_backing_store: BufferBackingStore<PacketBuffer> = create_buffers_from_mp(pkt_buffers_mp, num_buffers)
            .map_err(|(_mp, e)| e.into_str())?; // just drop the MappedPages

        // Update the packet buffer information
        let current_frame_paddr = frames_paddr;
        for i in 0..num_buffers {
            *pkt_buffers_backing_store.buffers.0[i] = PacketBuffer {
                frame: frames_backing_store.buffers.pop_front().unwrap(),
                length: 0,
                paddr: current_frame_paddr + i * core::mem::size_of::<EthernetFrame>()
            }
        }

        let buffers = core::mem::replace(&mut pkt_buffers_backing_store.buffers, VecDequeWrapper::new());
        
        // To Do: Remove this check
        assert!(frames_backing_store.buffers.len() == 0);
        assert!(pkt_buffers_backing_store.buffers.len() == 0);

        Ok(Mempool { 
            buffers, 
            frames_paddr,
            pkt_buffers_paddr,
            frames_mp: frames_backing_store,
            pkt_buffers_mp: pkt_buffers_backing_store
        })
    } 

    // pub fn get_buffer(&mut self) -> Option<PacketBuffer> {
    //     self.buffers.pop()
    // }

    // pub fn return_buffer(&mut self, buffer: PacketBuffer) {
    //     self.buffers.push(buffer);
    // }
}
