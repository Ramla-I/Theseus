//! A trusted wrapper over the verified Chunk.
//! Needed because verification fails on a trusted chunk that stores a FrameRange or RangeInclusive<Frame>, 
//! but succeeds with RangeInclusive<usize>.
//! 
//! We should be able to remove this module and work directly with the verified crate in the foreseeable future.
//! All this model should do is amke sure that the start and end of the stored `frames` is equal to the start and end of the `verified_chunk`

use alloc::collections::btree_map::Range;
use kernel_config::memory::PAGE_SIZE;
use memory_structs::{FrameRange, Frame, PhysicalAddress};
use range_inclusive::RangeInclusive;
use crate::{MemoryRegionType, AllocatedFrames, MIN_FRAME, MAX_FRAME};
use core::{borrow::Borrow, cmp::{Ordering, min, max}, fmt, ops::{Deref, DerefMut}};
use spin::{Once, Mutex};
use trusted_chunk::{
    trusted_chunk::*,
    linked_list::List,
    static_array::StaticArray,
};

pub(crate) static HEAP_INIT: Once<bool> = Once::new();
static CHUNK_ARRAY: Mutex<StaticArray> = Mutex::new(StaticArray::new());
static CHUNK_LIST: Mutex<List> = Mutex::new(List::new());
// pub(crate) static INTO_VERIFIED_CHUNK_FUNC: Once<fn(RangeInclusive<usize>) -> TrustedChunk> = Once::new();

#[derive(Debug, Eq)]
pub struct Chunk {
    /// The type of this memory chunk, e.g., whether it's in a free or reserved region.
    typ: MemoryRegionType,
    /// The Frames covered by this chunk, an inclusive range. 
    frames: FrameRange,
    /// The actual verified chunk
    verified_chunk: TrustedChunk
}

assert_not_impl_any!(Chunk: DerefMut, Clone);

impl Chunk {
    pub(crate) fn new(typ: MemoryRegionType, frames: FrameRange) -> Result<Self, &'static str> {
        if frames.is_empty() {
            return Err("Sanity Check: empty chunks should be created with the empty() function");
        }

        let verified_chunk = if HEAP_INIT.is_completed() {
            TrustedChunk::new(frames.to_range_inclusive(), &mut CHUNK_LIST.lock()).map_err(|_| "Failed to create a verified chunk due to an overlap")?
        } else {
            TrustedChunk::new_pre_heap(frames.to_range_inclusive(), &mut CHUNK_ARRAY.lock())
                .map(|(chunk, _)| chunk)
                .map_err(|chunk_error|{
                    match chunk_error {
                        ChunkCreationError::Overlap(idx) => "Failed to create a verified chunk due to an overlap",
                        ChunkCreationError::NoSpace => "Before the heap is initialized, requested more chunks than there is space for (64)"
                    }
                })?
        };

        Ok(Chunk {
            typ,
            frames,
            verified_chunk
        })
    }

    pub(crate) fn from_trusted_chunk(verified_chunk: TrustedChunk, frames: FrameRange, typ: MemoryRegionType) -> Chunk {
        Chunk {
            typ,
            frames,
            verified_chunk
        }
    }

    // /// This is a specializes way to create a TrustedChunk
    // pub(crate) fn from_unmapped_pte(typ: MemoryRegionType, frames: FrameRange) -> Result<Chunk, &'static str> {
    //     INTO_VERIFIED_CHUNK_FUNC.get()
    //         .ok_or("into verified chunk function wasn't initialized")
    //         .map(|function| {
    //             let verified_chunk = function(frames.to_range_inclusive());
    //             Chunk { typ, frames, verified_chunk }
    //         })
    // }

    pub(crate) fn frames(&self) -> FrameRange {
        self.frames.clone()
    }

    pub(crate) fn typ(&self) -> MemoryRegionType {
        self.typ
    }

    pub(crate) fn as_allocated_frames(self) -> AllocatedFrames {
        AllocatedFrames {
            frames: self,
        }
    }

    /// Returns a new `Chunk` with an empty range of frames. 
    pub(crate) const fn empty() -> Chunk {
        Chunk {
            typ: MemoryRegionType::Unknown,
            frames: FrameRange::empty(),
            verified_chunk: TrustedChunk::empty()
        }
    }

    pub(crate) fn merge(&mut self, mut other: Chunk) -> Result<(), Chunk> {
        if self.is_empty() || other.is_empty() {
            return Err(other);
        }

        // take out the TrustedChunk from other
        let other_verified_chunk = core::mem::replace(&mut other.verified_chunk, TrustedChunk::empty());
        
        // merged the other TrustedChunk with self
        // failure here means that the chunks cannot be merged
        self.verified_chunk.merge(other_verified_chunk)
            .map_err(|vchunk| {
                let _ = core::mem::replace(&mut other.verified_chunk, vchunk);
                other
            })?;

        // use the newly merged TrustedChunk to update the frame range
        self.frames = into_frame_range(&self.verified_chunk.frames());

        Ok(())
    }

    /// An inner function that breaks up the given chunk into multiple smaller chunks.
    /// 
    /// Returns a tuple of three chunks:
    /// 1. The `Chunk` containing the requested range of frames starting at `start_frame`.
    /// 2. The range of frames in the `self` that came before the beginning of the requested frame range.
    /// 3. The range of frames in the `self` that came after the end of the requested frame range.
    pub fn split(
        mut self,
        start_frame: Frame,
        num_frames: usize,
    ) -> (Chunk, Option<Chunk>, Option<Chunk>) {
        if self.is_empty() {
            return (self, None, None);
        }

        // take out the TrustedChunk
        let verified_chunk = core::mem::replace(&mut self.verified_chunk, TrustedChunk::empty());

        let (before, new_allocation, after) = match verified_chunk.split(start_frame.number(), num_frames) {
            Ok(x) => x,
            Err(vchunk) => {
                let _ = core::mem::replace(&mut self.verified_chunk, vchunk);
                return (self, None, None);
            }
        };

        (Chunk {
            typ: self.typ,
            frames: into_frame_range(&new_allocation.frames()),
            verified_chunk: new_allocation
        },
        before.and_then(|vchunk| 
            Some(Chunk{
                typ: self.typ,
                frames: into_frame_range(&vchunk.frames()),
                verified_chunk: vchunk
            })
        ), 
        after.and_then(|vchunk| 
            Some(Chunk{
                typ: self.typ,
                frames: into_frame_range(&vchunk.frames()),
                verified_chunk: vchunk
            })
        ))
    }

    pub fn split_at(mut self, at_frame: Frame) -> Result<(Chunk, Chunk), Chunk> {
        if self.is_empty() {
            return Err(self);
        }
        let typ = self.typ;

        // take out the TrustedChunk
        let verified_chunk = core::mem::replace(&mut self.verified_chunk, TrustedChunk::empty());

        let (first, second) = verified_chunk.split_at(at_frame.number())
            .map_err(|vchunk| {
                let _ = core::mem::replace(&mut self.verified_chunk, vchunk);
                self
            })?;

        Ok((Chunk {
            typ,
            frames: into_frame_range(&first.frames()),
            verified_chunk: first
        },
        Chunk {
            typ,
            frames: into_frame_range(&second.frames()),
            verified_chunk: second
        }))
    }
}

impl Deref for Chunk {
    type Target = FrameRange;
    fn deref(&self) -> &FrameRange {
        &self.frames
    }
}
impl Ord for Chunk {
    fn cmp(&self, other: &Self) -> Ordering {
        self.frames.start().cmp(other.frames.start())
    }
}
impl PartialOrd for Chunk {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl PartialEq for Chunk {
    fn eq(&self, other: &Self) -> bool {
        self.frames.start() == other.frames.start()
    }
}
impl Borrow<Frame> for &'_ Chunk {
    fn borrow(&self) -> &Frame {
        self.frames.start()
    }
}


fn into_frame_range(frames: &RangeInclusive<usize>) -> FrameRange {
    let start = FrameNum{ frame: *frames.start() }.into_frame()
        .expect("Verified chunk start was not a valid frame");
    
    let end = FrameNum{ frame: *frames.end() }.into_frame()
        .expect("Verified chunk end was not a valid frame");
    
    FrameRange::new(start, end)
}

struct FrameNum {
    frame: usize
}

impl FrameNum {
    fn into_frame(&self) -> Option<Frame> {
        PhysicalAddress::new(self.frame * PAGE_SIZE)
            .and_then(|addr| Some(Frame::containing_address(addr)))
    }
}