use memory_structs::{FrameRange, Frame};
use crate::{MemoryRegionType, AllocatedFrames, MIN_FRAME, MAX_FRAME};
use core::{borrow::Borrow, cmp::{Ordering, min, max}, fmt, ops::{Deref, DerefMut}};


/// A range of contiguous frames.
///
/// # Ordering and Equality
///
/// `Chunk` implements the `Ord` trait, and its total ordering is ONLY based on
/// its **starting** `Frame`. This is useful so we can store `Chunk`s in a sorted collection.
///
/// Similarly, `Chunk` implements equality traits, `Eq` and `PartialEq`,
/// both of which are also based ONLY on the **starting** `Frame` of the `Chunk`.
/// Thus, comparing two `Chunk`s with the `==` or `!=` operators may not work as expected.
/// since it ignores their actual range of frames.
#[derive(Debug, Eq)]
pub struct Chunk {
    /// The type of this memory chunk, e.g., whether it's in a free or reserved region.
    pub(crate)typ: MemoryRegionType,
    /// The Frames covered by this chunk, an inclusive range. 
    pub(crate)frames: FrameRange,
}
impl Chunk {
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
        }
    }

    pub(crate) fn merge(&mut self, other: Chunk) -> Result<(), Chunk> {
        if *self.start() == *other.end() + 1 {
            // `other` comes contiguously before `self`
            self.frames = FrameRange::new(*other.start(), *self.end());
        } 
        else if *self.end() + 1 == *other.start() {
            // `self` comes contiguously before `other`
            self.frames = FrameRange::new(*self.start(), *other.end());
        }
        else {
            // non-contiguous
            return Err(other);
        }

        // ensure the now-merged AllocatedFrames doesn't run its drop handler and free its frames.
        core::mem::forget(other); 
        Ok(())
    }

    /// An inner function that breaks up the given chunk into multiple smaller chunks.
    /// 
    /// Returns a tuple of three chunks:
    /// 1. The `Chunk` containing the requested range of frames starting at `start_frame`.
    /// 2. The range of frames in the `self` that came before the beginning of the requested frame range.
    /// 3. The range of frames in the `self` that came after the end of the requested frame range.
    pub fn split(
        self,
        start_frame: Frame,
        num_frames: usize,
    ) -> (Chunk, Option<Chunk>, Option<Chunk>) {
        // The new allocated chunk might start in the middle of an existing chunk,
        // so we need to break up that existing chunk into 3 possible chunks: before, newly-allocated, and after.
        //
        // Because Frames and PhysicalAddresses use saturating add/subtract, we need to double-check that 
        // we don't create overlapping duplicate Chunks at either the very minimum or the very maximum of the address space.
        let new_allocation = Chunk {
            typ: self.typ,
            // The end frame is an inclusive bound, hence the -1. Parentheses are needed to avoid overflow.
            frames: FrameRange::new(start_frame, start_frame + (num_frames - 1)),
        };
        let before = if start_frame == MIN_FRAME {
            None
        } else {
            Some(Chunk {
                typ: self.typ,
                frames: FrameRange::new(*self.start(), *new_allocation.start() - 1),
            })
        };
        let after = if new_allocation.end() == &MAX_FRAME { 
            None
        } else {
            Some(Chunk {
                typ: self.typ,
                frames: FrameRange::new(*new_allocation.end() + 1, *self.end()),
            })
        };

        // some sanity checks -- these can be removed or disabled for better performance
        if let Some(ref b) = before {
            assert!(!new_allocation.contains(b.end()));
            assert!(!b.contains(new_allocation.start()));
        }
        if let Some(ref a) = after {
            assert!(!new_allocation.contains(a.start()));
            assert!(!a.contains(new_allocation.end()));
        }

        (new_allocation, before, after)
    }

    pub fn split_at(self, at_frame: Frame) -> Result<(Chunk, Chunk), Chunk> {
        let end_of_first = at_frame - 1;

        let (first, second) = if at_frame == *self.start() && at_frame <= *self.end() {
            let first  = FrameRange::empty();
            let second = FrameRange::new(at_frame, *self.end());
            (first, second)
        } 
        else if at_frame == (*self.end() + 1) && end_of_first >= *self.start() {
            let first  = FrameRange::new(*self.start(), *self.end()); 
            let second = FrameRange::empty();
            (first, second)
        }
        else if at_frame > *self.start() && end_of_first <= *self.end() {
            let first  = FrameRange::new(*self.start(), end_of_first);
            let second = FrameRange::new(at_frame, *self.end());
            (first, second)
        }
        else {
            return Err(self);
        };

        let typ = self.typ;
        // ensure the original AllocatedFrames doesn't run its drop handler and free its frames.
        core::mem::forget(self);   
        Ok((
            Chunk { typ , frames: first }, 
            Chunk { typ , frames: second },
        ))
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