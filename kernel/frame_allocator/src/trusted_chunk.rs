use memory_structs::{FrameRange, Frame};
use range_inclusive::RangeInclusive;
use core::ops::Deref;

#[derive(Copy, Clone, Debug)]
pub enum ChunkCreationError {
    /// There was already a `TrustedChunk` created with an overlapping range
    Overlap(usize),
    /// In the pre-heap-intialization phase, if there is no more space in the array
    NoSpace,
    /// The requested range is empty (end > start)
    InvalidRange
}


/// A struct representing an unallocated region in memory.
/// Its functions are formally verified to prevent range overlaps between chunks.
#[cfg_attr(not(prusti), derive(Debug, PartialEq, Eq))]
pub struct TrustedChunk {
    frames: FrameRange
}

pub fn init(range: FrameRange) -> TrustedChunk{
    TrustedChunk{ frames: range }
}
// assert_not_impl_any!(TrustedChunk: DerefMut, Clone);

impl TrustedChunk {
    // pub fn start(&self) -> Frame {
    //     *self.frames.start()
    // }

    // pub fn end(&self) -> Frame {
    //     *self.frames.end()
    // }

    pub fn frames(&self) -> FrameRange {
        self.frames.clone()
    }

    pub const fn empty() -> TrustedChunk {
        TrustedChunk { frames: FrameRange::empty() }
    }

    pub fn is_empty(&self) -> bool {
        !(self.start() <= self.end())

    }

    pub fn new(chunk_range: FrameRange) -> Result<TrustedChunk, ChunkCreationError> {
        if chunk_range.is_empty() {
            return Err(ChunkCreationError::InvalidRange);
        }

        Ok(TrustedChunk { frames: chunk_range })
    }

    pub(crate) fn trusted_new(frames: FrameRange) -> TrustedChunk {
        TrustedChunk{frames}
    }

        /// Merges the given `other` `Frames` object into this `Frames` object (`self`).
    ///
    /// This function performs no allocation or re-mapping, it exists for convenience and usability purposes.
    ///
    /// The given `other` must be physically contiguous with `self`, i.e., come immediately before or after `self`.
    /// That is, either `self.start == other.end + 1` or `self.end + 1 == other.start` must be true. 
    ///
    /// If either of those conditions are met, `self` is modified and `Ok(())` is returned,
    /// otherwise `Err(other)` is returned.
    /// #[inline(always)]
    pub fn merge(&mut self, mut other: Self) -> Result<(), Self> {
        if self.is_empty() || other.is_empty() {
            return Err(other);
        }

        let frames = if *self.start() == *other.end() + 1 {
            // `other` comes contiguously before `self`
            FrameRange::new(*other.start(), *self.end())
        } 
        else if *self.end() + 1 == *other.start() {
            // `self` comes contiguously before `other`
            FrameRange::new(*self.start(), *other.end())
        }
        else {
            // non-contiguous
            return Err(other);
        };

        // ensure the now-merged Frames doesn't run its drop handler
        core::mem::forget(other); 
        // self.frames = frames.clone();
        self.frames = frames;
        Ok(())
    }

    /// Splits up the given `Frames` into multiple smaller `Frames`.
    /// 
    /// Returns a `SplitFrames` instance containing three `Frames`:
    /// 1. The range of frames in `self` that are before the beginning of `frames_to_extract`.
    /// 2. The `Frames` containing the requested range of frames, `frames_to_extract`.
    /// 3. The range of frames in `self` that are after the end of `frames_to_extract`.
    /// 
    /// If `frames_to_extract` is not contained within `self`, then `self` is returned unchanged within an `Err`.
    /// #[inline(always)]
    pub fn split_range(
        mut self,
        frames_to_extract: FrameRange
    ) -> Result<(Option<Self>, Self, Option<Self>), Self> {
        
        if !self.contains_range(&frames_to_extract) {
            return Err(self);
        }
        let start_frame = *frames_to_extract.start();
        let start_to_end = frames_to_extract;
        
        let before_start = if start_frame == crate::MIN_FRAME || start_frame == *self.start() {
            None
        } else {
            Some(FrameRange::new(*self.start(), *start_to_end.start() - 1))
        };

        let after_end = if *start_to_end.end() == crate::MAX_FRAME || *start_to_end.end() == *self.end() {
            None
        } else {
            Some(FrameRange::new(*start_to_end.end() + 1, *self.end()))
        };

        // ensure the original Frames doesn't run its drop handler and free its frames.
        core::mem::forget(self);
        Ok((
            before_start.map(|frames| TrustedChunk{ frames: frames }),
            TrustedChunk{ frames: start_to_end } , 
            after_end.map(|frames| TrustedChunk { frames: frames } ),
        ))
        
    }

    /// Splits this `Frames` into two separate `Frames` objects:
    /// * `[beginning : at_frame - 1]`
    /// * `[at_frame : end]`
    /// 
    /// This function follows the behavior of [`core::slice::split_at()`],
    /// thus, either one of the returned `Frames` objects may be empty. 
    /// * If `at_frame == self.start`, the first returned `Frames` object will be empty.
    /// * If `at_frame == self.end + 1`, the second returned `Frames` object will be empty.
    /// 
    /// Returns an `Err` containing this `Frames` if `at_frame` is otherwise out of bounds, or if `self` was empty.
    /// 
    /// [`core::slice::split_at()`]: https://doc.rust-lang.org/core/primitive.slice.html#method.split_at
    /// #[inline(always)]
    pub fn split_at(mut self, at_frame: Frame) -> Result<(Self, Self), Self> {
        if self.is_empty() { return Err(self); }

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

        // ensure the original Frames doesn't run its drop handler and free its frames.
        core::mem::forget(self);   
        Ok((
            TrustedChunk { frames: first } , 
            TrustedChunk { frames: second },
        ))
  
    }
}


impl Deref for TrustedChunk {
    type Target = FrameRange;
    fn deref(&self) -> &FrameRange {
        &self.frames
    }
}
