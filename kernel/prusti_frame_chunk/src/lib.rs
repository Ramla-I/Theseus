#![no_std]
#![allow(dead_code)]
#![allow(unused_imports)]

#[macro_use(private_fields)]
extern crate proc_static_assertions;

use memory_structs::{Frame, FrameRange};
use static_assertions::assert_not_impl_any;

use prusti_representation_creator::RepresentationCreator;
use prusti_external_spec::{trusted_option::*,trusted_result::*};
use proc_static_assertions::consumes;
use core::ops::{Deref, DerefMut};
use kernel_config::memory::{MAX_PAGE_NUMBER, MIN_PAGE_NUMBER};
use range_inclusive::*;
use prusti_contracts::*;


static INIT: spin::Once<bool> = spin::Once::new();

pub fn init_frame_chunk() -> Result<fn(FrameRange) -> FrameChunk, &'static str> {
    if INIT.is_completed() {
        Err("Trusted Chunk has already been initialized and callback has been returned")
    } else {
        INIT.call_once(|| true);
        Ok(create_from_unmapped)
    }
}

#[requires(frames.start_frame() <= frames.end_frame())]
#[ensures(result.start_frame() == frames.start_frame())]
#[ensures(result.end_frame() == frames.end_frame())]
fn create_from_unmapped(frames: FrameRange) -> FrameChunk {
    FrameChunk::trusted_new(&frames)
}

pub struct FrameChunkCreator(RepresentationCreator<FrameRange, FrameChunk>);

impl FrameChunkCreator {
    #[trusted]
    pub const fn new() -> Self { // To Do: this should be private and only called once
        FrameChunkCreator(RepresentationCreator::new(FrameChunk::trusted_new, true))
    }
}

impl Deref for FrameChunkCreator {
    type Target = RepresentationCreator<FrameRange, FrameChunk>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for FrameChunkCreator {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// A struct representing an unallocated region in memory.
/// Its functions are formally verified to prevent range overlaps between chunks.
#[derive(PartialEq, Eq)]
pub struct FrameChunk {
    frames: FrameRange
}


assert_not_impl_any!(FrameChunk: DerefMut, Clone);

impl FrameChunk {
    #[pure]
    #[trusted]
    #[ensures(result == self.frames.start())]
    pub fn start(&self) -> &Frame {
        self.frames.start()
    }

    #[pure]
    #[trusted]
    #[ensures(result == self.frames.end())]
    pub fn end(&self) -> &Frame {
        self.frames.end()
    }

    #[ensures(result.is_empty())]
    pub const fn empty() -> FrameChunk {
        FrameChunk { frames: FrameRange::empty() }
    }

    #[pure]
    pub fn is_empty(&self) -> bool {
        self.frames.is_empty()
    }

    /// Private function that creates a chunk without any checks.
    /// 
    /// Only used within other verified functions, or registered as a callback
    #[requires(frames.start_frame() <= frames.end_frame())]
    #[ensures(result.start() == frames.start())]
    #[ensures(result.end() == frames.end())]
    pub(crate) fn trusted_new(frames: &FrameRange) -> FrameChunk {
        FrameChunk{ frames: *frames }
    }


    /// Splits a range into 1-3 ranges, depending on where the split is at.
    /// It is formally verified that the resulting chunks are disjoint, contiguous and their start/end is equal to that of the original chunk.
    /// 
    /// # Post-conditions:
    /// * If it succeeds, then the resulting chunks have no overlapping ranges
    /// * If it succeeds, then the resulting chunks are contiguous
    /// * If it succeeds, then the resulting chunks combined have the same range as `self`
    /// * If it fails, then the original chunk is returned
    #[ensures(result.is_ok() ==> {
        let split_range = peek_result_ref(&result);
        ((split_range.0).is_some() ==> !peek_option_ref(&split_range.0).range_overlaps(&split_range.1)) 
        && ((split_range.2).is_some() ==> !split_range.1.range_overlaps(peek_option_ref(&split_range.2).deref()))
        && (((split_range.0).is_some() && (split_range.2).is_some()) ==> !peek_option_ref(&split_range.0).range_overlaps(peek_option_ref(&split_range.2).deref()))
    })]
    #[ensures(result.is_ok() ==> {
        let split_range = peek_result_ref(&result);
        ((split_range.0).is_some() ==> peek_option_ref(&split_range.0).end_frame() == split_range.1.start_frame() - 1)
        && ((split_range.2).is_some() ==> peek_option_ref(&split_range.2).start_frame() == split_range.1.end_frame() + 1)
    })]
    #[ensures(result.is_ok() ==> {
        let split_range = peek_result_ref(&result);
        ((split_range.0).is_some() ==> peek_option_ref(&split_range.0).start_frame() == self.start_frame())
        && ((split_range.0).is_none() ==> (split_range.1.start_frame() == self.start_frame() || (split_range.1.start_frame().number() == MIN_PAGE_NUMBER)))
        && ((split_range.2).is_some() ==> peek_option_ref(&split_range.2).end_frame() == self.end_frame())
        && ((split_range.2).is_none() ==> ((split_range.1.end_frame() == self.end_frame()) || (split_range.1.end_frame().number() == MAX_PAGE_NUMBER)))
    })]
    #[ensures(result.is_err() ==> {
        let orig_range = peek_err_ref(&result);
        (orig_range.start_frame() == self.start_frame()) && (orig_range.end_frame() == self.end_frame())
    })]
    #[consumes("self")]
    pub fn split_range(self, frames_to_extract: FrameRange) -> Result<(Option<FrameChunk>, FrameChunk, Option<FrameChunk>), FrameChunk> {
        
        let (before, start_to_end, after) = match self.frames.split_range(frames_to_extract) {
            Ok(x) => x,
            Err(_) => return Err(self)
        };

        core::mem::forget(self);

        let before_start = match before {
            Some(x) => Some(FrameChunk { frames: x }),
            None => None
        };

        let after_end = match after {
            Some(x) => Some(FrameChunk { frames: x }),
            None => None
        };
        
        Ok((
            before_start,
            FrameChunk { frames: start_to_end },
            after_end
        ))

    }


    /// Splits a chunk into 2 chunks at the frame with number `at_frame`.
    /// It is formally verified that the resulting chunks are disjoint, contiguous and their start/end is equal to that of the original chunk.
    /// 
    /// # Post-conditions:
    /// * If it succeeds, then both chunks can't be empty
    /// * If it succeeds and the first chunk is empty, then the second chunk is equal to `self`
    /// * If it succeeds and the second chunk is empty, then the first chunk is equal to `self`
    /// * If it succeeds and both chunks aren't empty, then the chunks are contiguous and their combined range is equal to the range of `self`
    /// * If it fails, then the original chunk is returned
    #[ensures(result.is_ok() ==> {
        let split_range = peek_result_ref(&result);
        split_range.0.is_empty() && !split_range.1.is_empty() ||
        !split_range.0.is_empty() && split_range.1.is_empty() ||
        !split_range.0.is_empty() && !split_range.1.is_empty()
    })]
    #[ensures(result.is_ok() ==> {
        let split_range = peek_result_ref(&result);
        split_range.0.is_empty() ==> (split_range.1.start_frame() == self.start_frame() && split_range.1.end_frame() == self.end_frame())
    })]
    #[ensures(result.is_ok() ==> {
        let split_range = peek_result_ref(&result);
        split_range.1.is_empty() ==> (split_range.0.start_frame() == self.start_frame() && split_range.0.end_frame() == self.end_frame())
    })]
    #[ensures(result.is_ok() ==> {
        let split_range = peek_result_ref(&result);
        (!split_range.0.is_empty() && !split_range.1.is_empty()) ==> (
            split_range.0.start_frame() == self.start_frame() 
            && split_range.0.end_frame() == at_frame - 1
            && split_range.1.start_frame() == at_frame 
            && split_range.1.end_frame() == self.end_frame()
        )
    })]
    #[ensures(result.is_err() ==> {
        let orig_chunk = peek_err_ref(&result);
        (orig_chunk.start_frame() == self.start_frame()) && (orig_chunk.end_frame() == self.end_frame())
    })]
    #[consumes("self")]
    pub fn split_at(self, at_frame: Frame) -> Result<(FrameChunk, FrameChunk), FrameChunk> {

        let (first, second) = match self.frames.split_at(at_frame) {
            Ok(x) => x,
            Err(_) => return Err(self)
        };

        core::mem::forget(self);   
        Ok((FrameChunk{ frames: first }, FrameChunk{ frames: second }))
    }


    /// Merges `other` into `self`.
    /// Succeeds if `other` lies right before `self` or right after.
    /// 
    /// # Post-conditions:
    /// * If it succeeds, then `other` and `self` were contiguous, and either `self`'s start bound has been updated to `other`'s start 
    /// or `self`s end has been updated to `other`'s end
    /// * If it fails, then `self` remains unchanged and `other` is returned
    #[ensures(result.is_ok() ==> 
        (old(self.start_frame()) == other.end_frame() + 1 && self.start_frame() == other.start_frame() && self.end_frame() == old(self.end_frame())) 
        || 
        (old(self.end_frame()) + 1 == other.start_frame() && self.end_frame() == other.end_frame() && self.start_frame() == old(self.start_frame()))
    )]
    #[ensures(result.is_err() ==> {
        let chunk = peek_err_ref(&result);
        (chunk.start_frame() == other.start_frame()) && (chunk.end_frame() == other.end_frame()) 
    })]
    #[ensures(result.is_err() ==> {
        (self.start_frame() == old(self.start_frame())) && (self.end_frame() == old(self.end_frame())) 
    })]
    #[consumes("FrameChunk")]
    pub fn merge(&mut self, other: FrameChunk) -> Result<(), FrameChunk> {
        if self.frames.merge(other.frames).is_ok() {
            core::mem::forget(other);
            Ok(())
        } else {
            Err(other)
        }
    }
}


impl Deref for FrameChunk {
    type Target = FrameRange;
    #[pure]
    fn deref(&self) -> &FrameRange {
        &self.frames
    }
}