//! This file is an exact copy for the verfied functions in memory_structs but for some reason
//! verification panics when using the functions from memory_structs.
//! The errors are Prusti errors that the Range Inclusive::start() and RangeInclusive::end() functions
//! are not pure when they are, and that functions like start_frame aren't imported.
//! For now, I've moved all the functions necessary for verification here but eventually we should be able
//! to remove this file.
 
use prusti_contracts::*;

use prusti_external_spec:: {
    trusted_option::*,
    trusted_result::*
};
use prusti_representation_creator::resource_identifier::ResourceIdentifier;
use core::char::MAX;
use core::ops::{Deref, DerefMut, Add, Sub};
use core::cmp::{PartialOrd, Ord, Ordering};
use range_inclusive::*;


pub(crate) const MAX_VIRTUAL_ADDRESS: usize = usize::MAX;
pub(crate) const MAX_PAGE_NUMBER: usize = MAX_VIRTUAL_ADDRESS / PAGE_SIZE;
pub(crate) const PAGE_SIZE: usize = 4096;

// //Prusti error: Unsupported constant type
// pub(crate) const MIN_FRAME: Frame = Frame { number: 0 };
// //Prusti error: Unsupported constant type
// pub(crate) const MAX_FRAME: Frame = Frame { number: 0xFF_FFFF_FFFF }; // usize::MAX & 0x000F_FFFF_FFFF_FFFF / PAGE_SIZE

pub(crate) const MIN_FRAME_NUMBER: usize = 0;
pub(crate) const MAX_FRAME_NUMBER: usize = 0xFF_FFFF_FFFF; // usize::MAX & 0x000F_FFFF_FFFF_FFFF / PAGE_SIZE


#[pure]
fn min(a: usize, b: usize) -> usize {
    if a < b { a } else { b }
}

#[pure]
fn max(a: usize, b: usize) -> usize {
    if a > b { a } else { b }
}

#[pure]
fn min_frame(a: Frame, b: Frame) -> Frame {
    if a < b { a } else { b }
}

#[pure]
fn max_frame(a: Frame, b: Frame) -> Frame {
    if a > b { a } else { b }
}

#[pure]
#[trusted]
#[ensures(usize::MAX - a < b ==> result == usize::MAX)]
#[ensures(usize::MAX - a > b ==> result == a + b)]
#[ensures(usize::MAX - a == b ==> result == a + b)]
#[ensures(usize::MAX - a == b ==> result == usize::MAX)]
fn saturating_add(a: usize, b: usize) -> usize {
     a.saturating_add(b)
}

#[pure]
#[trusted]
#[ensures(a < b ==> result == 0)]
#[ensures(a >= b ==> result == a - b)]
#[ensures(a == b ==> result == 0)]
fn saturating_sub(a: usize, b: usize) -> usize {
     a.saturating_sub(b)
}


#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
// #[cfg_attr(not(prusti), derive(Debug))]
pub struct Frame{
    number: usize
}

impl Deref for Frame {
    type Target = usize;
    #[pure]
    fn deref(&self) -> &Self::Target {
        &self.number
    }
}

#[extern_spec]
impl PartialOrd for Frame {
    #[pure]
    #[ensures(result == self.number.partial_cmp(&other.number))]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering>;
}

#[refine_trait_spec]
impl Add<usize> for Frame {
    type Output = Frame;
    #[pure]
    #[trusted]
    #[ensures(result.number == min(MAX_FRAME_NUMBER, saturating_add(self.number, rhs)))]
    #[ensures(result >= self)]
    #[ensures(rhs == 0 ==> result == self)]
    fn add(self, rhs: usize) -> Frame {
        Frame {
            number: min(MAX_FRAME_NUMBER, saturating_add(self.number, rhs)),
        }
    }
}

#[refine_trait_spec]
impl Sub<usize> for Frame {
    type Output = Frame;
    #[pure]
    #[trusted]
    #[ensures(result.number == saturating_sub(self.number, rhs))]
    #[ensures(result <= self)]
    #[ensures(rhs == 0 ==> result == self)]
    fn sub(self, rhs: usize) -> Frame {
        Frame {
            number: saturating_sub(self.number, rhs),
        }
    }
}
impl Frame {
    #[pure]
    pub const fn number(&self) -> usize {
        self.number
    }
}
/// A struct representing an unallocated region in memory.
/// Its functions are formally verified to prevent range overlaps between chunks.
// #[cfg_attr(not(prusti), derive(Debug))]
#[derive(Copy, Clone, PartialEq, Eq)]
pub struct FrameRange(RangeInclusive<Frame>);

impl ResourceIdentifier for FrameRange {
    // type Resource = FrameChunk;

    #[pure]
    // #[trusted]
    fn overlaps(&self, other: &Self) -> bool {
        self.range_overlaps(other)
    }

    // #[pure]
    // #[trusted]
    // fn equal_to_resource(&self, resource: &Self::Resource) -> bool {
    //     self.start() == resource.start() && self.end() == resource.end()
    // }
}

impl FrameRange {
    #[ensures(result.start_frame() == start)]
    #[ensures(result.end_frame() == end)]
    pub const fn new(start: Frame, end: Frame) -> FrameRange {
        FrameRange(RangeInclusive::new(start, end))
    }

    #[trusted] // otherwise use constructor with spec
    #[ensures(result.is_empty())]
    pub const fn empty() -> FrameRange {
        FrameRange( RangeInclusive::new(Frame{ number: 1 }, Frame{ number: 0 }) )
    }
}

// The newly added methods for FrameRange required for verification
impl FrameRange {
    #[pure]
    #[trusted]
    #[ensures(result == *self.0.start())]
    pub fn start_frame(&self) -> Frame {
        *self.0.start()
    }

    #[pure]
    #[trusted]
    #[ensures(result == *self.0.end())]
    pub fn end_frame(&self) -> Frame {
        *self.0.end()
    }

    #[pure]
    #[ensures(result == (self.start_frame() > self.end_frame()))]
    #[ensures(result == !(self.start_frame() <= self.end_frame()))]
    pub fn is_empty(&self) -> bool {
        !(self.start_frame() <= self.end_frame())
    }

    #[pure]
    #[trusted] // has to be trusted to call itself, which then requires us to define a spec for the fn as well :(
    #[ensures(result == other.range_overlaps(&self))] // if we dont have this condition, then post-condition of push_unique_with_precond wont' verify
    #[ensures({
        let starts = max_frame(self.start_frame(), other.start_frame());
        let ends   = min_frame(self.end_frame(), other.end_frame());
        result == (starts <= ends) 
   })]
    /// Returning a FrameRange here requires use to set the RangeInclusive new function as pure which
    /// requires Idx to be Copy, so just return bool.
    pub fn range_overlaps(&self, other: &FrameRange) -> bool {
        let starts = max_frame(self.start_frame(), other.start_frame());
        let ends   = min_frame(self.end_frame(), other.end_frame());
        starts <= ends
    }

    #[pure]
    pub fn contains_range(&self, other: &FrameRange) -> bool {
        !other.is_empty()
        && (other.start_frame() >= self.start_frame())
        && (other.end_frame() <= self.end_frame())
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
        && ((split_range.2).is_some() ==> !split_range.1.range_overlaps(peek_option_ref(&split_range.2)))
        && (((split_range.0).is_some() && (split_range.2).is_some()) ==> !peek_option_ref(&split_range.0).range_overlaps(peek_option_ref(&split_range.2)))
    })]
    #[ensures(result.is_ok() ==> {
        let split_range = peek_result_ref(&result);
        ((split_range.0).is_some() ==> peek_option_ref(&split_range.0).end_frame() == split_range.1.start_frame() - 1)
        && ((split_range.2).is_some() ==> peek_option_ref(&split_range.2).start_frame() == split_range.1.end_frame() + 1)
    })]
    #[ensures(result.is_ok() ==> {
        let split_range = peek_result_ref(&result);
        ((split_range.0).is_some() ==> peek_option_ref(&split_range.0).start_frame() == self.start_frame())
        && ((split_range.0).is_none() ==> (split_range.1.start_frame() == self.start_frame() || (split_range.1.start_frame().number == MIN_FRAME_NUMBER)))
        && ((split_range.2).is_some() ==> peek_option_ref(&split_range.2).end_frame() == self.end_frame())
        && ((split_range.2).is_none() ==> ((split_range.1.end_frame() == self.end_frame()) || (split_range.1.end_frame().number == MAX_FRAME_NUMBER)))
    })]
    #[ensures(result.is_err() ==> {
        let orig_range = peek_err_ref(&result);
        (orig_range.start_frame() == self.start_frame()) && (orig_range.end_frame() == self.end_frame())
    })]
    pub fn split_range(self, frames_to_extract: FrameRange) -> Result<(Option<FrameRange>, FrameRange, Option<FrameRange>), FrameRange> {
        let min_frame = Frame { number: MIN_FRAME_NUMBER };
        let max_frame = Frame { number: MAX_FRAME_NUMBER };

        if !self.contains_range(&frames_to_extract) {
            return Err(self);
        }

        let start_frame = frames_to_extract.start_frame();
        let end_frame = frames_to_extract.end_frame();
        
        let before_start = if start_frame == min_frame || start_frame == self.start_frame() {
            None
        } else {
            let a = FrameRange::new(self.start_frame(), start_frame - 1);
            Some(a)

        };
        
        let start_to_end = frames_to_extract;
        
        let after_end = if end_frame == max_frame || end_frame == self.end_frame() {
            None
        } else {
            Some(FrameRange::new(end_frame + 1, self.end_frame())) 
        };

        Ok((before_start, start_to_end, after_end))
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
    pub fn split_at(self, at_frame: Frame) -> Result<(Self, Self), Self> {
        if self.is_empty() {
            return Err(self);
        }
        let end_of_first = at_frame - 1;

        let (first, second) = if (at_frame == self.start_frame()) && (at_frame <= self.end_frame()) {
            let first  = FrameRange::empty();
            let second = FrameRange::new(at_frame, self.end_frame());
            (first, second)
        } 
        else if (at_frame == self.end_frame() + 1) && (end_of_first >= self.start_frame()) {
            let first  = FrameRange::new(self.start_frame(), self.end_frame()); 
            let second = FrameRange::empty();
            (first, second)
        }
        else if (at_frame > self.start_frame()) && (end_of_first <= self.end_frame()) {
            let first  = FrameRange::new(self.start_frame(), end_of_first);
            let second = FrameRange::new(at_frame, self.end_frame());
            (first, second)
        }
        else {
            return Err(self);
        };
 
        Ok(( first, second ))
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
    pub fn merge(&mut self, other: Self) -> Result<(), Self> {
        if self.is_empty() || other.is_empty() {
            return Err(other);
        }

        if self.start_frame() == (other.end_frame() + 1) {
            // `other` comes contiguously before `self`
            *self = FrameRange::new(other.start_frame(), self.end_frame());
        } 
        else if (self.end_frame() + 1) == other.start_frame() {
            // `self` comes contiguously before `other`
            *self = FrameRange::new(self.start_frame(), other.end_frame());
        }
        else {
            // non-contiguous
            return Err(other);
        } 
        Ok(())
    }
}


impl Deref for FrameRange {
    type Target = RangeInclusive<Frame>;
    #[pure]
    fn deref(&self) -> &RangeInclusive<Frame> {
        &self.0
    }
}