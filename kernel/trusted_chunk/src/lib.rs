#![no_std]
#![feature(box_patterns)]
#![allow(unused)]

#[cfg(feature="prusti")]
extern crate prusti_contracts;

#[cfg(feature="prusti")]
use prusti_contracts::*;

extern crate alloc;
use core::ops::{Deref, RangeInclusive};
use core::{borrow::Borrow, cmp::{Ordering, min, max}, fmt, ops::{DerefMut}};
use crate::{
    linked_list::*,
    trusted_range_inclusive::*,
    trusted_option::*,
};

pub mod linked_list;
pub(crate) mod trusted_range_inclusive;
pub(crate) mod trusted_option;

/*** Constants taken from kernel_config crate. Only required if CHECK_OVERFLOWS flag is enabled. ***/ 
/// The lower 12 bits of a virtual address correspond to the P1 page frame offset. 
const PAGE_SHIFT: usize = 12;
/// Page size is 4096 bytes, 4KiB pages.
const PAGE_SIZE: usize = 1 << PAGE_SHIFT;
const MAX_VIRTUAL_ADDRESS: usize = usize::MAX;
const MAX_PAGE_NUMBER: usize = MAX_VIRTUAL_ADDRESS / PAGE_SIZE;


/// A trusted function that will create a new `TrustedChunk` and add it to the `chunk_list`.
/// It will return an Err if:
/// * range.start > range.end
/// * there is an overlap with an existing chunk
#[cfg_attr(feature="prusti", trusted)]
pub fn create_new_trusted_chunk(range: RangeInclusive<usize>, chunk_list: &mut List) -> Result<TrustedChunk, &'static str> {
    if range.start() > range.end() {
        return Err("Invalid range, start > end");
    }
    TrustedChunk::new(TrustedRangeInclusive::new(*range.start(), *range.end()), chunk_list)
        .ok_or("Overlapping range with an existing Chunk!")
}

/// An unsafe trusted function that will create a new `TrustedChunk`.
/// This does not perform any check for overlaps or correct ranges and is only supposed to be used before heap initialization,
/// and also currently the drop handler
#[cfg_attr(feature="prusti", trusted)]
pub fn create_new_trusted_chunk_unsafe(range: RangeInclusive<usize>) -> TrustedChunk {
    TrustedChunk::trusted_new(TrustedRangeInclusive::new(*range.start(), *range.end()))
}


/// A trusted function that will create a `List` of the chunks (RangeInclusive) added to the frame allocator.
/// This should be called once the heap is initialized.
/// It will return an Err if for any of the chunks:
/// * chunk.start > chunk.end
/// * there is an overlap between chunks
/// This ensures that the List is initialized in a consistent state.
/// 
/// Ideally, creating and managing the initial array, and the conversion, should also be in this crate, 
/// but for some reason all verification efforts of StaticArray and StaticArrayLinkedList fail.
/// (See static_array.rs and static_array_linked_list.rs)
#[cfg_attr(feature="prusti", trusted)]
pub fn add_untracked_chunks_to_list(unallocated_regions: [Option<RangeInclusive<usize>>; 32], list: &mut List) -> Result<(), &'static str> {
    for range in unallocated_regions {
        if let Some(r) = range {
            if r.start() <= r.end() {
                let elem = TrustedRangeInclusive::new(*r.start(), *r.end());
                if List::overlaps(&list.head, elem) {
                    return Err("overlapping ranges in the initial array of chunks");
                }
                list.push(elem);
            } else {
                return Err("Invalid range, start > end");
            }
        }
    }
    Ok(())
}

/// A struct representing a unique unallocated region in memory.
/// An instantiation of this struct is formally verified to not overlap with any other `TrustedChunk`.
/// 
/// # Warning
/// A `TrustedRangeInclusive` is created with the precondition that start <= end.
/// Outside of verified code, it is the caller's responsibility to make sure that precondition is upheld.
#[derive(Debug, Eq)] // can't be use with Prusti
pub struct TrustedChunk {
    frames: TrustedRangeInclusive
}

impl Ord for TrustedChunk {
    fn cmp(&self, other: &Self) -> Ordering {
        self.frames.start.cmp(&other.frames.start)
    }
}
impl PartialOrd for TrustedChunk {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl PartialEq for TrustedChunk {
    fn eq(&self, other: &Self) -> bool {
        self.frames.start == other.frames.start
    }
}

impl TrustedChunk {
    #[cfg_attr(feature="prusti", pure)]
    pub fn start(&self) -> usize {
        self.frames.start
    }

    #[cfg_attr(feature="prusti", pure)]
    pub fn end(&self) -> usize {
        self.frames.end
    }

    /// Creates a new `TrustedChunk` iff no other chunk in the system overlaps with it, and adds the newly created chunk range to `chunk_list`.
    /// Returns None if there is an overlap.
    /// 
    /// Ideally, this function should also contain postconditions about no overlap with any chunk in the existing list,
    /// but Prusti starts giving errors at this level.
    /// For now, this function is easy to manually inspect and all List functions are formally verified.
    #[cfg_attr(feature="prusti", requires(frames.start <= frames.end))]
    #[cfg_attr(feature="prusti", ensures(result.is_some() ==> peek_chunk(&result).frames.start == frames.start))]
    #[cfg_attr(feature="prusti", ensures(result.is_some() ==> peek_chunk(&result).frames.end == frames.end))]
    fn new(frames: TrustedRangeInclusive, chunk_list: &mut List) -> Option<TrustedChunk> {
        if Self::add_chunk_to_list(frames, chunk_list) {
            Some(TrustedChunk { frames })
        } else {
            None
        }
    }

    /// A trusted function that adds `frames` to the `chunk_list`  and returns true only if
    /// there is no overlap with any range in the list.
    /// 
    /// Ideally, this function should not be trusted, and instead should be verified with the same postconditions as in push()
    /// and range_overlaps_in_list().
    /// Unfortunately, Prusti starts giving errors at this level.
    /// For now, this function is easy to manually inspect and all List functions are formally verified.
    #[cfg_attr(feature="prusti", trusted)]
    fn add_chunk_to_list(frames: TrustedRangeInclusive, chunk_list: &mut List) -> bool {
        if chunk_list.range_overlaps_in_list(frames, 0).is_some(){
            false
        } else {
            chunk_list.push(frames);
            true
        }
    }

    /// Internal function that creates a chunk without any checks.
    /// Only to be used in the split() function.
    #[cfg_attr(feature="prusti", requires(frames.start <= frames.end))]
    #[cfg_attr(feature="prusti", ensures(result.frames.start == frames.start))]
    #[cfg_attr(feature="prusti", ensures(result.frames.end == frames.end))]
    fn trusted_new(frames: TrustedRangeInclusive) -> TrustedChunk {
        TrustedChunk{frames}
    }

    /// Splits a chunk in to 1-3 chunks, depending on where the split is at.
    /// It is formally verified that the resulting chunks are disjoint, contiguous and their start/end is equal to that of the original chunk.
    #[cfg_attr(feature="prusti", requires(start_page >= self.frames.start))]
    #[cfg_attr(feature="prusti", requires(start_page + num_frames - 1 <= self.frames.end))]
    #[cfg_attr(feature="prusti", requires(num_frames > 0))]
    // #[cfg_attr(feature="prusti", requires(self.frames.end <= MAX_PAGE_NUMBER))]
    #[cfg_attr(feature="prusti", ensures(split_chunk_has_no_overlapping_ranges(&result.0, &result.1, &result.2)))]
    #[cfg_attr(feature="prusti", ensures(split_chunk_is_contiguous(&result.0, &result.1, &result.2)))]
    #[cfg_attr(feature="prusti", ensures(start_end_are_equal(&self, &result)))]
    pub fn split(&self, start_page: usize, num_frames: usize, chunk_list: &mut List) -> (Option<TrustedChunk>, TrustedChunk, Option<TrustedChunk>) {
        let first_chunk = if start_page == self.frames.start  {
            None
        } else {
            Some(TrustedChunk::trusted_new(TrustedRangeInclusive::new(self.frames.start, start_page - 1)))
        };

        let second_chunk = TrustedChunk::trusted_new(TrustedRangeInclusive::new(start_page, start_page + num_frames - 1));

        let third_chunk = if start_page + num_frames - 1 == self.frames.end {
            None
        } else {
            Some(TrustedChunk::trusted_new(TrustedRangeInclusive::new(start_page + num_frames, self.frames.end)))
        };

        (first_chunk, second_chunk, third_chunk)
    }
}


impl Deref for TrustedChunk {
    type Target = TrustedRangeInclusive;
    fn deref(&self) -> &TrustedRangeInclusive {
        &self.frames
    }
}


/*** Following are a set of pure functions that are only used in the specification of a TrustedChunk ***/

/// Checks that either chunk1 ends before chunk2 starts, or that chunk2 ends before chunk1 starts.
#[cfg_attr(feature="prusti", pure)]
fn chunks_do_not_overlap(chunk1: &TrustedChunk, chunk2: &TrustedChunk) -> bool {
    (chunk1.frames.end < chunk2.frames.start) | (chunk2.frames.end < chunk1.frames.start)
}

/// Returns true if there is no overlap in the ranges of `chunk1`, `chunk2` and `chunk3`.
#[cfg_attr(feature="prusti", pure)]
fn split_chunk_has_no_overlapping_ranges(chunk1: &Option<TrustedChunk>, chunk2: &TrustedChunk, chunk3: &Option<TrustedChunk>) -> bool {
    let mut no_overlap = true;

    if let Some(c1) = chunk1 {
        no_overlap &= chunks_do_not_overlap(c1, chunk2);
        if let Some(c3) = chunk3 {
            no_overlap &= chunks_do_not_overlap(c1, c3);
            no_overlap &= chunks_do_not_overlap(chunk2, c3);
        }
    } else {
        if let Some(c3) = chunk3 {
            no_overlap &= chunks_do_not_overlap(chunk2, c3);
        }
    }

    no_overlap
}

/// Returns true if the start and end of the original chunk is equal to the extreme bounds of the split chunk.
#[cfg_attr(feature="prusti", pure)]
fn start_end_are_equal(orig_chunk: &TrustedChunk, split_chunk: &(Option<TrustedChunk>, TrustedChunk, Option<TrustedChunk>)) -> bool {
    let (chunk1,chunk2,chunk3) = split_chunk;
    let min_page;
    let max_page;

    if let Some(c1) = chunk1 {
        min_page = c1.frames.start;    
    } else {
        min_page = chunk2.frames.start;
    }

    if let Some(c3) = chunk3 {
        max_page = c3.frames.end;    
    } else {
        max_page = chunk2.frames.end;
    }

    min_page == orig_chunk.frames.start && max_page == orig_chunk.frames.end
}


/// Returns true if `chunk1`, `chunk2` and `chunk3` are contiguous.
#[cfg_attr(feature="prusti", pure)]
// The following are only required if CHECK_OVERFLOWS flag is enabled
// #[cfg_attr(feature="prusti", requires(end_frame_is_less_than_max_or_none(chunk1)))]
// #[cfg_attr(feature="prusti", requires(end_frame_is_less_than_max(chunk2)))]
// #[cfg_attr(feature="prusti", requires(end_frame_is_less_than_max_or_none(chunk3)))]
fn split_chunk_is_contiguous(chunk1: &Option<TrustedChunk>, chunk2: &TrustedChunk, chunk3: &Option<TrustedChunk>) -> bool {
    let mut contiguous = true;
    if let Some(c1) = chunk1 {
        contiguous &= c1.frames.end + 1 == chunk2.frames.start
    } 
    if let Some(c3) = chunk3 {
        contiguous &= chunk2.frames.end + 1 == c3.frames.start
    }
    contiguous
}

/// Returns true if the end frame of the chunk is less than `MAX_PAGE_NUMBER`, or if the chunk is None.
#[cfg_attr(feature="prusti", pure)]
fn end_frame_is_less_than_max_or_none(chunk: &Option<TrustedChunk>) -> bool {
    if let Some(c) = chunk {
        if c.frames.end <= MAX_PAGE_NUMBER {
            return true;
        } else {
            return false;
        }
    } else {
        return true;
    }

}

/// Returns true if the end frame of the chunk is less than `MAX_PAGE_NUMBER`.
#[cfg_attr(feature="prusti", pure)]
fn end_frame_is_less_than_max(chunk: &TrustedChunk) -> bool {
    if chunk.frames.end <= MAX_PAGE_NUMBER {
        return true;
    } else {
        return false;
    }
}
