//! Provides an allocator for physical memory frames.
//! The minimum unit of allocation is a single frame. 
//!
//! This is currently a modified and more complex version of the `page_allocator` crate.
//! TODO: extract the common code and create a generic allocator that can be specialized to allocate pages or frames.
//! 
//! This also supports early allocation of frames before heap allocation is available, 
//! and does so behind the scenes using the same single interface. 
//! Early pre-heap allocations are limited to tracking a small number of available chunks (currently 32).
//! 
//! Once heap allocation is available, it uses a dynamically-allocated list of frame chunks to track allocations.
//! 
//! The core allocation function is [`allocate_frames_deferred()`](fn.allocate_frames_deferred.html), 
//! but there are several convenience functions that offer simpler interfaces for general usage. 
//!
//! # Notes and Missing Features
//! This allocator currently does **not** merge freed chunks (de-fragmentation). 
//! We don't need to do so until we actually run out of address space or until 
//! a requested address is in a chunk that needs to be merged.

#![no_std]
#![feature(box_into_inner)]

extern crate alloc;
#[macro_use] extern crate log;
extern crate kernel_config;
extern crate memory_structs;
extern crate spin;
#[macro_use] extern crate static_assertions;
extern crate intrusive_collections;
extern crate range_inclusive;

#[cfg(test)]
mod test;

mod static_array_rb_tree;
// mod static_array_linked_list;
mod chunk;
mod region;

use core::{borrow::Borrow, cmp::{Ordering, min, max}, fmt, ops::{Deref, DerefMut, RangeInclusive}, marker::PhantomData};
use kernel_config::memory::*;
use memory_structs::{PhysicalAddress, Frame, FrameRange};
use spin::Mutex;
use intrusive_collections::Bound;
use static_array_rb_tree::*;
use chunk::*;
use region::*;
use range_inclusive::RangeInclusiveIterator;

const FRAME_SIZE: usize = PAGE_SIZE;
pub(crate) const MIN_FRAME: Frame = Frame::containing_address(PhysicalAddress::zero());
pub(crate) const MAX_FRAME: Frame = Frame::containing_address(PhysicalAddress::new_canonical(usize::MAX));

// Note: we keep separate lists for "free, general-purpose" areas and "reserved" areas, as it's much faster. 

/// The single, system-wide list of free physical memory frames available for general usage. 
static FREE_GENERAL_FRAMES_LIST: Mutex<StaticArrayRBTree<Chunk>> = Mutex::new(StaticArrayRBTree::empty()); 
/// The single, system-wide list of free physical memory frames reserved for specific usage. 
static FREE_RESERVED_FRAMES_LIST: Mutex<StaticArrayRBTree<Chunk>> = Mutex::new(StaticArrayRBTree::empty()); 

/// The fixed list of all known regions that are available for general use.
/// This does not indicate whether these regions are currently allocated, 
/// rather just where they exist and which regions are known to this allocator.
static GENERAL_REGIONS: Mutex<StaticArrayRBTree<Region>> = Mutex::new(StaticArrayRBTree::empty());
/// The fixed list of all known regions that are reserved for specific purposes. 
/// This does not indicate whether these regions are currently allocated, 
/// rather just where they exist and which regions are known to this allocator.
static RESERVED_REGIONS: Mutex<StaticArrayRBTree<Region>> = Mutex::new(StaticArrayRBTree::empty());


/// Initialize the frame allocator with the given list of available and reserved physical memory regions.
///
/// Any regions in either of the lists may overlap, this is checked for and handled properly.
/// Reserved regions take priority -- if a reserved region partially or fully overlaps any part of a free region,
/// that portion will be considered reserved, not free. 
/// 
/// The iterator (`R`) over reserved physical memory regions must be cloneable, 
/// as this runs before heap allocation is available, and we may need to iterate over it multiple times. 
/// 
/// ## Return
/// Upon success, this function returns a callback function that allows the caller
/// (the memory subsystem init function) to convert a range of unmapped frames 
/// back into an [`AllocatedFrames`] object.
pub fn init<F, R, P>(
    free_physical_memory_areas: F,
    reserved_physical_memory_areas: R,
) -> Result<fn(FrameRange) -> AllocatedFrames, &'static str> 
    where P: Borrow<PhysicalMemoryRegion>,
          F: IntoIterator<Item = P>,
          R: IntoIterator<Item = P> + Clone,
{
    if  FREE_GENERAL_FRAMES_LIST .lock().len() != 0 ||
        FREE_RESERVED_FRAMES_LIST.lock().len() != 0 ||
        GENERAL_REGIONS          .lock().len() != 0 ||
        RESERVED_REGIONS         .lock().len() != 0 
    {
        return Err("BUG: Frame allocator was already initialized, cannot be initialized twice.");
    }

    // start with all lists using the Region type so we can merge and manipulate until we're sure we have non-overlapping regions
    let mut free_list: [Option<Region>; 32] = Default::default();
    let mut free_list_idx = 0;
    let mut reserved_list: [Option<Region>; 32] = Default::default();
    let mut reserved_list_idx = 0;
    let mut free_regions: [Option<Region>; 32] = Default::default();
    let mut reserved_regions: [Option<Region>; 32] = Default::default();

    // Populate the list of free regions for general-purpose usage.
    for area in free_physical_memory_areas.into_iter() {
        let area = area.borrow();
        // debug!("Frame Allocator: looking to add free physical memory area: {:?}", area);
        check_and_add_free_region(
            &area,
            &mut free_list,
            &mut free_list_idx,
            reserved_physical_memory_areas.clone(),
        );
    }

    // Insert all of the reserved memory areas into the list of free reserved regions,
    // while de-duplicating overlapping areas by merging them.
    for reserved in reserved_physical_memory_areas.into_iter() {
        let reserved = reserved.borrow();
        let mut reserved_was_merged = false;
        for existing in reserved_list[..reserved_list_idx].iter_mut().flatten() {
            if let Some(_overlap) = existing.overlap(reserved) {
                // merge the `reserved` range into the `existing` range
                existing.frames = FrameRange::new(
                    min(*existing.start(), *reserved.start()),
                    max(*existing.end(),   *reserved.end()),
                );
                reserved_was_merged = true;
                break;
            }
        }
        if !reserved_was_merged {
            reserved_list[reserved_list_idx] = Some(Region {
                typ:  MemoryRegionType::Reserved,
                frames: reserved.frames.clone(),
            });
            reserved_list_idx += 1;
        }
    }


    // Finally, one last sanity check -- ensure no two regions overlap. 
    let all_areas = free_list[..free_list_idx].iter().flatten()
        .chain(reserved_list[..reserved_list_idx].iter().flatten());
    for (i, elem) in all_areas.clone().enumerate() {
        let next_idx = i + 1;
        for other in all_areas.clone().skip(next_idx) {
            if let Some(overlap) = elem.overlap(other) {
                panic!("BUG: frame allocator free list had overlapping ranges: \n \t {:?} and {:?} overlap at {:?}",
                    elem, other, overlap,
                );
            }
        }
    }


    // Here, since we're sure we now have a list of regions that we don't overlap, we can create lists of formally verified Chunks
    let mut free_list_w_chunks: [Option<Chunk>; 32] = Default::default();
    let mut reserved_list_w_chunks: [Option<Chunk>; 32] = Default::default();
    for i in 0..32 {
        if let Some(chunk) = &reserved_list[i] {
            reserved_list_w_chunks[i] = Some(Chunk::new(
                MemoryRegionType::Reserved,
                chunk.frames.clone()
            )?);
        }
        if let Some(chunk) = &free_list[i] {
            free_list_w_chunks[i] = Some(Chunk::new(
                MemoryRegionType::Free,
                chunk.frames.clone()
            )?);
        }
    }

    for i in 0..32 {
        if let Some(chunk) = &reserved_list[i] {
            reserved_regions[i] = Some(Region {
                typ: MemoryRegionType::Reserved,
                frames: chunk.frames.clone()
            });
        }
        if let Some(chunk) = &free_list[i] {
            free_regions[i] = Some(Region {
                typ: MemoryRegionType::Free,
                frames: chunk.frames.clone()
            });
        }
    }

    *FREE_GENERAL_FRAMES_LIST.lock()  = StaticArrayRBTree::new(free_list_w_chunks);
    *FREE_RESERVED_FRAMES_LIST.lock() = StaticArrayRBTree::new(reserved_list_w_chunks);
    *GENERAL_REGIONS.lock()           = StaticArrayRBTree::new(free_regions);
    *RESERVED_REGIONS.lock()          = StaticArrayRBTree::new(reserved_regions);

    Ok(into_allocated_frames)
}


/// The main logic of the initialization routine 
/// used to populate the list of free frame chunks.
///
/// This function recursively iterates over the given `area` of frames
/// and adds any ranges of frames within that `area` that are not covered by
/// the given list of `reserved_physical_memory_areas`.
fn check_and_add_free_region<P, R>(
    area: &FrameRange,
    free_list: &mut [Option<Region>; 32],
    free_list_idx: &mut usize,
    reserved_physical_memory_areas: R,
)
    where P: Borrow<PhysicalMemoryRegion>,
          R: IntoIterator<Item = P> + Clone,
{
    // This will be set to the frame that is the start of the current free region. 
    let mut current_start = *area.start();
    // This will be set to the frame that is the end of the current free region. 
    let mut current_end = *area.end();
    // trace!("looking at sub-area {:X?} to {:X?}", current_start, current_end);

    for reserved in reserved_physical_memory_areas.clone().into_iter() {
        let reserved = &reserved.borrow().frames;
        // trace!("\t Comparing with reserved area {:X?}", reserved);
        if reserved.contains(&current_start) {
            // info!("\t\t moving current_start from {:X?} to {:X?}", current_start, *reserved.end() + 1);
            current_start = *reserved.end() + 1;
        }
        if &current_start <= reserved.start() && reserved.start() <= &current_end {
            // Advance up to the frame right before this reserved region started.
            // info!("\t\t moving current_end from {:X?} to {:X?}", current_end, min(current_end, *reserved.start() - 1));
            current_end = min(current_end, *reserved.start() - 1);
            if area.end() <= reserved.end() {
                // Optimization here: the rest of the current area is reserved,
                // so there's no need to keep iterating over the reserved areas.
                // info!("\t !!! skipping the rest of the area");
                break;
            } else {
                let after = FrameRange::new(*reserved.end() + 1, *area.end());
                // warn!("moving on to after {:X?}", after);
                // Here: the current area extends past this current reserved area,
                // so there might be another free area that starts after this reserved area.
                check_and_add_free_region(
                    &after,
                    free_list,
                    free_list_idx,
                    reserved_physical_memory_areas.clone(),
                );
            }
        }
    }

    let new_area = FrameRange::new(current_start, current_end);
    if new_area.size_in_frames() > 0 {
        free_list[*free_list_idx] = Some(Region {
            typ:  MemoryRegionType::Free,
            frames: new_area,
        });
        *free_list_idx += 1;
    }
}


/// A region of physical memory.
#[derive(Clone, Debug)]
pub struct PhysicalMemoryRegion {
    pub frames: FrameRange,
    pub typ: MemoryRegionType,
}
impl PhysicalMemoryRegion {
    pub fn new(frames: FrameRange, typ: MemoryRegionType) -> PhysicalMemoryRegion {
        PhysicalMemoryRegion { frames, typ }
    }
}
impl Deref for PhysicalMemoryRegion {
    type Target = FrameRange;
    fn deref(&self) -> &FrameRange {
        &self.frames
    }
}

/// Types of physical memory. See each variant's documentation.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MemoryRegionType {
    /// Memory that is available for any general purpose.
    Free,
    /// Memory that is reserved for special use and is only ever allocated from if specifically requested. 
    /// This includes custom memory regions added by third parties, e.g., 
    /// device memory discovered and added by device drivers later during runtime.
    Reserved,
    /// Memory of an unknown type.
    /// This is a default value that acts as a sanity check, because it is invalid
    /// to do any real work (e.g., allocation, access) with an unknown memory region.
    Unknown,
}

/// Represents a range of allocated physical memory [`Frame`]s; derefs to [`FrameRange`].
/// 
/// These frames are not immediately accessible because they're not yet mapped
/// by any virtual memory pages.
/// You must do that separately in order to create a `MappedPages` type,
/// which can then be used to access the contents of these frames.
/// 
/// This object represents ownership of the range of allocated physical frames;
/// if this object falls out of scope, its allocated frames will be auto-deallocated upon drop. 
pub struct AllocatedFrames {
    frames: Chunk,
}

// AllocatedFrames must not be Cloneable, and it must not expose its inner frames as mutable.
assert_not_impl_any!(AllocatedFrames: DerefMut, Clone);

impl Deref for AllocatedFrames {
    type Target = FrameRange;
    fn deref(&self) -> &FrameRange {
        &self.frames
    }
}
impl fmt::Debug for AllocatedFrames {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "AllocatedFrames({:?})", self.frames)
    }
}

impl AllocatedFrames {
    /// Returns an empty AllocatedFrames object that performs no frame allocation. 
    /// Can be used as a placeholder, but will not permit any real usage. 
    pub const fn empty() -> AllocatedFrames {
        AllocatedFrames {
            frames: Chunk::empty()
        }
    }

    /// Merges the given `AllocatedFrames` object `other` into this `AllocatedFrames` object (`self`).
    /// This is just for convenience and usability purposes, it performs no allocation or remapping.
    ///
    /// The given `other` must be physically contiguous with `self`, i.e., come immediately before or after `self`.
    /// That is, either `self.start == other.end + 1` or `self.end + 1 == other.start` must be true. 
    ///
    /// If either of those conditions are met, `self` is modified and `Ok(())` is returned,
    /// otherwise `Err(other)` is returned.
    pub fn merge(&mut self, mut other: AllocatedFrames) -> Result<(), AllocatedFrames> {
        // if *self.start() == *other.end() + 1 {
        //     // `other` comes contiguously before `self`
        //     self.frames = FrameRange::new(*other.start(), *self.end());
        // } 
        // else if *self.end() + 1 == *other.start() {
        //     // `self` comes contiguously before `other`
        //     self.frames = FrameRange::new(*self.start(), *other.end());
        // }
        // else {
        //     // non-contiguous
        //     return Err(other);
        // }

        let mut chunk = core::mem::replace(&mut other.frames, Chunk::empty());
        match self.frames.merge(chunk) {
            Ok(_) => {
                // ensure the now-merged AllocatedFrames doesn't run its drop handler and free its frames.
                core::mem::forget(other); 
                Ok(())
            },
            Err(chunk) => {
                Err(AllocatedFrames{frames: chunk})
            }

        }

    }

    /// Splits this `AllocatedFrames` into two separate `AllocatedFrames` objects:
    /// * `[beginning : at_frame - 1]`
    /// * `[at_frame : end]`
    /// 
    /// This function follows the behavior of [`core::slice::split_at()`],
    /// thus, either one of the returned `AllocatedFrames` objects may be empty. 
    /// * If `at_frame == self.start`, the first returned `AllocatedFrames` object will be empty.
    /// * If `at_frame == self.end + 1`, the second returned `AllocatedFrames` object will be empty.
    /// 
    /// Returns an `Err` containing this `AllocatedFrames` if `at_frame` is otherwise out of bounds.
    /// 
    /// [`core::slice::split_at()`]: https://doc.rust-lang.org/core/primitive.slice.html#method.split_at
    pub fn split(mut self, at_frame: Frame) -> Result<(AllocatedFrames, AllocatedFrames), AllocatedFrames> {
        // let end_of_first = at_frame - 1;

        // let (first, second) = if at_frame == *self.start() && at_frame <= *self.end() {
        //     let first  = FrameRange::empty();
        //     let second = FrameRange::new(at_frame, *self.end());
        //     (first, second)
        // } 
        // else if at_frame == (*self.end() + 1) && end_of_first >= *self.start() {
        //     let first  = FrameRange::new(*self.start(), *self.end()); 
        //     let second = FrameRange::empty();
        //     (first, second)
        // }
        // else if at_frame > *self.start() && end_of_first <= *self.end() {
        //     let first  = FrameRange::new(*self.start(), end_of_first);
        //     let second = FrameRange::new(at_frame, *self.end());
        //     (first, second)
        // }
        // else {
        //     return Err(self);
        // };

        // ensure the original AllocatedFrames doesn't run its drop handler and free its frames.
        // core::mem::forget(self);   
        // Ok((
        //     AllocatedFrames { frames: first }, 
        //     AllocatedFrames { frames: second },
        // ))
        let mut chunk = core::mem::replace(&mut self.frames, Chunk::empty());
        match chunk.split_at(at_frame) {
            Ok((chunk1, chunk2)) => {
                // ensure the now-merged AllocatedFrames doesn't run its drop handler and free its frames.
                core::mem::forget(self); 
                Ok((
                    AllocatedFrames{frames: chunk1}, 
                    AllocatedFrames{frames: chunk2}
                ))
            },
            Err(chunk) => {
                Err(AllocatedFrames{frames: chunk})
            }

        }
    }

    /// Returns an `AllocatedFrame` if this `AllocatedFrames` object contains only one frame.
    /// 
    /// ## Panic
    /// Panics if this `AllocatedFrame` contains multiple frames or zero frames.
    pub fn as_allocated_frame(&self) -> AllocatedFrame {
        assert!(self.size_in_frames() == 1);
        AllocatedFrame {
            frame: *self.start(),
            _phantom: PhantomData,
        }
    }
}

/// This function is a callback used to convert `UnmappedFrames` into `AllocatedFrames`.
/// `UnmappedFrames` represents frames that have been unmapped from a page that had
/// exclusively mapped them, indicating that no others pages have been mapped 
/// to those same frames, and thus, they can be safely deallocated.
/// 
/// This exists to break the cyclic dependency cycle between this crate and
/// the `page_table_entry` crate, since `page_table_entry` must depend on types
/// from this crate in order to enforce safety when modifying page table entries.
fn into_allocated_frames(frames: FrameRange) -> AllocatedFrames {
    let typ = if frame_is_in_list(&RESERVED_REGIONS.lock(), frames.start()) {
        MemoryRegionType::Reserved
    } else {
        MemoryRegionType::Free
    };
    AllocatedFrames { frames: Chunk::trusted_new(typ, frames) }
}

impl Drop for AllocatedFrames {
    fn drop(&mut self) {
        if self.size_in_frames() == 0 { return; }

        let (list, typ) = if frame_is_in_list(&RESERVED_REGIONS.lock(), self.start()) {
            (&FREE_RESERVED_FRAMES_LIST, MemoryRegionType::Reserved)
        } else {
            (&FREE_GENERAL_FRAMES_LIST, MemoryRegionType::Free)
        };
        // trace!("frame_allocator: deallocating {:?}, typ {:?}", self, typ);

        // Simply add the newly-deallocated chunk to the free frames list.
        let mut locked_list = list.lock();
        let chunk = Chunk::empty();
        let res = locked_list.insert(core::mem::replace(&mut self.frames, chunk));
        match res {
            Ok(_inserted_free_chunk) => return,
            Err(c) => error!("BUG: couldn't insert deallocated chunk {:?} into free frame list", c),
        }
        
        // Here, we could optionally use above `_inserted_free_chunk` to merge the adjacent (contiguous) chunks
        // before or after the newly-inserted free chunk. 
        // However, there's no *need* to do so until we actually run out of address space or until 
        // a requested address is in a chunk that needs to be merged.
        // Thus, for performance, we save that for those future situations.
    }
}

impl<'f> IntoIterator for &'f AllocatedFrames {
    type IntoIter = AllocatedFramesIter<'f>;
    type Item = AllocatedFrame<'f>;
    fn into_iter(self) -> Self::IntoIter {
        AllocatedFramesIter {
            _owner: self,
            range: self.frames.clone().into_iter(),
        }
    }
}

/// An iterator over each [`AllocatedFrame`] in a range of [`AllocatedFrames`].
/// 
/// We must implement our own iterator type here in order to tie the lifetime `'f`
/// of a returned `AllocatedFrame<'f>` type to the lifetime of its containing `AllocatedFrames`.
/// This is because the underlying type of `AllocatedFrames` is a [`FrameRange`],
/// which itself is a [`core::ops::RangeInclusive`] of [`Frame`]s, and unfortunately the
/// `RangeInclusive` type doesn't implement an immutable iterator.
/// 
/// Iterating through a `RangeInclusive` actually modifies its own internal range,
/// so we must avoid doing that because it would break the semantics of a `FrameRange`.
/// In fact, this is why [`FrameRange`] only implements `IntoIterator` but
/// does not implement [`Iterator`] itself.
pub struct AllocatedFramesIter<'f> {
    _owner: &'f AllocatedFrames,
    range: RangeInclusiveIterator<Frame>,
}
impl<'f> Iterator for AllocatedFramesIter<'f> {
    type Item = AllocatedFrame<'f>;
    fn next(&mut self) -> Option<Self::Item> {
        self.range.next().map(|frame|
            AllocatedFrame {
                frame, _phantom: PhantomData,
            }
        )
    }
}

/// A reference to a single frame within a range of `AllocatedFrames`.
/// 
/// The lifetime of this type is tied to the lifetime of its owning `AllocatedFrames`.
#[derive(Debug)]
pub struct AllocatedFrame<'f> {
    frame: Frame,
    _phantom: PhantomData<&'f Frame>,
}
impl<'f> Deref for AllocatedFrame<'f> {
    type Target = Frame;
    fn deref(&self) -> &Self::Target {
        &self.frame
    }
}
assert_not_impl_any!(AllocatedFrame: DerefMut, Clone);


/// A series of pending actions related to frame allocator bookkeeping,
/// which may result in heap allocation. 
/// 
/// The actions are triggered upon dropping this struct. 
/// This struct can be returned from the `allocate_frames()` family of functions 
/// in order to allow the caller to precisely control when those actions 
/// that may result in heap allocation should occur. 
/// Such actions include adding chunks to lists of free frames or frames in use. 
/// 
/// The vast majority of use cases don't care about such precise control, 
/// so you can simply drop this struct at any time or ignore it
/// with a `let _ = ...` binding to instantly drop it. 
pub struct DeferredAllocAction<'list> {
    /// A reference to the list into which we will insert the free general-purpose `Chunk`s.
    free_list: &'list Mutex<StaticArrayRBTree<Chunk>>,
    /// A reference to the list into which we will insert the free "reserved" `Chunk`s.
    reserved_list: &'list Mutex<StaticArrayRBTree<Chunk>>,
    /// A free chunk that needs to be added back to the free list.
    free1: Chunk,
    /// Another free chunk that needs to be added back to the free list.
    free2: Chunk,
}
impl<'list> DeferredAllocAction<'list> {
    fn new<F1, F2>(free1: F1, free2: F2) -> DeferredAllocAction<'list> 
        where F1: Into<Option<Chunk>>,
              F2: Into<Option<Chunk>>,
    {
        let free1 = free1.into().unwrap_or_else(Chunk::empty);
        let free2 = free2.into().unwrap_or_else(Chunk::empty);
        DeferredAllocAction {
            free_list: &FREE_GENERAL_FRAMES_LIST,
            reserved_list: &FREE_RESERVED_FRAMES_LIST,
            free1,
            free2
        }
    }
}
impl<'list> Drop for DeferredAllocAction<'list> {
    fn drop(&mut self) {
        let mut chunk1 = Chunk::empty();
        let mut chunk2 = Chunk::empty();

        core::mem::swap(&mut chunk1, &mut self.free1);
        core::mem::swap(&mut chunk2, &mut self.free2);

        // Insert all of the chunks, both allocated and free ones, into the list. 
        if chunk1.size_in_frames() > 0 {
            match chunk1.typ() {
                MemoryRegionType::Free     => { self.free_list.lock().insert(chunk1).unwrap(); }
                MemoryRegionType::Reserved => { self.reserved_list.lock().insert(chunk1).unwrap(); }
                _ => error!("BUG likely: DeferredAllocAction encountered free1 chunk {:?} of a type Unknown", chunk1),
            }
        }
        if chunk2.size_in_frames() > 0 {
            match chunk2.typ() {
                MemoryRegionType::Free     => { self.free_list.lock().insert(chunk2).unwrap(); }
                MemoryRegionType::Reserved => { self.reserved_list.lock().insert(chunk2).unwrap(); }
                _ => error!("BUG likely: DeferredAllocAction encountered free2 chunk {:?} of a type Unknown", chunk2),
            };
        }
    }
}


/// Possible allocation errors.
#[derive(Debug)]
enum AllocationError {
    /// The requested address was not free: it was already allocated, or is outside the range of this allocator.
    AddressNotFree(Frame, usize),
    /// The address space was full, or there was not a large-enough chunk 
    /// or enough remaining chunks that could satisfy the requested allocation size.
    OutOfAddressSpace(usize),
    /// ToDo: remove
    InternalError,
}
impl From<AllocationError> for &'static str {
    fn from(alloc_err: AllocationError) -> &'static str {
        match alloc_err {
            AllocationError::AddressNotFree(..) => "address was in use or outside of this frame allocator's range",
            AllocationError::OutOfAddressSpace(..) => "out of physical address space",
            AllocationError::InternalError => "problem with frame allocato logic",
        }
    }
}


/// Searches the given `list` for the chunk that contains the range of frames from
/// `requested_frame` to `requested_frame + num_frames`.
fn find_specific_chunk(
    list: &mut StaticArrayRBTree<Chunk>,
    requested_frame: Frame,
    num_frames: usize
) -> Result<(AllocatedFrames, DeferredAllocAction<'static>), AllocationError> {

    // The end frame is an inclusive bound, hence the -1. Parentheses are needed to avoid overflow.
    let requested_end_frame = requested_frame + (num_frames - 1);

    match &mut list.0 {
        Inner::Array(ref mut arr) => {
            for elem in arr.iter_mut() {
                if let Some(chunk) = elem {
                    if requested_frame >= *chunk.start() && requested_end_frame <= *chunk.end() {
                        // Here: `chunk` was big enough and did contain the requested address.
                        return allocate_from_chosen_chunk(requested_frame, num_frames, ValueRefMut::Array(elem));
                    }
                }
            }
        }
        Inner::RBTree(ref mut tree) => {
            let mut contiguous_chunk_1 = None;
            let mut cursor_mut = tree.upper_bound_mut(Bound::Included(&requested_frame));
            if let Some(chunk) = cursor_mut.get().map(|w| w.deref().clone()) {
                if chunk.contains(&requested_frame) {
                    if requested_end_frame <= *chunk.end() {
                        return allocate_from_chosen_chunk(requested_frame, num_frames, ValueRefMut::RBTree(cursor_mut));
                    } else {
                        // We found the chunk containing the requested address, but it was too small to cover all of the requested frames.
                        // Let's try to merge the next-highest contiguous chunk to see if those two chunks together 
                        // cover enough frames to fulfill the allocation request.
                        //
                        // trace!("Frame allocator: found chunk containing requested address, but it was too small. \
                        //     Attempting to merge multiple chunks during an allocation. \
                        //     Requested address: {:?}, num_frames: {}, chunk: {:?}",
                        //     requested_frame, num_frames, chunk,
                        // );
                        // let next_contiguous_chunk: Option<Chunk> = {
                            let next_cursor = cursor_mut.peek_next();
                            if let Some(next_chunk) = next_cursor.get().map(|w| w.deref()) {
                                if *chunk.end() + 1 == *next_chunk.start() {
                                    // Here: next chunk was contiguous with the original chunk. 
                                    if requested_end_frame <= *next_chunk.end() {
                                        // trace!("Frame allocator: found suitably-large contiguous next {:?} after initial too-small {:?}", next_chunk, chunk);
                                        // Some(next_chunk.clone())
                                        contiguous_chunk_1 = Some(ValueRefMut::RBTree(cursor_mut));
                                    } else {
                                        todo!("Frame allocator: found chunk containing requested address, but it was too small. \
                                            Theseus does not yet support merging more than two chunks during an allocation request. \
                                            Requested address: {:?}, num_frames: {}, chunk: {:?}, next_chunk {:?}",
                                            requested_frame, num_frames, chunk, next_chunk
                                        );
                                        // None
                                    }
                                } else {
                                    trace!("Frame allocator: next {:?} was not contiguously above initial too-small {:?}", next_chunk, chunk);
                                    // None
                                }
                            } else {
                                trace!("Frame allocator: couldn't get next chunk above initial too-small {:?}", chunk);
                                // None
                            }
                        // };
                        if let Some(ref_chunk) = contiguous_chunk_1 {
                            let chunk1 = retrieve_chunk_from_ref(ref_chunk).ok_or(AllocationError::InternalError)?;
                            let requested_contiguous_frame = *chunk1.end() + 1;
                            let mut cursor_mut = tree.upper_bound_mut(Bound::Included(&requested_contiguous_frame));
                            if let Some(next_chunk) = cursor_mut.get().map(|w| w.deref()) {
                                if next_chunk.contains(&requested_contiguous_frame) {
                                    // let mut next_chunk_removed = retrieve_chunk_from_ref(ValueRefMut::RBTree(cursor_mut)).ok_or(AllocationError::InternalError)?;
                                    // next_chunk_removed.frames = FrameRange::new(*chunk1.start(), *next_chunk.end());
                                    return adjust_chosen_chunk_contiguous(requested_frame, num_frames, chunk1, ValueRefMut::RBTree(cursor_mut));
                                
                                } else {
                                    error!("chunk must contain range");
                                    return Err(AllocationError::InternalError);
                                }
                            }
                        // if let Some(mut next_chunk) = next_contiguous_chunk {
                        //     // We found a suitable chunk that came contiguously after the initial too-small chunk. 
                        //     // Remove the initial chunk (since we have a cursor pointing to it already) 
                        //     // and "merge" it into this `next_chunk`.
                        //     let _removed_initial_chunk = cursor_mut.remove();
                        //     // trace!("Frame allocator: removed suitably-large contiguous next {:?} after initial too-small {:?}", _removed_initial_chunk, chunk);
                        //     // Here, `cursor_mut` has been moved forward to point to the `next_chunk` now. 
                        //     next_chunk.frames = FrameRange::new(*chunk.start(), *next_chunk.end());
                        //     return allocate_from_chosen_chunk(requested_frame, num_frames, &next_chunk, ValueRefMut::RBTree(cursor_mut));
                        }
                    }
                }
            }
        }
    }

    Err(AllocationError::AddressNotFree(requested_frame, num_frames))
}


/// Searches the given `list` for any chunk large enough to hold at least `num_frames`.
fn find_any_chunk<'list>(
    list: &'list mut StaticArrayRBTree<Chunk>,
    num_frames: usize
) -> Result<(AllocatedFrames, DeferredAllocAction<'static>), AllocationError> {
    // During the first pass, we ignore designated regions.
    match list.0 {
        Inner::Array(ref mut arr) => {
            for elem in arr.iter_mut() {
                if let Some(chunk) = elem {
                    // Skip chunks that are too-small or in the designated regions.
                    if  chunk.size_in_frames() < num_frames || chunk.typ() != MemoryRegionType::Free {
                        continue;
                    } 
                    else {
                        return allocate_from_chosen_chunk(*chunk.start(), num_frames,  ValueRefMut::Array(elem));
                    }
                }
            }
        }
        Inner::RBTree(ref mut tree) => {
            // Because we allocate new frames by peeling them off from the beginning part of a chunk, 
            // it's MUCH faster to start the search for free frames from higher addresses moving down. 
            // This results in an O(1) allocation time in the general case, until all address ranges are already in use.
            let mut cursor = tree.upper_bound_mut(Bound::<&Chunk>::Unbounded);
            while let Some(chunk) = cursor.get().map(|w| w.deref()) {
                if num_frames <= chunk.size_in_frames() && chunk.typ() == MemoryRegionType::Free {
                    return allocate_from_chosen_chunk(*chunk.start(), num_frames, ValueRefMut::RBTree(cursor));
                }
                warn!("Frame allocator: inefficient scenario: had to search multiple chunks \
                    (skipping {:?}) while trying to allocate {} frames at any address.",
                    chunk, num_frames
                );
                cursor.move_prev();
            }
        }
    }

    error!("frame_allocator: non-reserved chunks are all allocated (requested {} frames). \
        TODO: we could attempt to merge free chunks here.", num_frames
    );

    Err(AllocationError::OutOfAddressSpace(num_frames))
}

fn retrieve_chunk_from_ref(mut chosen_chunk_ref: ValueRefMut<Chunk>) -> Option<Chunk> {
    // Remove the chosen chunk from the free frame list.
    let removed_val = chosen_chunk_ref.remove();
    
    let chosen_chunk = match removed_val {
        RemovedValue::Array(c) => c,
        RemovedValue::RBTree(option_chunk) => {
            if let Some(boxed_chunk) = option_chunk {  
                Some(boxed_chunk.into_inner())
            } else {
                None
            }
        }
    };
    chosen_chunk
}

/// The final part of the main allocation routine that splits the given chosen chunk
/// into multiple smaller chunks, thereby "allocating" frames from it.
///
/// This function breaks up that chunk into multiple ones and returns an `AllocatedFrames` 
/// from (part of) that chunk, ranging from `start_frame` to `start_frame + num_frames`.
fn allocate_from_chosen_chunk(
    start_frame: Frame,
    num_frames: usize,
    mut chosen_chunk_ref: ValueRefMut<Chunk>,
) -> Result<(AllocatedFrames, DeferredAllocAction<'static>), AllocationError> {
    let chosen_chunk = retrieve_chunk_from_ref(chosen_chunk_ref).ok_or(AllocationError::InternalError)?;

    let (new_allocation, before, after) = chosen_chunk.split(start_frame, num_frames);

    // TODO: Re-use the allocated wrapper if possible, rather than allocate a new one entirely.
    // if let RemovedValue::RBTree(Some(wrapper_adapter)) = _removed_chunk { ... }

    Ok((
        new_allocation.as_allocated_frames(),
        DeferredAllocAction::new(before, after),
    ))

}

/// The final part of the main allocation routine. 
///
/// The given chunk is the one we've chosen to allocate from. 
/// This function breaks up that chunk into multiple ones and returns an `AllocatedFrames` 
/// from (part of) that chunk, ranging from `start_frame` to `start_frame + num_frames`.
fn adjust_chosen_chunk_contiguous(
    start_frame: Frame,
    num_frames: usize,
    mut chunk1: Chunk,
    chunk2_ref: ValueRefMut<Chunk>,
) -> Result<(AllocatedFrames, DeferredAllocAction<'static>), AllocationError> {

    // TODO:: should merge
    let mut chunk2 = retrieve_chunk_from_ref(chunk2_ref).ok_or(AllocationError::InternalError)?;
    // chunk2.frames = FrameRange::new(*chunk1.start(), *chunk2.end());

    chunk1.merge(chunk2).expect("FRAME_ALLOCATOR: Failed to merge two chunks");
    let chosen_chunk = chunk1;

    let (new_allocation, before, after) = chosen_chunk.split(start_frame, num_frames);

    // TODO: Re-use the allocated wrapper if possible, rather than allocate a new one entirely.
    // if let RemovedValue::RBTree(Some(wrapper_adapter)) = _removed_chunk { ... }

    Ok((
        new_allocation.as_allocated_frames(),
        DeferredAllocAction::new(before, after),
    ))
}

/// Returns whether the given `Frame` is contained within the given `list`.
fn frame_is_in_list(
    list: &StaticArrayRBTree<Region>,
    frame: &Frame,
) -> bool {
    match &list.0 {
        Inner::Array(ref arr) => {
            for elem in arr.iter() {
                if let Some(chunk) = elem {
                    if chunk.contains(frame) { 
                        return true;
                    }
                }
            }
        }
        Inner::RBTree(ref tree) => {     
            let cursor = tree.upper_bound(Bound::Included(frame));
            if let Some(chunk) = cursor.get().map(|w| w.deref()) {
                if chunk.contains(frame) {
                    return true;
                }
            }
        }
    }

    false
}


/// Adds the given `frames` to the given `list` as a Chunk of reserved frames. 
/// 
/// Returns the range of **new** frames that were added to the list, 
/// which will be a subset of the given input `frames`.
///
/// Currently, this function adds no new frames at all if any frames within the given `frames` list
/// overlap any existing regions at all. 
/// TODO: handle partially-overlapping regions by extending existing regions on either end.
fn add_reserved_region(
    list: &mut StaticArrayRBTree<Chunk>,
    frames: FrameRange,
) -> Result<FrameRange, &'static str> {

    // Check whether the reserved region overlaps any existing regions.
    match &mut list.0 {
        Inner::Array(ref mut arr) => {
            for elem in arr.iter() {
                if let Some(chunk) = elem {
                    if let Some(_overlap) = chunk.overlap(&frames) {
                        // trace!("Failed to add reserved region {:?} due to overlap {:?} with existing chunk {:?}",
                        //     frames, _overlap, chunk
                        // );
                        return Err("Failed to add reserved region that overlapped with existing reserved regions (array).");
                    }
                }
            }
        }
        Inner::RBTree(ref mut tree) => {
            let mut cursor_mut = tree.upper_bound_mut(Bound::Included(frames.start()));
            while let Some(chunk) = cursor_mut.get().map(|w| w.deref()) {
                if chunk.start() > frames.end() {
                    // We're iterating in ascending order over a sorted tree,
                    // so we can stop looking for overlapping regions once we pass the end of the new frames to add.
                    break;
                }
                if let Some(_overlap) = chunk.overlap(&frames) {
                    // trace!("Failed to add reserved region {:?} due to overlap {:?} with existing chunk {:?}",
                    //     frames, _overlap, chunk
                    // );
                    return Err("Failed to add reserved region that overlapped with existing reserved regions (RBTree).");
                }
                cursor_mut.move_next();
            }
        }
    }

    list.insert(Chunk::new(
        MemoryRegionType::Reserved,
        frames.clone(),
    )?).map_err(|_c| "BUG: Failed to insert non-overlapping frames into list.")?;

    Ok(frames)
}


/// Adds the given `frames` to the given `list` as a Chunk of reserved frames. 
/// 
/// Returns the range of **new** frames that were added to the list, 
/// which will be a subset of the given input `frames`.
///
/// Currently, this function adds no new frames at all if any frames within the given `frames` list
/// overlap any existing regions at all. 
/// Handling partially-overlapping regions 
fn add_reserved_region_to_region_list(
    list: &mut StaticArrayRBTree<Region>,
    frames: FrameRange,
) -> Result<FrameRange, &'static str> {

    // Check whether the reserved region overlaps any existing regions.
    match &mut list.0 {
        Inner::Array(ref mut arr) => {
            for elem in arr.iter() {
                if let Some(chunk) = elem {
                    if let Some(_overlap) = chunk.overlap(&frames) {
                        // trace!("Failed to add reserved region {:?} due to overlap {:?} with existing chunk {:?}",
                        //     frames, _overlap, chunk
                        // );
                        return Err("Failed to add reserved region that overlapped with existing reserved regions (array).");
                    }
                }
            }
        }
        Inner::RBTree(ref mut tree) => {
            let mut cursor_mut = tree.upper_bound_mut(Bound::Included(frames.start()));
            while let Some(chunk) = cursor_mut.get().map(|w| w.deref()) {
                if chunk.start() > frames.end() {
                    // We're iterating in ascending order over a sorted tree,
                    // so we can stop looking for overlapping regions once we pass the end of the new frames to add.
                    break;
                }
                if let Some(_overlap) = chunk.overlap(&frames) {
                    // trace!("Failed to add reserved region {:?} due to overlap {:?} with existing chunk {:?}",
                    //     frames, _overlap, chunk
                    // );
                    return Err("Failed to add reserved region that overlapped with existing reserved regions (RBTree).");
                }
                cursor_mut.move_next();
            }
        }
    }

    list.insert(Region {
        typ: MemoryRegionType::Reserved,
        frames: frames.clone(),
    }).map_err(|_c| "BUG: Failed to insert non-overlapping frames into list.")?;

    Ok(frames)
}

/// The core frame allocation routine that allocates the given number of physical frames,
/// optionally at the requested starting `PhysicalAddress`.
/// 
/// This simply reserves a range of frames; it does not perform any memory mapping. 
/// Thus, the memory represented by the returned `AllocatedFrames` isn't directly accessible
/// until you map virtual pages to them.
/// 
/// Allocation is based on a red-black tree and is thus `O(log(n))`.
/// Fragmentation isn't cleaned up until we're out of address space, but that's not really a big deal.
/// 
/// # Arguments
/// * `requested_paddr`: if `Some`, the returned `AllocatedFrames` will start at the `Frame`
///   containing this `PhysicalAddress`. 
///   If `None`, the first available `Frame` range will be used, starting at any random physical address.
/// * `num_frames`: the number of `Frame`s to be allocated. 
/// 
/// # Return
/// If successful, returns a tuple of two items:
/// * the frames that were allocated, and
/// * an opaque struct representing details of bookkeeping-related actions that may cause heap allocation. 
///   Those actions are deferred until this returned `DeferredAllocAction` struct object is dropped, 
///   allowing the caller (such as the heap implementation itself) to control when heap allocation may occur.
pub fn allocate_frames_deferred(
    requested_paddr: Option<PhysicalAddress>,
    num_frames: usize,
) -> Result<(AllocatedFrames, DeferredAllocAction<'static>), &'static str> {
    if num_frames == 0 {
        warn!("frame_allocator: requested an allocation of 0 frames... stupid!");
        return Err("cannot allocate zero frames");
    }
    
    if let Some(paddr) = requested_paddr {
        let start_frame = Frame::containing_address(paddr);
        let end_frame = start_frame + (num_frames - 1);
        // Try to allocate the frames at the specific address.
        let mut free_reserved_frames_list = FREE_RESERVED_FRAMES_LIST.lock();
        if let Ok(success) = find_specific_chunk(&mut free_reserved_frames_list, start_frame, num_frames) {
            Ok(success)
        } else {
            // If allocation failed, then the requested `start_frame` may be found in the general-purpose list
            // or may represent a new, previously-unknown reserved region that we must add.
            // We first attempt to allocate it from the general-purpose free regions.
            if let Ok(result) = find_specific_chunk(&mut FREE_GENERAL_FRAMES_LIST.lock(), start_frame, num_frames) {
                Ok(result)
            } 
            // If we failed to allocate the requested frames from the general list,
            // we can add a new reserved region containing them,
            // but ONLY if those frames are *NOT* in the general-purpose region.
            else if {
                let g = GENERAL_REGIONS.lock();  
                !frame_is_in_list(&g, &start_frame) && !frame_is_in_list(&g, &end_frame)
            } {
                let frames = FrameRange::new(start_frame, end_frame);
                let new_reserved_frames = add_reserved_region_to_region_list(&mut RESERVED_REGIONS.lock(), frames)?;
                // If we successfully added a new reserved region,
                // then add those frames to the actual list of *available* reserved regions.
                let _new_free_reserved_frames = add_reserved_region(&mut free_reserved_frames_list, new_reserved_frames.clone())?;
                assert_eq!(new_reserved_frames, _new_free_reserved_frames);
                find_specific_chunk(&mut free_reserved_frames_list, start_frame, num_frames)
            } 
            else {
                Err(AllocationError::AddressNotFree(start_frame, num_frames))
            }
        }
    } else {
        find_any_chunk(&mut FREE_GENERAL_FRAMES_LIST.lock(), num_frames)
    }.map_err(From::from) // convert from AllocationError to &str
}


/// Similar to [`allocated_frames_deferred()`](fn.allocate_frames_deferred.html),
/// but accepts a size value for the allocated frames in number of bytes instead of number of frames. 
/// 
/// This function still allocates whole frames by rounding up the number of bytes. 
pub fn allocate_frames_by_bytes_deferred(
    requested_paddr: Option<PhysicalAddress>,
    num_bytes: usize,
) -> Result<(AllocatedFrames, DeferredAllocAction<'static>), &'static str> {
    let actual_num_bytes = if let Some(paddr) = requested_paddr {
        num_bytes + (paddr.value() % FRAME_SIZE)
    } else {
        num_bytes
    };
    let num_frames = (actual_num_bytes + FRAME_SIZE - 1) / FRAME_SIZE; // round up
    allocate_frames_deferred(requested_paddr, num_frames)
}


/// Allocates the given number of frames with no constraints on the starting physical address.
/// 
/// See [`allocate_frames_deferred()`](fn.allocate_frames_deferred.html) for more details. 
pub fn allocate_frames(num_frames: usize) -> Option<AllocatedFrames> {
    allocate_frames_deferred(None, num_frames)
        .map(|(af, _action)| af)
        .ok()
}


/// Allocates frames with no constraints on the starting physical address, 
/// with a size given by the number of bytes. 
/// 
/// This function still allocates whole frames by rounding up the number of bytes. 
/// See [`allocate_frames_deferred()`](fn.allocate_frames_deferred.html) for more details. 
pub fn allocate_frames_by_bytes(num_bytes: usize) -> Option<AllocatedFrames> {
    allocate_frames_by_bytes_deferred(None, num_bytes)
        .map(|(af, _action)| af)
        .ok()
}


/// Allocates frames starting at the given `PhysicalAddress` with a size given in number of bytes. 
/// 
/// This function still allocates whole frames by rounding up the number of bytes. 
/// See [`allocate_frames_deferred()`](fn.allocate_frames_deferred.html) for more details. 
pub fn allocate_frames_by_bytes_at(paddr: PhysicalAddress, num_bytes: usize) -> Result<AllocatedFrames, &'static str> {
    allocate_frames_by_bytes_deferred(Some(paddr), num_bytes)
        .map(|(af, _action)| af)
}


/// Allocates the given number of frames starting at (inclusive of) the frame containing the given `PhysicalAddress`.
/// 
/// See [`allocate_frames_deferred()`](fn.allocate_frames_deferred.html) for more details. 
pub fn allocate_frames_at(paddr: PhysicalAddress, num_frames: usize) -> Result<AllocatedFrames, &'static str> {
    allocate_frames_deferred(Some(paddr), num_frames)
        .map(|(af, _action)| af)
}


/// Converts the frame allocator from using static memory (a primitive array) to dynamically-allocated memory.
/// 
/// Call this function once heap allocation is available. 
/// Calling this multiple times is unnecessary but harmless, as it will do nothing after the first invocation.
#[doc(hidden)] 
pub fn convert_to_heap_allocated() {
    FREE_GENERAL_FRAMES_LIST.lock().convert_to_heap_allocated();
    FREE_RESERVED_FRAMES_LIST.lock().convert_to_heap_allocated();
    GENERAL_REGIONS.lock().convert_to_heap_allocated();
    RESERVED_REGIONS.lock().convert_to_heap_allocated();
}

/// A debugging function used to dump the full internal state of the frame allocator. 
#[doc(hidden)] 
pub fn dump_frame_allocator_state() {
    debug!("----------------- FREE GENERAL FRAMES ---------------");
    FREE_GENERAL_FRAMES_LIST.lock().iter().for_each(|e| debug!("\t {:?}", e) );
    debug!("-----------------------------------------------------");
    debug!("----------------- FREE RESERVED FRAMES --------------");
    FREE_RESERVED_FRAMES_LIST.lock().iter().for_each(|e| debug!("\t {:?}", e) );
    debug!("-----------------------------------------------------");
    debug!("------------------ GENERAL REGIONS -----------------");
    GENERAL_REGIONS.lock().iter().for_each(|e| debug!("\t {:?}", e) );
    debug!("-----------------------------------------------------");
    debug!("------------------ RESERVED REGIONS -----------------");
    RESERVED_REGIONS.lock().iter().for_each(|e| debug!("\t {:?}", e) );
    debug!("-----------------------------------------------------");
}
