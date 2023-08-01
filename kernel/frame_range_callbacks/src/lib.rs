#![no_std]
//! This crate contains callbacks to create `FrameChunk` objects and then `UnmappedFrames` objects from an `UnmappedFrames`.
//! It's required to avoid a cyclic dependency between the `frame_allocator` and `page_table_entry` crates. 
//! 
//! The public `from_unmapped()` function ensures that an `UnmappedFrames` object has to be consumed to run the callbacks,
/// making sure that it can only be called when a PTE has been unmapped.

extern crate page_table_entry;
extern crate frame_allocator;
extern crate trusted_chunk;
extern crate memory_structs;
extern crate spin;
extern crate range_inclusive;

use core::ops::{Deref};
use page_table_entry::UnmappedFrameRange;
use frame_allocator::UnmappedFrames;
use trusted_chunk::frame_chunk::FrameChunk;
use memory_structs::FrameRange;
use spin::Once;
use range_inclusive::RangeInclusive;

/// This is a private callback used to convert `UnmappedFrames` into a `FrameChunk`.
/// The `FrameChunk` is then used to create an `UnmappedFrames`.
/// 
/// This is safe because the init function in the `trusted_chunk` crate returns this callback only once,
/// and only this crate has access to the callback. The callback function has been verified with the 
/// invariant that the new `FrameChunk` has the same bounds as the range passed as an argument.
static INTO_TRUSTED_CHUNK_FUNC: Once<fn(FrameRange) -> FrameChunk> = Once::new();

/// This is a private callback used to convert `UnmappedFrames` into `UnmappedFrames`.
/// 
/// This exists to break the cyclic dependency cycle between `page_table_entry` and
/// `frame_allocator`, which depend on each other as such:
/// * `frame_allocator` needs to `impl Into<AllocatedPages> for UnmappedFrames`
///    in order to allow unmapped exclusive frames to be safely deallocated
/// * `page_table_entry` needs to use the `UnmappedFrames` type in order to allow
///   page table entry values to be set safely to a real physical frame that is owned and exists.
/// 
/// To get around that, the `frame_allocator::init()` function returns a callback
/// to its function that allows converting a range of unmapped frames back into `UnmappedFrames`,
/// which then allows them to be dropped and thus deallocated.
/// 
/// This is safe because the frame allocator can only be initialized once, and also because
/// only this crate has access to that function callback and can thus guarantee
/// that it is only invoked for `UnmappedFrames`.
static INTO_ALLOCATED_FRAMES_FUNC: Once<fn(FrameChunk, FrameRange) -> UnmappedFrames> = Once::new();

pub fn init(into_trusted_chunk_fn: fn(FrameRange) -> FrameChunk, into_alloc_frames_fn: fn(FrameChunk, FrameRange) -> UnmappedFrames) {
    INTO_TRUSTED_CHUNK_FUNC.call_once(|| into_trusted_chunk_fn);
    INTO_ALLOCATED_FRAMES_FUNC.call_once(|| into_alloc_frames_fn);
}

pub fn from_unmapped(unmapped_frames: UnmappedFrameRange) -> Result<UnmappedFrames, &'static str> {
    let frames = unmapped_frames.deref().clone();
    let tc = INTO_TRUSTED_CHUNK_FUNC.get()
        .ok_or("BUG: Mapper::unmap(): the `INTO_TRUSTED_CHUNK_FUNC` callback was not initialized")
        .map(|into_func| into_func(*unmapped_frames.deref()))?;

    INTO_ALLOCATED_FRAMES_FUNC.get()
        .ok_or("BUG: Mapper::unmap(): the `INTO_ALLOCATED_FRAMES_FUNC` callback was not initialized")
        .map(|into_func| into_func(tc, frames))
}