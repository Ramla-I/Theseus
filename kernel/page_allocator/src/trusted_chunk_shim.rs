//! A trusted wrapper over the verified Chunk.
//! Needed because verification fails on a trusted chunk that stores a PageRange or RangeInclusive<Page>, 
//! but succeeds with RangeInclusive<usize>.
//! 
//! We should be able to remove this module and work directly with the verified crate in the foreseeable future.
//! All this module should do is make sure that the start and end of the stored `pages` is equal to the start and end of the `verified_chunk`

use alloc::collections::btree_map::Range;
use kernel_config::memory::PAGE_SIZE;
use memory_structs::{PageRange, Page, VirtualAddress};
use range_inclusive::RangeInclusive;
use crate::{AllocatedPages, MIN_PAGE, MAX_PAGE};
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

#[derive(Debug, Eq)]
pub struct Chunk {
    /// The Pages covered by this chunk, an inclusive range. 
    pages: PageRange,
    /// The actual verified chunk
    verified_chunk: TrustedChunk
}

assert_not_impl_any!(Chunk: DerefMut, Clone);

impl Chunk {
    pub(crate) fn new(pages: PageRange) -> Result<Self, &'static str> {
        if pages.is_empty() {
            return Err("Sanity Check: empty chunks should be created with the empty() function");
        }

        let verified_chunk = if HEAP_INIT.is_completed() {
            TrustedChunk::new(pages.to_range_inclusive(), &mut CHUNK_LIST.lock()).map_err(|_| "Failed to create a verified chunk due to an overlap")?
        } else {
            TrustedChunk::new_pre_heap(pages.to_range_inclusive(), &mut CHUNK_ARRAY.lock())
                .map(|(chunk, _)| chunk)
                .map_err(|chunk_error|{
                    match chunk_error {
                        ChunkCreationError::Overlap(idx) => "Failed to create a verified chunk due to an overlap",
                        ChunkCreationError::NoSpace => "Before the heap is initialized, requested more chunks than there is space for (64)"
                    }
                })?
        };

        Ok(Chunk {
            pages,
            verified_chunk
        })
    }

    // pub(crate) fn trusted_new(typ: MemoryRegionType, pages: pageRange) -> Result<Chunk, &'static str> {
    //     INTO_VERIFIED_CHUNK_FUNC.get()
    //         .ok_or("into verified chunk function wasn't initialized")
    //         .map(|function| {
    //             let verified_chunk = function(pages.to_range_inclusive());
    //             Chunk { typ, pages, verified_chunk }
    //         })
    // }

    pub(crate) fn pages(&self) -> PageRange {
        self.pages.clone()
    }

    pub(crate) fn as_allocated_pages(self) -> AllocatedPages {
        AllocatedPages {
            pages: self,
        }
    }

    /// Returns a new `Chunk` with an empty range of pages. 
    pub(crate) const fn empty() -> Chunk {
        Chunk {
            pages: PageRange::empty(),
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

        // use the newly merged TrustedChunk to update the page range
        self.pages = into_page_range(&self.verified_chunk.frames());

        Ok(())
    }

    /// An inner function that breaks up the given chunk into multiple smaller chunks.
    /// 
    /// Returns a tuple of three chunks:
    /// 1. The `Chunk` containing the requested range of pages starting at `start_page`.
    /// 2. The range of pages in the `self` that came before the beginning of the requested page range.
    /// 3. The range of pages in the `self` that came after the end of the requested page range.
    pub fn split(
        mut self,
        start_page: Page,
        num_pages: usize,
    ) -> (Chunk, Option<Chunk>, Option<Chunk>) {
        if self.is_empty() {
            return (self, None, None);
        }

        // take out the TrustedChunk
        let verified_chunk = core::mem::replace(&mut self.verified_chunk, TrustedChunk::empty());

        let (before, new_allocation, after) = match verified_chunk.split(start_page.number(), num_pages) {
            Ok(x) => x,
            Err(vchunk) => {
                let _ = core::mem::replace(&mut self.verified_chunk, vchunk);
                return (self, None, None);
            }
        };

        (Chunk {
            pages: into_page_range(&new_allocation.frames()),
            verified_chunk: new_allocation
        },
        before.and_then(|vchunk| 
            Some(Chunk{
                pages: into_page_range(&vchunk.frames()),
                verified_chunk: vchunk
            })
        ), 
        after.and_then(|vchunk| 
            Some(Chunk{
                pages: into_page_range(&vchunk.frames()),
                verified_chunk: vchunk
            })
        ))
    }

    pub fn split_at(mut self, at_page: Page) -> Result<(Chunk, Chunk), Chunk> {
        if self.is_empty() {
            return Err(self);
        }

        // take out the TrustedChunk
        let verified_chunk = core::mem::replace(&mut self.verified_chunk, TrustedChunk::empty());

        let (first, second) = verified_chunk.split_at(at_page.number())
            .map_err(|vchunk| {
                let _ = core::mem::replace(&mut self.verified_chunk, vchunk);
                self
            })?;

        Ok((Chunk {
            pages: into_page_range(&first.frames()),
            verified_chunk: first
        },
        Chunk {
            pages: into_page_range(&second.frames()),
            verified_chunk: second
        }))
    }
}

impl Deref for Chunk {
    type Target = PageRange;
    fn deref(&self) -> &PageRange {
        &self.pages
    }
}
impl Ord for Chunk {
    fn cmp(&self, other: &Self) -> Ordering {
        self.pages.start().cmp(other.pages.start())
    }
}
impl PartialOrd for Chunk {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl PartialEq for Chunk {
    fn eq(&self, other: &Self) -> bool {
        self.pages.start() == other.pages.start()
    }
}
impl Borrow<Page> for &'_ Chunk {
    fn borrow(&self) -> &Page {
        self.pages.start()
    }
}


fn into_page_range(pages: &RangeInclusive<usize>) -> PageRange {
    let start = PageNum{ page: *pages.start() }.into_page()
        .expect("Verified chunk start was not a valid page");
    
    let end = PageNum{ page: *pages.end() }.into_page()
        .expect("Verified chunk end was not a valid page");
    
    PageRange::new(start, end)
}

struct PageNum {
    page: usize
}

impl PageNum {
    fn into_page(&self) -> Option<Page> {
        VirtualAddress::new(self.page * PAGE_SIZE)
            .and_then(|addr| Some(Page::containing_address(addr)))
    }
}