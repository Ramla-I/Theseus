#![no_std]
#![allow(dead_code)]
#![allow(unused_imports)]

#[macro_use(private_fields)] 
extern crate proc_static_assertions;

use prusti_contracts::*;
use memory_structs::{Page, PageRange};
use static_assertions::assert_not_impl_any;
use prusti_representation_creator::RepresentationCreator;
use prusti_external_spec::{trusted_option::*,trusted_result::*};
use core::ops::{Deref, DerefMut};
use proc_static_assertions::consumes;
use kernel_config::memory::{MAX_PAGE_NUMBER, MIN_PAGE_NUMBER};
use range_inclusive::*;

pub struct PageChunkCreator(RepresentationCreator<PageRange, PageChunk>);

impl PageChunkCreator {
    pub const fn new() -> Self { // To Do: This function should only be called once
        PageChunkCreator(RepresentationCreator::new(PageChunk::trusted_new, true))
    }
}

impl Deref for PageChunkCreator {
    type Target = RepresentationCreator<PageRange, PageChunk>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for PageChunkCreator {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// A struct representing an unallocated region in memory.
/// Its functions are formally verified to prevent range overlaps between chunks.
#[derive(PartialEq, Eq)]
#[private_fields("pages")]
pub struct PageChunk {
    pages: PageRange
}


assert_not_impl_any!(PageChunk: DerefMut, Clone);

impl PageChunk {
    #[pure]
    #[trusted]
    #[ensures(result == self.pages.start())]
    pub fn start(&self) -> &Page {
        self.pages.start()
    }

    #[pure]
    #[trusted]
    #[ensures(result == self.pages.end())]
    pub fn end(&self) -> &Page {
        self.pages.end()
    }

    #[ensures(result.is_empty())]
    pub const fn empty() -> PageChunk {
        PageChunk { pages: PageRange::empty() }
    }

    #[pure]
    pub fn is_empty(&self) -> bool {
        self.pages.is_empty()
    }

    pub const fn range(&self) -> PageRange {
        self.pages
    }

    /// Private function that creates a chunk without any checks.
    /// 
    /// Only used within other verified functions, or registered as a callback
    #[requires(pages.start_page() <= pages.end_page())]
    #[ensures(result.start() == pages.start())]
    #[ensures(result.end() == pages.end())]
    pub(crate) fn trusted_new(pages: &PageRange) -> PageChunk {
        PageChunk{ pages: *pages }
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
        ((split_range.0).is_some() ==> peek_option_ref(&split_range.0).end_page() == split_range.1.start_page() - 1)
        && ((split_range.2).is_some() ==> peek_option_ref(&split_range.2).start_page() == split_range.1.end_page() + 1)
    })]
    #[ensures(result.is_ok() ==> {
        let split_range = peek_result_ref(&result);
        ((split_range.0).is_some() ==> peek_option_ref(&split_range.0).start_page() == self.start_page())
        && ((split_range.0).is_none() ==> (split_range.1.start_page() == self.start_page() || (split_range.1.start_page().number() == MIN_PAGE_NUMBER)))
        && ((split_range.2).is_some() ==> peek_option_ref(&split_range.2).end_page() == self.end_page())
        && ((split_range.2).is_none() ==> ((split_range.1.end_page() == self.end_page()) || (split_range.1.end_page().number() == MAX_PAGE_NUMBER)))
    })]
    #[ensures(result.is_err() ==> {
        let orig_range = peek_err_ref(&result);
        (orig_range.start_page() == self.start_page()) && (orig_range.end_page() == self.end_page())
    })]
    #[consumes("self")]
    pub fn split_range(self, pages_to_extract: PageRange) -> Result<(Option<PageChunk>, PageChunk, Option<PageChunk>), PageChunk> {
        
        let (before, start_to_end, after) = match self.pages.split_range(pages_to_extract) {
            Ok(x) => x,
            Err(_) => return Err(self)
        };

        core::mem::forget(self);

        let before_start = match before {
            Some(x) => Some(PageChunk { pages: x }),
            None => None
        };

        let after_end = match after {
            Some(x) => Some(PageChunk { pages: x }),
            None => None
        };
        
        Ok((
            before_start,
            PageChunk { pages: start_to_end },
            after_end
        ))

    }


    /// Splits a chunk into 2 chunks at the page with number `at_page`.
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
        split_range.0.is_empty() ==> (split_range.1.start_page() == self.start_page() && split_range.1.end_page() == self.end_page())
    })]
    #[ensures(result.is_ok() ==> {
        let split_range = peek_result_ref(&result);
        split_range.1.is_empty() ==> (split_range.0.start_page() == self.start_page() && split_range.0.end_page() == self.end_page())
    })]
    #[ensures(result.is_ok() ==> {
        let split_range = peek_result_ref(&result);
        (!split_range.0.is_empty() && !split_range.1.is_empty()) ==> (
            split_range.0.start_page() == self.start_page() 
            && split_range.0.end_page() == at_page - 1
            && split_range.1.start_page() == at_page 
            && split_range.1.end_page() == self.end_page()
        )
    })]
    #[ensures(result.is_err() ==> {
        let orig_chunk = peek_err_ref(&result);
        (orig_chunk.start_page() == self.start_page()) && (orig_chunk.end_page() == self.end_page())
    })]
    #[consumes("self")]
    pub fn split_at(self, at_page: Page) -> Result<(PageChunk, PageChunk), PageChunk> {

        let (first, second) = match self.pages.split_at(at_page) {
            Ok(x) => x,
            Err(_) => return Err(self)
        };

        core::mem::forget(self);   
        Ok((PageChunk{ pages: first }, PageChunk{ pages: second }))
    }


    /// Merges `other` into `self`.
    /// Succeeds if `other` lies right before `self` or right after.
    /// 
    /// # Post-conditions:
    /// * If it succeeds, then `other` and `self` were contiguous, and either `self`'s start bound has been updated to `other`'s start 
    /// or `self`s end has been updated to `other`'s end
    /// * If it fails, then `self` remains unchanged and `other` is returned
    #[ensures(result.is_ok() ==> 
        (old(self.start_page()) == other.end_page() + 1 && self.start_page() == other.start_page() && self.end_page() == old(self.end_page())) 
        || 
        (old(self.end_page()) + 1 == other.start_page() && self.end_page() == other.end_page() && self.start_page() == old(self.start_page()))
    )]
    #[ensures(result.is_err() ==> {
        let chunk = peek_err_ref(&result);
        (chunk.start_page() == other.start_page()) && (chunk.end_page() == other.end_page()) 
    })]
    #[ensures(result.is_err() ==> {
        (self.start_page() == old(self.start_page())) && (self.end_page() == old(self.end_page())) 
    })]
    #[consumes("PageChunk")]
    pub fn merge(&mut self, other: PageChunk) -> Result<(), PageChunk> {
        if self.pages.merge(other.pages).is_ok() {
            core::mem::forget(other);
            Ok(())
        } else {
            Err(other)
        }
    }
}


impl Deref for PageChunk {
    type Target = PageRange;
    #[pure]
    fn deref(&self) -> &PageRange {
        &self.pages
    }
}