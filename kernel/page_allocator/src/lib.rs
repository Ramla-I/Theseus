//! Provides an allocator for virtual memory pages.
//! The minimum unit of allocation is a single page. 
//! 
//! This also supports early allocation of pages (up to 32 separate chunks)
//! before heap allocation is available, and does so behind the scenes using the same single interface. 
//! 
//! Once heap allocation is available, it uses a dynamically-allocated list of page chunks to track allocations.
//! 
//! The core allocation function is [`allocate_pages_deferred()`](fn.allocate_pages_deferred.html), 
//! but there are several convenience functions that offer simpler interfaces for general usage. 
//!
//! # Notes
//! This allocator only makes one attempt to merge deallocated pages into existing
//! free chunks for de-fragmentation. It does not iteratively merge adjacent chunks in order to
//! maximally combine separate chunks into the biggest single chunk.
//! Instead, free chunks are lazily merged only when running out of address space
//! or when needed to fulfill a specific request.

#![no_std]
#![feature(adt_const_params)]
#![allow(incomplete_features)]


extern crate alloc;
#[macro_use] extern crate log;
extern crate kernel_config;
extern crate memory_structs;
extern crate spin;
#[macro_use] extern crate static_assertions;
extern crate intrusive_collections;
use intrusive_collections::Bound;

mod static_array_rb_tree;
// mod static_array_linked_list;

use core::{borrow::Borrow, cmp::{Ordering, max, min}, fmt, ops::{Deref, DerefMut}};
use kernel_config::memory::*;
use memory_structs::{VirtualAddress, Page, PageRange, MemoryState};
use spin::{Mutex, Once};
use static_array_rb_tree::*;


/// Certain regions are pre-designated for special usage, specifically the kernel's initial identity mapping.
/// They will be allocated from if an address within them is specifically requested;
/// otherwise, they will only be allocated from as a "last resort" if all other non-designated address ranges are exhausted.
///
/// Any virtual addresses **less than or equal** to this address are considered "designated".
/// This lower part of the address range that's designated covers from 0x0 to this address.
static DESIGNATED_PAGES_LOW_END: Once<Page> = Once::new();

/// Defines the upper part of the address space that's designated, similar to `DESIGNATED_PAGES_LOW_END`. 
/// Any virtual addresses **greater than or equal to** this address is considered "designated".
/// This higher part of the address range covers from:
/// the beginning of the recursive P4 entry used for modifying upcoming page tables
/// to the very end of the address space.
///
/// TODO: once the heap is fully dynamic and not dependent on static addresses,
/// we can exclude the heap from the designated region.
static DESIGNATED_PAGES_HIGH_START: Page = Page::containing_address(
	VirtualAddress::new_canonical(UPCOMING_PAGE_TABLE_RECURSIVE_P4_START)
);

const MIN_PAGE: Page = Page::containing_address(VirtualAddress::zero());
const MAX_PAGE: Page = Page::containing_address(VirtualAddress::new_canonical(MAX_VIRTUAL_ADDRESS));

/// The single, system-wide list of free chunks of virtual memory pages.
static FREE_PAGE_LIST: Mutex<StaticArrayRBTree<FreePages>> = Mutex::new(StaticArrayRBTree::empty());


/// Initialize the page allocator.
///
/// # Arguments
/// * `end_vaddr_of_low_designated_region`: the `VirtualAddress` that marks the end of the 
///   lower designated region, which should be the ending address of the initial kernel image
///   (a lower-half identity address).
/// 
/// The page allocator considers two regions as "designated" regions. It will only allocate pages
/// within these designated regions if the specifically-requested address falls within them.
/// 1. The lower designated region is for identity-mapped bootloader content
///    and base kernel image sections, which is used during OS initialization.
/// 2. The higher designated region is for the same content, mapped to the higher half
///    of the address space. It also excludes the address ranges for the P4 entries that
///    Theseus uses for recursive page table mapping.
///    * See [`RECURSIVE_P4_INDEX`] and [`UPCOMING_PAGE_TABLE_RECURSIVE_P4_INDEX`].
///
/// General allocation requests for pages at any virtual address will not use
/// addresses within designated regions unless the entire address space is already in use,
/// which is an extraordinarily unlikely (i.e., basically impossible) situation.
pub fn init(end_vaddr_of_low_designated_region: VirtualAddress) -> Result<(), &'static str> {
	assert!(end_vaddr_of_low_designated_region < DESIGNATED_PAGES_HIGH_START.start_address());
	let designated_low_end_page = DESIGNATED_PAGES_LOW_END.call_once(
		|| Page::containing_address(end_vaddr_of_low_designated_region)
	);
	let designated_low_end = *designated_low_end_page;

	let initial_free_chunks = [
		// The first region contains all pages from address zero to the end of the low designated region,
		// which is generally reserved for identity-mapped bootloader stuff and base kernel image sections.
		Some(FreePages::new(
			PageRange::new(
				Page::containing_address(VirtualAddress::zero()),
				designated_low_end,
			)
		)),
		// The second region contains the massive range from the end of the low designated region
		// to the beginning of the high designated region, which comprises the majority of the address space.
		// The beginning of the high designated region starts at the reserved P4 entry used to
		// recursively map the "upcoming" page table (i.e., UPCOMING_PAGE_TABLE_RECURSIVE_P4_INDEX).
		Some(FreePages::new(
			PageRange::new(
				designated_low_end + 1,
				DESIGNATED_PAGES_HIGH_START - 1,
			)
		)),
		// Here, we skip the addresses covered by the `UPCOMING_PAGE_TABLE_RECURSIVE_P4_INDEX`.

		// The third region contains the range of addresses reserved for the heap,
		// which ends at the beginning of the addresses covered by the `RECURSIVE_P4_INDEX`,
		Some(FreePages::new(
			PageRange::new(
				Page::containing_address(VirtualAddress::new_canonical(KERNEL_HEAP_START)),
				// This is the page right below the beginning of the 510th entry of the top-level P4 page table.
				Page::containing_address(VirtualAddress::new_canonical(RECURSIVE_P4_START - 1)),
			)
		)),
		// Here, we skip the addresses covered by the `RECURSIVE_P4_INDEX`.

		// The fourth region contains all pages in the 511th (last) entry of P4.
		Some(FreePages::new(
			PageRange::new(
				Page::containing_address(VirtualAddress::new_canonical(KERNEL_TEXT_START)),
				MAX_PAGE,
			)
		)),
		None, None, None, None,
		None, None, None, None, None, None, None, None,
		None, None, None, None, None, None, None, None,
		None, None, None, None, None, None, None, None,
	];

	*FREE_PAGE_LIST.lock() = StaticArrayRBTree::new(initial_free_chunks);
	Ok(())
}


/// Represents a range of allocated `VirtualAddress`es, specified in `Page`s. 
/// 
/// These pages are not initially mapped to any physical memory frames, you must do that separately
/// in order to actually use their memory; see the `MappedPages` type for more. 
/// 
/// This object represents ownership of the allocated virtual pages;
/// if this object falls out of scope, its allocated pages will be auto-deallocated upon drop. 
/// A range of contiguous pages.
///
/// # Ordering and Equality
///
/// `Chunk` implements the `Ord` trait, and its total ordering is ONLY based on
/// its **starting** `Page`. This is useful so we can store `Chunk`s in a sorted collection.
///
/// Similarly, `Chunk` implements equality traits, `Eq` and `PartialEq`,
/// both of which are also based ONLY on the **starting** `Page` of the `Chunk`.
/// Thus, comparing two `Chunk`s with the `==` or `!=` operators may not work as expected.
/// since it ignores their actual range of pages.
#[derive(Eq)]
pub struct Pages<const S: MemoryState> {
	/// The Pages covered by this chunk, an inclusive range. 
	pages: PageRange,
}

/// A type alias for `Pages` in the `Free` state.
pub type FreePages = Pages<{MemoryState::Free}>;
/// A type alias for `Pages` in the `Allocated` state.
pub type AllocatedPages = Pages<{MemoryState::Allocated}>;
/// A type alias for `Pages` in the `Mapped` state.
pub type PagesMapped = Pages<{MemoryState::Mapped}>;
/// A type alias for `Pages` in the `Unmapped` state.
pub type UnmappedPages = Pages<{MemoryState::Unmapped}>;


// Pages must not be Cloneable, and it must not expose its inner Pages as mutable.
assert_not_impl_any!(Pages<{MemoryState::Free}>: DerefMut, Clone);
assert_not_impl_any!(Pages<{MemoryState::Allocated}>: DerefMut, Clone);
assert_not_impl_any!(Pages<{MemoryState::Mapped}>: DerefMut, Clone);
assert_not_impl_any!(Pages<{MemoryState::Unmapped}>: DerefMut, Clone);


impl FreePages {
	/// Creates a new `Pages` object in the `Free` state.
    ///
    /// The page allocator logic is responsible for ensuring that no two `Pages` objects overlap.
    pub(crate) fn new(pages: PageRange) -> Self {
        Pages {
            pages,
        }
    }

    /// Consumes this `Pages` in the `Free` state and converts them into the `Allocated` state.
    pub fn into_allocated_pages(self) -> AllocatedPages {    
        let f = Pages {
            pages: self.pages.clone(),
        };
        core::mem::forget(self);
        f
    }
}

impl AllocatedPages {
    /// Consumes this `Frames` in the `Allocated` state and converts them into the `Mapped` state.
    /// This should only be called once a `MappedPages` has been created from the `Frames`.
    pub fn into_mapped_pages(self) -> PagesMapped {    
        let f = Pages {
            pages: self.pages.clone(),
        };
        core::mem::forget(self);
        f
    }
}

impl UnmappedPages {
    /// Consumes this `Frames` in the `Unmapped` state and converts them into the `Allocated` state.
    pub fn into_allocated_pages(self) -> AllocatedPages {    
        let f = Pages {
            pages: self.pages.clone(),
        };
        core::mem::forget(self);
        f
    }
}

impl PagesMapped {
    /// Consumes this `Frames` in the `Unmapped` state and converts them into the `Allocated` state.
    pub fn into_unmapped_pages(self) -> UnmappedPages {    
        let f = Pages {
            pages: self.pages.clone(),
        };
        core::mem::forget(self);
        f
    }
}

impl<const S: MemoryState> Deref for Pages<S> {
    type Target = PageRange;
    fn deref(&self) -> &PageRange {
        &self.pages
    }
}
impl<const S: MemoryState> Ord for Pages<S> {
    fn cmp(&self, other: &Self) -> Ordering {
        self.pages.start().cmp(other.pages.start())
    }
}
impl<const S: MemoryState> PartialOrd for Pages<S> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl<const S: MemoryState> PartialEq for Pages<S> {
    fn eq(&self, other: &Self) -> bool {
        self.pages.start() == other.pages.start()
    }
}
impl<const S: MemoryState> Borrow<Page> for &'_ Pages<S> {
	fn borrow(&self) -> &Page {
		self.pages.start()
	}
}
impl<const S: MemoryState> fmt::Debug for Pages <S>{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
		write!(f, "Pages({:?})", self.pages)
	}
}

/// The result of splitting a `Pages` object into multiple smaller `Pages` objects.
pub struct SplitPages<const S: MemoryState>  {
    before_start:   Option<Pages<S>>,
    start_to_end:   Pages<S>,
    after_end:      Option<Pages<S>>,
}

impl<const S: MemoryState> Pages<S> {
	/// Returns an empty `Pages` object that performs no page allocation. 
    /// Can be used as a placeholder, but will not permit any real usage. 
    pub const fn empty() -> Pages<S> {
        Pages {
			pages: PageRange::empty()
		}
	}

	/// Returns the starting `VirtualAddress` in this range of pages.
    pub fn start_address(&self) -> VirtualAddress {
        self.pages.start_address()
    }

	/// Returns the size in bytes of this range of pages.
    pub fn size_in_bytes(&self) -> usize {
        self.pages.size_in_bytes()
    }

	/// Returns the size in number of pages of this range of pages.
    pub fn size_in_pages(&self) -> usize {
        self.pages.size_in_pages()
    }

	/// Returns the starting `Page` in this range of pages.
	pub fn start(&self) -> &Page {
		self.pages.start()
	}

	/// Returns the ending `Page` (inclusive) in this range of pages.
	pub fn end(&self) -> &Page {
		self.pages.end()
	}

	/// Returns a reference to the inner `PageRange`, which is cloneable/iterable.
	pub fn range(&self) -> &PageRange {
		&self.pages
	}

	/// Returns the offset of the given `VirtualAddress` within this range of pages,
	/// i.e., `addr - self.start_address()`.
	///
	/// If the given `addr` is not covered by this range of pages, this returns `None`.
	///
	/// ## Examples
	/// If the range covers addresses `0x2000` to `0x4000`,
	/// then `offset_of_address(0x3500)` would return `Some(0x1500)`.
	pub const fn offset_of_address(&self, addr: VirtualAddress) -> Option<usize> {
		self.pages.offset_of_address(addr)
	}

	/// Returns the `VirtualAddress` at the given offset into this range of pages,
	/// i.e., `self.start_address() + offset`.
	///
	/// If the given `offset` is not within this range of pages, this returns `None`.
	///
	/// ## Examples
	/// If the range covers addresses `0x2000` through `0x3FFF`,
	/// then `address_at_offset(0x1500)` would return `Some(0x3500)`,
	/// and `address_at_offset(0x2000)` would return `None`.
	pub const fn address_at_offset(&self, offset: usize) -> Option<VirtualAddress> {
		self.pages.address_at_offset(offset)
	}

	/// Merges the given `AllocatedPages` object `ap` into this `AllocatedPages` object (`self`).
	/// This is just for convenience and usability purposes, it performs no allocation or remapping.
    ///
	/// The `ap` must be virtually contiguous and come immediately after `self`,
	/// that is, `self.end` must equal `ap.start`. 
	/// If this condition is met, `self` is modified and `Ok(())` is returned,
	/// otherwise `Err(ap)` is returned.
	pub fn merge(&mut self, ap: Pages<S>) -> Result<(), Pages<S>> {
		// make sure the pages are contiguous
		if *ap.start() != (*self.end() + 1) {
			return Err(ap);
		}
		self.pages = PageRange::new(*self.start(), *ap.end());
		// ensure the now-merged AllocatedPages doesn't run its drop handler and free its pages.
		core::mem::forget(ap); 
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
    pub fn split_range(
        self,
        pages_to_extract: PageRange
    ) -> Result<SplitPages<S>, Self> {
        
        if !self.contains_range(&pages_to_extract) {
            return Err(self);
        }
        
        let start_page = *pages_to_extract.start();
        let start_to_end = Pages { pages: pages_to_extract, ..self };
        
        let before_start = if start_page == MIN_PAGE || start_page == *self.start() {
            None
        } else {
            Some(Pages { pages: PageRange::new(*self.start(), *start_to_end.start() - 1), ..self })
        };

        let after_end = if *start_to_end.end() == MAX_PAGE || *start_to_end.end() == *self.end() {
            None
        } else {
            Some(Pages { pages: PageRange::new(*start_to_end.end() + 1, *self.end()), ..self })
        };

        core::mem::forget(self);
        Ok(SplitPages { before_start, start_to_end, after_end })
    }


	/// Splits this `AllocatedPages` into two separate `AllocatedPages` objects:
    /// * `[beginning : at_page - 1]`
    /// * `[at_page : end]`
    /// 
    /// This function follows the behavior of [`core::slice::split_at()`],
    /// thus, either one of the returned `AllocatedPages` objects may be empty. 
    /// * If `at_page == self.start`, the first returned `AllocatedPages` object will be empty.
    /// * If `at_page == self.end + 1`, the second returned `AllocatedPages` object will be empty.
    /// 
    /// Returns an `Err` containing this `AllocatedPages` if `at_page` is otherwise out of bounds.
	/// 
    /// [`core::slice::split_at()`]: https://doc.rust-lang.org/core/primitive.slice.html#method.split_at
    pub fn split_at(self, at_page: Page) -> Result<(Self, Self), Self> {
        if self.is_empty() { return Err(self); }
        
		let end_of_first = at_page - 1;

        let (first, second) = if at_page == *self.start() && at_page <= *self.end() {
            let first  = PageRange::empty();
            let second = PageRange::new(at_page, *self.end());
            (first, second)
        } 
        else if at_page == (*self.end() + 1) && end_of_first >= *self.start() {
            let first  = PageRange::new(*self.start(), *self.end()); 
            let second = PageRange::empty();
            (first, second)
        }
        else if at_page > *self.start() && end_of_first <= *self.end() {
            let first  = PageRange::new(*self.start(), end_of_first);
            let second = PageRange::new(at_page, *self.end());
            (first, second)
        }
        else {
            return Err(self);
        };

        // ensure the original AllocatedPages doesn't run its drop handler and free its pages.
        core::mem::forget(self);   
        Ok((
            Pages { pages: first }, 
            Pages { pages: second },
        ))
    }
}

impl<const S: MemoryState> Drop for Pages<S> {
    fn drop(&mut self) {
		match S {
			MemoryState::Free => {
				if self.size_in_pages() == 0 { return; }
				// trace!("page_allocator: deallocating {:?}", self);
	
				let mut free_pages = Pages {
					pages: self.pages.clone(),
				};
				let mut list = FREE_PAGE_LIST.lock();
				match &mut list.0 {
					// For early allocations, just add the deallocated chunk to the free pages list.
					Inner::Array(_) => {
						if list.insert(free_pages).is_ok() {
							return;
						}
					}
	
					// For full-fledged deallocations, use the entry API to efficiently determine if
					// we can merge the deallocated pages with an existing contiguously-adjactent chunk
					// or if we need to insert a new chunk.
					Inner::RBTree(ref mut tree) => {
						let mut cursor_mut = tree.lower_bound_mut(Bound::Included(free_pages.start()));
						if let Some(next_pages_ref) = cursor_mut.get() {
							if *free_pages.end() + 1 == *next_pages_ref.start() {
								// extract the next chunk from the list
                                let mut next_pages: FreePages = cursor_mut
                                    .replace_with(Wrapper::new_link(Pages::empty()))
                                    .expect("BUG: couldn't remove next pages from free list in drop handler")
									.into_inner();						
			
								// trace!("Prepending {:?} onto beg of next {:?}", free_pages, next_pages.deref());
								if free_pages.merge(next_pages).is_ok() {
                                    // trace!("newly merged next chunk: {:?}", next_pages);
                                    // now return newly merged chunk into list
                                    match cursor_mut.replace_with(Wrapper::new_link(free_pages)) { 
                                        Ok(_) => { return; }
                                        Err(f) => free_pages = f.into_inner(), 
                                    }
                                } else {
                                    panic!("BUG: couldn't merge deallocated chunk into next chunk");
                                }
							}
						}
						if let Some(prev_pages_ref) = cursor_mut.peek_prev().get() {
							if *prev_pages_ref.end() + 1 == *free_pages.start() {
								// trace!("Appending {:?} onto end of prev {:?}", free_pages, prev_pages.deref());
								cursor_mut.move_prev();
								if let Some(_prev_pages_ref) = cursor_mut.get() {
                                    // extract the next chunk from the list
                                    let mut prev_pages = cursor_mut
                                        .replace_with(Wrapper::new_link(Pages::empty()))
                                        .expect("BUG: couldn't remove next frames from free list in drop handler")
                                        .into_inner();

                                    if prev_pages.merge(free_pages).is_ok() {
                                        // trace!("newly merged prev chunk: {:?}", prev_pages);
                                        // now return newly merged chunk into list
                                        match cursor_mut.replace_with(Wrapper::new_link(prev_pages)) { 
                                            Ok(_) => { return; }
                                            Err(f) => free_pages = f.into_inner(), 
                                        }
                                    } else {
                                        panic!("BUG: couldn't merge deallocated chunk into prev chunk");
                                    }
                                }
							}
						}
	
						// trace!("Inserting new chunk for deallocated {:?} ", free_pages.pages);
						cursor_mut.insert(Wrapper::new_link(free_pages));
						return;
					}
				}
				log::error!("BUG: couldn't insert deallocated {:?} into free page list", self.pages);
			},
            MemoryState::Allocated => { 
                // trace!("Converting AllocatedPages to FreePages. Drop handler will be called again {:?}", self.pages);
                let _to_drop = FreePages::new(self.pages.clone()); 
            }
            MemoryState::Mapped => {
			    let _to_drop = UnmappedPages { pages: self.pages.clone() };
			}	
            MemoryState::Unmapped => {
                let _to_drop = AllocatedPages { pages: self.pages.clone() };
            }

		}
    }
}



/// A series of pending actions related to page allocator bookkeeping,
/// which may result in heap allocation. 
/// 
/// The actions are triggered upon dropping this struct. 
/// This struct can be returned from the `allocate_pages()` family of functions 
/// in order to allow the caller to precisely control when those actions 
/// that may result in heap allocation should occur. 
/// Such actions include adding chunks to lists of free pages or pages in use. 
/// 
/// The vast majority of use cases don't care about such precise control, 
/// so you can simply drop this struct at any time or ignore it
/// with a `let _ = ...` binding to instantly drop it. 
pub struct DeferredAllocAction<'list> {
	/// A reference to the list into which we will insert the free `Chunk`s.
	free_list: &'list Mutex<StaticArrayRBTree<FreePages>>,
	/// A free chunk that needs to be added back to the free list.
	free1: FreePages,
	/// Another free chunk that needs to be added back to the free list.
	free2: FreePages,
}
impl<'list> DeferredAllocAction<'list> {
	fn new<F1, F2>(free1: F1, free2: F2) -> DeferredAllocAction<'list> 
		where F1: Into<Option<FreePages>>,
			  F2: Into<Option<FreePages>>,
	{
		let free_list = &FREE_PAGE_LIST;
		let free1 = free1.into().unwrap_or_else(FreePages::empty);
		let free2 = free2.into().unwrap_or_else(FreePages::empty);
		DeferredAllocAction { free_list, free1, free2 }
	}
}
impl<'list> Drop for DeferredAllocAction<'list> {
	fn drop(&mut self) {
		let pages1 = core::mem::replace(&mut self.free1, Pages::empty());
        let pages2 = core::mem::replace(&mut self.free2, Pages::empty());
        
		// Insert all of the chunks, both allocated and free ones, into the list. 
		if pages1.size_in_pages() > 0 {
			self.free_list.lock().insert(pages1).unwrap();
		}
		if pages2.size_in_pages() > 0 {
			self.free_list.lock().insert(pages2).unwrap();
		}
	}
}


/// Possible errors returned by the page allocator.
#[derive(Debug)]
pub enum AllocationError {
	/// The requested address was not free: it was already allocated, or is outside the range of this allocator.
	AddressNotFree(Page, usize),
	/// The address space was full, or there was not a large-enough chunk 
	/// or enough remaining chunks (within the given `PageRange`, if any)
	/// that could satisfy the requested allocation size.
	OutOfAddressSpace(usize, Option<PageRange>),
	/// The allocator has not yet been initialized.
	NotInitialized,
}
impl From<AllocationError> for &'static str {
	fn from(alloc_err: AllocationError) -> &'static str {
		match alloc_err {
			AllocationError::AddressNotFree(..) => "address was in use or outside of this page allocator's range",
			AllocationError::OutOfAddressSpace(_, Some(_range)) => "out of virtual address space in specified range",
			AllocationError::OutOfAddressSpace(_, None) => "out of virtual address space",
			AllocationError::NotInitialized => "the page allocator has not yet been initialized",
		}
	}
}


/// Searches the given `list` for the chunk that contains the range of pages from
/// `requested_page` to `requested_page + num_pages`.
fn find_specific_chunk(
	list: &mut StaticArrayRBTree<FreePages>,
	requested_page: Page,
	num_pages: usize
) -> Result<(AllocatedPages, DeferredAllocAction<'static>), AllocationError> {

	// The end page is an inclusive bound, hence the -1. Parentheses are needed to avoid overflow.
	let requested_end_page = requested_page + (num_pages - 1); 

	match &mut list.0 {
		Inner::Array(ref mut arr) => {
			for elem in arr.iter_mut() {
				if let Some(chunk) = elem {
					if requested_page >= *chunk.start() && requested_end_page <= *chunk.end() {
						// Here: `chunk` was big enough and did contain the requested address.
						// warn!("1 {:?}, num_pages = {}", requested_page, num_pages);
						// warn!("1 {:?}", requested_page + num_pages);
						// warn!("1 {:?}", requested_page + num_pages -1);
						return adjust_chosen_chunk(PageRange::new(requested_page, requested_page + (num_pages - 1)), ValueRefMut::Array(elem));
					}
				}
			}
		}
		Inner::RBTree(ref mut tree) => {
			let mut cursor_mut = tree.upper_bound_mut(Bound::Included(&requested_page));
			if let Some(chunk) = cursor_mut.get().map(|w| w.deref()) {
				if requested_page >= *chunk.start() {
					if requested_end_page <= *chunk.end() {
						// warn!("2 {:?}, num_pages = {}", requested_page, num_pages);
						return adjust_chosen_chunk(PageRange::new(requested_page, requested_page + (num_pages - 1)), ValueRefMut::RBTree(cursor_mut));
					} else {
						// // Here, we've found a chunk that includes the requested start page, but it's too small
						// // to cover the number of requested pages. 
						// // Thus, we attempt to merge this chunk with the next contiguous chunk(s) to create one single larger chunk.
						// let chunk = chunk.clone(); // ends the above borrow on `cursor_mut`
						// let mut new_end_page = *chunk.end();
						// cursor_mut.move_next();
						// while let Some(next_chunk) = cursor_mut.get().map(|w| w.deref()) {
						// 	if *next_chunk.start() - 1 == new_end_page {
						// 		new_end_page = *next_chunk.end();
						// 		cursor_mut.remove().expect("BUG: page_allocator failed to merge contiguous chunks.");
						// 		// The above call to `cursor_mut.remove()` advances the cursor to the next chunk.
						// 	} else {
						// 		break; // the next chunk wasn't contiguous, so stop iterating.
						// 	}
						// }

						// if new_end_page > *chunk.end() {
						// 	cursor_mut.move_prev(); // move the cursor back to the original chunk
						// 	let _removed_chunk = cursor_mut.replace_with(Wrapper::new_link(Chunk { pages: PageRange::new(*chunk.start(), new_end_page) }))
						// 		.expect("BUG: page_allocator failed to replace the current chunk while merging contiguous chunks.");
						// 	return adjust_chosen_chunk(PageRange::new(requested_page, requested_page + num_pages - 1), ValueRefMut::RBTree(cursor_mut));
						// }
						
						let mut first_chunk = cursor_mut.remove().expect("BUG: page_allocator failed to merge contiguous chunks.");
						let mut found_contiguous = true;
						let mut new_end_page = *first_chunk.end();
						while found_contiguous {
							found_contiguous = false;
							if let Some(next_chunk) = cursor_mut.get().map(|w| w.deref()) {
								if *next_chunk.start() - 1 == new_end_page {
									found_contiguous = true;
									new_end_page = *next_chunk.end();
								} else {
									trace!("Page allocator: next {:?} was not contiguously above initial too-small {:?}", next_chunk, first_chunk);
								}
							} else {
								trace!("Page allocator: couldn't get next chunk above initial too-small {:?}", first_chunk);
							}
							
							if found_contiguous {
								let chunk2 = cursor_mut.remove().expect("BUG: page_allocator failed to merge contiguous chunks.").into_inner();
								first_chunk.merge(chunk2).expect("BUG: page_allocator failed to merge contiguous chunks.");							
							}
						}
						// ToDo: put in conditional
						cursor_mut.insert_before(first_chunk);
						cursor_mut.move_prev(); //back to original chunk that has now been combined to the max

						if let Some(combined_chunk) = cursor_mut.get().map(|w| w.deref()) {
							if requested_page >= *combined_chunk.start() {
								if requested_end_page <= *combined_chunk.end() {
									// warn!("3 {:?}, num_pages = {}", requested_page, num_pages);
									return adjust_chosen_chunk(PageRange::new(requested_page, requested_page + (num_pages - 1)), ValueRefMut::RBTree(cursor_mut));
								}
							}
						}

					}
				}
			}
		}
	}

	Err(AllocationError::AddressNotFree(requested_page, num_pages))
}


/// Searches the given `list` for any chunk large enough to hold at least `num_pages`.
///
/// If a given range is specified, the returned `AllocatedPages` *must* exist
/// fully within that inclusive range of pages.
///
/// If no range is specified, this function first attempts to find a suitable chunk
/// that is **not** within the designated regions,
/// and only allocates from the designated regions as a backup option.
fn find_any_chunk(
	list: &mut StaticArrayRBTree<FreePages>,
	num_pages: usize,
	within_range: Option<&PageRange>,
) -> Result<(AllocatedPages, DeferredAllocAction<'static>), AllocationError> {
	let designated_low_end = DESIGNATED_PAGES_LOW_END.get()
		.ok_or(AllocationError::NotInitialized)?;
	let full_range = PageRange::new(*designated_low_end + 1, DESIGNATED_PAGES_HIGH_START - 1);
	let range = within_range.unwrap_or(&full_range);

	// During the first pass, we only search within the given range.
	// If no range was given, we search from the end of the low designated region
	// to the start of the high designated region.
	match list.0 {
		Inner::Array(ref mut arr) => {
			for elem in arr.iter_mut() {
				if let Some(chunk) = elem {
					// Use max and min below to ensure that the range of pages we allocate from
					// is within *both* the current chunk's bounds and the range's bounds.
					let lowest_possible_start_page = *max(chunk.start(), range.start());
					let highest_possible_end_page  = *min(chunk.end(), range.end());
					if lowest_possible_start_page + num_pages <= highest_possible_end_page { // ToDO: should there be a -1
						// warn!("4 {:?}, num_pages = {}", lowest_possible_start_page, num_pages);
					
						return adjust_chosen_chunk(
							PageRange::new(
								lowest_possible_start_page, 
								lowest_possible_start_page + (num_pages - 1)
							),
							ValueRefMut::Array(elem),
						);
					}

					// The early static array is not sorted, so we must iterate over all elements.
				}
			}
		}
		Inner::RBTree(ref mut tree) => {
			// NOTE: if RBTree had a `range_mut()` method, we could simply do the following:
			// ```
			// let eligible_chunks = tree.range_mut(
			//     Bound::Included(range.start()),
			//     Bound::Included(range.end())
			// );
			// for c in eligible_chunks { ... }
			// ```
			//
			// However, RBTree doesn't have a `range_mut()` method, so we use cursors for manual iteration.
			//
			// Because we allocate new pages by peeling them off from the beginning part of a chunk, 
			// it's MUCH faster to start the search for free pages from higher addresses moving down. 
			// This results in an O(1) allocation time in the general case, until all address ranges are already in use.
			let mut cursor = tree.upper_bound_mut(Bound::Included(range.end()));
			while let Some(chunk) = cursor.get().map(|w| w.deref()) {
				// Use max and min below to ensure that the range of pages we allocate from
				// is within *both* the current chunk's bounds and the range's bounds.
				let lowest_possible_start_page = *max(chunk.start(), range.start());
				let highest_possible_end_page  = *min(chunk.end(), range.end());
				if lowest_possible_start_page + num_pages <= highest_possible_end_page {
					// warn!("5 {:?}, num_pages = {}", lowest_possible_start_page, num_pages);				
					return adjust_chosen_chunk(
						PageRange::new(
							lowest_possible_start_page,
							lowest_possible_start_page + (num_pages - 1),
						),
						ValueRefMut::RBTree(cursor)
					);
				}

				if chunk.start() <= range.start() {
					break; // move on to searching through the designated regions
				}
				warn!("page_allocator: unlikely scenario: had to search multiple chunks while trying to allocate {} pages in {:?}.", num_pages, range);
				cursor.move_prev();
			}
		}
	}

	// If we failed to find suitable pages within the given range, return an error.
	if let Some(range) = within_range {
		return Err(AllocationError::OutOfAddressSpace(num_pages, Some(range.clone())));
	}

	// If we can't find any suitable chunks in the non-designated regions, then look in both designated regions.
	warn!("PageAllocator: unlikely scenario: non-designated chunks are all allocated, \
		  falling back to allocating {} pages from designated regions!", num_pages);
	match list.0 {
		Inner::Array(ref mut arr) => {
			for elem in arr.iter_mut() {
				if let Some(chunk) = elem {
					if num_pages <= chunk.size_in_pages() {
						// warn!("6 {:?}, num_pages = {}", *chunk.start(), num_pages);
						return adjust_chosen_chunk(PageRange::new(*chunk.start(), *chunk.start() + (num_pages - 1)), ValueRefMut::Array(elem));
					}
				}
			}
		}
		Inner::RBTree(ref mut tree) => {
			// NOTE: if RBTree had a `range_mut()` method, we could simply do the following:
			// ```
			// let eligible_chunks = tree.range(
			// 	Bound::<&Page>::Unbounded,
			// 	Bound::Included(&DESIGNATED_PAGES_LOW_END)
			// ).chain(tree.range(
			// 	Bound::Included(&DESIGNATED_PAGES_HIGH_START),
			// 	Bound::<&Page>::Unbounded
			// ));
			// for c in eligible_chunks { ... }
			// ```
			//
			// RBTree doesn't have a `range_mut()` method, so we use cursors for two rounds of iteration.
			// The first iterates over the lower designated region, from higher addresses to lower, down to zero.
			let mut cursor = tree.upper_bound_mut(Bound::Included(designated_low_end));
			while let Some(chunk) = cursor.get().map(|w| w.deref()) {
				if num_pages < chunk.size_in_pages() {
					// warn!("7 {:?}, num_pages = {}", *chunk.start(), num_pages);
					return adjust_chosen_chunk(PageRange::new(*chunk.start(), *chunk.start() + (num_pages - 1)), ValueRefMut::RBTree(cursor));
				}
				cursor.move_prev();
			}

			// The second iterates over the higher designated region, from the highest (max) address down to the designated region boundary.
			let mut cursor = tree.upper_bound_mut::<FreePages>(Bound::Unbounded);
			while let Some(chunk) = cursor.get().map(|w| w.deref()) {
				if chunk.start() < &DESIGNATED_PAGES_HIGH_START {
					// we already iterated over non-designated pages in the first match statement above, so we're out of memory. 
					break; 
				}
				if num_pages < chunk.size_in_pages() {
					// warn!("8 {:?}, num_pages = {}", *chunk.start(), num_pages);
					return adjust_chosen_chunk(PageRange::new(*chunk.start(), *chunk.start() + (num_pages - 1)), ValueRefMut::RBTree(cursor));
				}
				cursor.move_prev();
			}
		}
	}

	Err(AllocationError::OutOfAddressSpace(num_pages, None))
}


/// Removes a `Pages` object from the RBTree. 
/// `pages_ref` is basically a wrapper over the cursor which stores the position of the pages.
fn retrieve_pages_from_ref(mut pages_ref: ValueRefMut<FreePages>) -> Option<FreePages> {
    // Remove the chosen chunk from the free frame list.
    let removed_val = pages_ref.remove();
    
    match removed_val {
        RemovedValue::Array(c) => c,
        RemovedValue::RBTree(option_frames) => {
            option_frames.map(|c| c.into_inner())
        }
    }
}


/// The final part of the main allocation routine that optionally merges two contiguous chunks and 
/// then splits the resulting chunk into multiple smaller chunks, thereby "allocating" pages from it.
///
/// This function breaks up that chunk into multiple ones and returns an `AllocatedPages` 
/// from (part of) that chunk that has the same range as `pages_to_allocate`.
fn adjust_chosen_chunk(
	pages_to_allocate: PageRange,
	chosen_chunk_ref: ValueRefMut<FreePages>,
) -> Result<(AllocatedPages, DeferredAllocAction<'static>), AllocationError> {

	let chosen_chunk = retrieve_pages_from_ref(chosen_chunk_ref)
		.expect("BUG: Failed to retrieve chunk from free list");

	// trace!("to allocate: {:?}, chosen_chunk: {:?}", pages_to_allocate, chosen_chunk);
	let SplitPages{ before_start, start_to_end: new_allocation, after_end } = chosen_chunk.split_range(pages_to_allocate)
        .expect("BUG: Failed to split chunk");

	// some sanity checks -- these can be removed or disabled for better performance
	if let Some(ref b) = before_start {
		assert!(!new_allocation.contains(b.end()));
		assert!(!b.contains(new_allocation.start()));
	}
	if let Some(ref a) = after_end {
		assert!(!new_allocation.contains(a.start()));
		assert!(!a.contains(new_allocation.end()));
	}

	// TODO: Re-use the allocated wrapper if possible, rather than allocate a new one entirely.
	// if let RemovedValue::RBTree(Some(wrapper_adapter)) = _removed_chunk { ... }

	Ok((
		new_allocation.into_allocated_pages(),
		DeferredAllocAction::new(before_start, after_end),
	))
}


/// Possible options when requested pages from the page allocator.
pub enum AllocationRequest<'r> {
	/// The allocated pages can be located at any virtual address.
	Any,
	/// The allocated pages must start exactly at the given `VirtualAddress`.
	AtVirtualAddress(VirtualAddress),
	/// The allocated pages can be located anywhere within the given range.
	WithinRange(&'r PageRange),
}

/// The core page allocation routine that allocates the given number of virtual pages,
/// optionally at the requested starting `VirtualAddress`.
/// 
/// This simply reserves a range of virtual addresses, it does not allocate 
/// actual physical memory frames nor do any memory mapping. 
/// Thus, the returned `AllocatedPages` aren't directly usable until they are mapped to physical frames. 
/// 
/// Allocation is based on a red-black tree and is thus `O(log(n))`.
/// Fragmentation isn't cleaned up until we're out of address space, but that's not really a big deal.
/// 
/// # Arguments
/// * `request`: whether to allocate `num_pages` pages at any address,
///    at a specific virtual address, or withing a specified range.
/// * `num_pages`: the number of `Page`s to be allocated. 
/// 
/// # Return
/// If successful, returns a tuple of two items:
/// * the pages that were allocated, and
/// * an opaque struct representing details of bookkeeping-related actions that may cause heap allocation. 
///   Those actions are deferred until this returned `DeferredAllocAction` struct object is dropped, 
///   allowing the caller (such as the heap implementation itself) to control when heap allocation may occur.
pub fn allocate_pages_deferred(
	request: AllocationRequest,
	num_pages: usize,
) -> Result<(AllocatedPages, DeferredAllocAction<'static>), &'static str> {
	if num_pages == 0 {
		warn!("PageAllocator: requested an allocation of 0 pages... stupid!");
		return Err("cannot allocate zero pages");
	}

	let mut locked_list = FREE_PAGE_LIST.lock();

	// The main logic of the allocator is to find an appropriate chunk that can satisfy the allocation request.
	// An appropriate chunk satisfies the following conditions:
	// - Can fit the requested size (starting at the requested address) within the chunk.
	// - The chunk can only be within in a designated region if a specific address was requested, 
	//   or all other non-designated chunks are already in use.
	let res = match request {
		AllocationRequest::AtVirtualAddress(vaddr) => {
			find_specific_chunk(&mut locked_list, Page::containing_address(vaddr), num_pages)
		}
		AllocationRequest::Any => {
			find_any_chunk(&mut locked_list, num_pages, None)
		}
		AllocationRequest::WithinRange(range) => {
			find_any_chunk(&mut locked_list, num_pages, Some(range))
		}
	};
	res.map_err(From::from) // convert from AllocationError to &str
}


/// Similar to [`allocated_pages_deferred()`](fn.allocate_pages_deferred.html),
/// but accepts a size value for the allocated pages in number of bytes instead of number of pages. 
/// 
/// This function still allocates whole pages by rounding up the number of bytes. 
pub fn allocate_pages_by_bytes_deferred(
	request: AllocationRequest,
	num_bytes: usize,
) -> Result<(AllocatedPages, DeferredAllocAction<'static>), &'static str> {
	let actual_num_bytes = if let AllocationRequest::AtVirtualAddress(vaddr) = request {
		num_bytes + (vaddr.value() % PAGE_SIZE)
	} else {
		num_bytes
	};
	let num_pages = (actual_num_bytes + PAGE_SIZE - 1) / PAGE_SIZE; // round up
	allocate_pages_deferred(request, num_pages)
}


/// Allocates the given number of pages with no constraints on the starting virtual address.
/// 
/// See [`allocate_pages_deferred()`](fn.allocate_pages_deferred.html) for more details. 
pub fn allocate_pages(num_pages: usize) -> Option<AllocatedPages> {
	allocate_pages_deferred(AllocationRequest::Any, num_pages)
		.map(|(ap, _action)| ap)
		.ok()
}


/// Allocates pages with no constraints on the starting virtual address, 
/// with a size given by the number of bytes. 
/// 
/// This function still allocates whole pages by rounding up the number of bytes. 
/// See [`allocate_pages_deferred()`](fn.allocate_pages_deferred.html) for more details. 
pub fn allocate_pages_by_bytes(num_bytes: usize) -> Option<AllocatedPages> {
	allocate_pages_by_bytes_deferred(AllocationRequest::Any, num_bytes)
		.map(|(ap, _action)| ap)
		.ok()
}


/// Allocates pages starting at the given `VirtualAddress` with a size given in number of bytes. 
/// 
/// This function still allocates whole pages by rounding up the number of bytes. 
/// See [`allocate_pages_deferred()`](fn.allocate_pages_deferred.html) for more details. 
pub fn allocate_pages_by_bytes_at(vaddr: VirtualAddress, num_bytes: usize) -> Result<AllocatedPages, &'static str> {
	allocate_pages_by_bytes_deferred(AllocationRequest::AtVirtualAddress(vaddr), num_bytes)
		.map(|(ap, _action)| ap)
}


/// Allocates the given number of pages starting at (inclusive of) the page containing the given `VirtualAddress`.
/// 
/// See [`allocate_pages_deferred()`](fn.allocate_pages_deferred.html) for more details. 
pub fn allocate_pages_at(vaddr: VirtualAddress, num_pages: usize) -> Result<AllocatedPages, &'static str> {
	allocate_pages_deferred(AllocationRequest::AtVirtualAddress(vaddr), num_pages)
		.map(|(ap, _action)| ap)
}


/// Allocates the given number of pages with the constraint that
/// they must be within the given inclusive `range` of pages.
pub fn allocate_pages_in_range(
	num_pages: usize,
	range: &PageRange,
) -> Result<AllocatedPages, &'static str> {
	allocate_pages_deferred(AllocationRequest::WithinRange(range), num_pages)
		.map(|(ap, _action)| ap)
}


/// Allocates pages with a size given in number of bytes with the constraint that
/// they must be within the given inclusive `range` of pages.
pub fn allocate_pages_by_bytes_in_range(
	num_bytes: usize,
	range: &PageRange,
) -> Result<AllocatedPages, &'static str> {
	allocate_pages_by_bytes_deferred(AllocationRequest::WithinRange(range), num_bytes)
		.map(|(ap, _action)| ap)
}


/// Converts the page allocator from using static memory (a primitive array) to dynamically-allocated memory.
/// 
/// Call this function once heap allocation is available. 
/// Calling this multiple times is unnecessary but harmless, as it will do nothing after the first invocation.
#[doc(hidden)] 
pub fn convert_page_allocator_to_heap_based() {
	FREE_PAGE_LIST.lock().convert_to_heap_allocated();
}

/// A debugging function used to dump the full internal state of the page allocator. 
#[doc(hidden)] 
pub fn dump_page_allocator_state() {
	debug!("--------------- FREE PAGES LIST ---------------");
	for c in FREE_PAGE_LIST.lock().iter() {
		debug!("{:X?}", c);
	}
	debug!("---------------------------------------------------");
}
