use memory_structs::{PageRange, Page};
use crate::{AllocatedPages};
use core::{borrow::Borrow, cmp::{Ordering, min, max}, fmt, ops::{Deref, DerefMut}};

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
#[derive(Debug, Eq)]
pub(crate) struct Chunk {
	/// The Pages covered by this chunk, an inclusive range. 
	pub(crate) pages: PageRange,
}
assert_not_impl_any!(Chunk: DerefMut, Clone);

impl Chunk {
	pub(crate) fn as_allocated_pages(self) -> AllocatedPages {
		AllocatedPages {
			pages: self,
		}
	}

	/// Returns a new `Chunk` with an empty range of pages. 
	pub(crate) const fn empty() -> Chunk {
		Chunk {
			pages: PageRange::empty(),
		}
	}

    pub(crate) fn merge(&mut self, other: Chunk) -> Result<(), Chunk> {
        if *self.start() == *other.end() + 1 {
            // `other` comes contiguously before `self`
            self.pages = PageRange::new(*other.start(), *self.end());
        } 
        else if *self.end() + 1 == *other.start() {
            // `self` comes contiguously before `other`
            self.pages = PageRange::new(*self.start(), *other.end());
        }
        else {
            // non-contiguous
            return Err(other);
        }

        // ensure the now-merged AllocatedFrames doesn't run its drop handler and free its frames.
        core::mem::forget(other); 
        Ok(())
    }

    pub fn split_at(self, at_page: Page) -> Result<(Chunk, Chunk), Chunk> {
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

        // ensure the original AllocatedFrames doesn't run its drop handler and free its frames.
        core::mem::forget(self);   
        Ok((
            Chunk { pages: first }, 
            Chunk { pages: second },
        ))
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
