use prusti_contracts::*;
use crate::resource_identifier::*;
use prusti_external_spec::{trusted_option::*,trusted_result::*};

#[cfg_attr(test, derive(Debug))]
pub struct StaticArray<T: ResourceIdentifier> {
    arr: [Option<T>; 64], 
}

impl<T: ResourceIdentifier> StaticArray<T> {
    pub const fn new() -> Self {
        StaticArray {
            arr: [None; 64],
        }
    }

    #[pure]
    pub const fn len(&self) -> usize {
        self.arr.len()
    }

    /// Looks up an element in the array.
    /// 
    /// # Pre-conditions:
    /// * index is less than the length
    #[pure]
    #[requires(index < self.len())]
    pub fn lookup(&self, index: usize) -> Option<T> {
        self.arr[index]
    }

    /// Updates an element in the array.
    /// 
    /// # Pre-conditions:
    /// * index is less than the length
    /// 
    /// Added this function because the verifier cannot reason about remaining elements in the array when we update an element
    /// or at least not when using the lookup function. 
    /// The same conditions seemed to verify in push() if we used self.arr[i] rather than self.lookup(i)
    #[requires(index < self.len())]
    #[ensures(self.lookup(index) == Some(val))]
    #[ensures(forall(|i: usize| (i < self.len() && i != index) ==> self.lookup(i) == old(self.lookup(i))))]
    #[ensures(forall(|i: usize| (i < self.len() && i != index) ==> self.lookup(i).is_some() == old(self.lookup(i).is_some())))]
    #[ensures(forall(|i: usize| (i < self.len() && i != index) ==> self.lookup(i).is_none() == old(self.lookup(i).is_none())))]
    pub fn update(&mut self, index: usize, val: T) {
        self.arr[index] = Some(val)
    }

    
    predicate! {
        // predicate to check that the elements in the array are ordered
        pub fn ordered_static_array(&self) -> bool {
            forall(|i: usize| (i < self.len() && self.lookup(i).is_some()) ==> {
                forall(|j: usize| (j < i) ==> self.lookup(j).is_some())
            })
            &&
            forall(|i: usize| (i < self.len() && self.lookup(i).is_none()) ==> {
                forall(|j: usize| (i <= j && j < self.len()) ==> self.lookup(j).is_none())
            })
        }
    }


    predicate! {
        pub fn element_is_some(&self, idx: usize) -> bool { idx < self.len() && self.lookup(idx).is_some() }
    }


    #[requires(self.ordered_static_array())]
    #[ensures(result.is_err() ==> // moving these to a match statement leads to compiler error
        forall(|i: usize| (i < self.len()) ==> self.lookup(i).is_some() && old(self.lookup(i)) == self.lookup(i))
    )]
    #[ensures(result.is_ok() ==> {
        let idx = peek_result(&result);
        self.element_is_some(idx)
        && peek_option(&self.lookup(idx)) == value 
        && self.ordered_static_array()
        && forall(|i: usize| ((i < self.len()) && (i != idx)) ==> self.lookup(i) == old(self.lookup(i)))
    })]
	pub(crate) fn push(&mut self, value: T) -> Result<usize,()> {
        let mut i = 0;

        while i < self.len() {
            body_invariant!(forall(|j: usize| ((j < i) ==> self.lookup(j).is_some())));
            body_invariant!(i < self.len());

            if self.lookup(i).is_none() { 
                // self.arr[i] = Some(value); // post-conditions will not verify with this line
                self.update(i, value);
                return Ok(i)
            }
            i += 1;
        }
        return Err(());
	}

    /// Returns the index of the first element in the array, starting from `index`, which overlaps with `elem`.
    /// Returns None if there is no overlap.
    ///  
    /// # Pre-conditions:
    /// * index is less than or equal to the array length
    /// 
    /// # Post-conditions:
    /// * if the result is Some(idx), then idx is less than the list's length.
    /// * if the result is Some(idx), then the element at idx is Some(_)
    /// * if the result is Some(idx), then the element at idx overlaps with `elem`
    /// * if the result is None, then no element in the array overlaps with `elem`
    #[requires(index <= self.len())]
    #[ensures(
        match result {
            Some(idx) => self.element_is_some(idx) && peek_option(&self.lookup(idx)).overlaps(&elem),
            None => forall(|i: usize| (index <= i && self.element_is_some(i)) ==> {
                !peek_option(&self.lookup(i)).overlaps(&elem)
            })
        }
    )]
    #[ensures(forall(|i: usize| (i < self.len()) ==> self.lookup(i) == old(self.lookup(i))))]
    pub(crate) fn elem_overlaps_in_array(&self, elem: T, index: usize) -> Option<usize> {
        if index >= self.arr.len() {
            return None;
        }

        let ret = match self.arr[index] {
            Some(val) => {
                if val.overlaps(&elem) {
                    Some(index)
                } else {
                    self.elem_overlaps_in_array(elem, index + 1)
                }
            },
            None => {
                self.elem_overlaps_in_array(elem, index + 1)
            }
        };
        ret
    }
}