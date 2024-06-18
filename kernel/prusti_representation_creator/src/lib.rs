//! To verify this crate, in the directory with the Cargo.toml file, run the cargo-prusti executable:
//! "../../../prusti-release-2023-08-22/cargo-prusti"
//! The Prusti.toml file in this directory contains the configuration for the verification.

#![no_std]
#![allow(dead_code)]
#![allow(unused_imports)]

extern crate prusti_contracts;
extern crate prusti_external_spec;
extern crate alloc;

pub mod linked_list;
pub mod static_array;
pub mod resource_identifier;

#[cfg(test)]
mod test;

use prusti_contracts::*;
use crate::{
    resource_identifier::ResourceIdentifier, 
    static_array::StaticArray,
    linked_list::List,
};
use prusti_external_spec::{trusted_option::*,trusted_result::*};


#[derive(Clone, Copy)]
pub enum RepresentationCreationError {
    Overlap(usize),
    NoSpace
}

pub struct RepresentationCreator<T: ResourceIdentifier, R> {
    array: Option<StaticArray<T>>,
    list: List<T>,
    constructor: fn(&T) -> R
}

impl<T: ResourceIdentifier, R> RepresentationCreator<T, R> {
    #[trusted] // so we can take a function pointer as an argument
    #[ensures(result.list.len() == 0)]
    #[ensures(
        match result.array {
            Some(ref array) => pre_heap && forall(|i: usize| (i < array.len()) ==> array.lookup(i).is_none()),
            None => !pre_heap
        }
    )]
    pub const fn new(constructor: fn(&T) -> R, pre_heap: bool) -> Self {
        RepresentationCreator {
            array: if pre_heap { Some(StaticArray::new()) } else { None },
            list: List::new(), 
            constructor
        }
    }


    #[requires(match self.array {
        Some(ref array) => array.ordered_static_array(),
        None => true
    })] 
    #[ensures(result.is_ok() ==> {
        let (new_resource, idx) = peek_result_ref(&result);
        // resource_id.equal_to_resource(&new_resource) &&
        match self.array {
            Some(ref array) => forall(|i: usize| (i < array.len() && i != *idx) ==> {
                array.lookup(i) == old(array.lookup(i)) 
                && array.lookup(i).is_some() ==> !peek_option(&array.lookup(i)).overlaps(&resource_id)
            }),
            None => forall(|i: usize| i < old(self.list.len()) ==> !old(self.list.lookup(i)).overlaps(&resource_id))
        }
    })]    
    pub fn create_unique_representation(&mut self, resource_id: T) -> Result<(R, usize), RepresentationCreationError> 
    {
        let idx = match self.array { // ugly because Prusti doesn't understand the ? operator
            Some(ref mut array) => match Self::add_representation_info_pre_heap(array, resource_id) {
                Ok(idx) => idx,
                Err(err) => return Err(err)
            },
            None => match Self::add_representation_info_post_heap(&mut self.list, resource_id) {
                Ok(idx) => idx,
                Err(err) => return Err(err)
            }
        };

        Ok((self.create_new_representation(resource_id), idx))
    }

    #[requires(array.ordered_static_array())]
    #[ensures(result.is_err() ==> {
        match peek_err(&result) {
            RepresentationCreationError::Overlap(idx) => 
                array.element_is_some(idx) && peek_option(&array.lookup(idx)).overlaps(&resource_id),
            RepresentationCreationError::NoSpace => forall(|i: usize| i < array.len() ==> array.lookup(i).is_some())
        }
    })]
    #[ensures(result.is_ok() ==> {
        let idx = peek_result(&result);
        array.element_is_some(idx) && resource_id == peek_option(&array.lookup(idx))
        && forall(|i: usize| (i < array.len() && i != idx) ==> { array.lookup(i) == old(array.lookup(i))
            && array.lookup(i).is_some() ==> !peek_option(&array.lookup(i)).overlaps(&resource_id)
        })
    })]
    fn add_representation_info_pre_heap(array: &mut StaticArray<T>, resource_id: T) -> Result<usize, RepresentationCreationError> {
        let overlap_idx = array.elem_overlaps_in_array(resource_id, 0);
        match overlap_idx {
            Some(idx) => {
                Err(RepresentationCreationError::Overlap(idx))
            },
            None => {
                match array.push(resource_id) { // can't use closures because Prusti doesn't understand them :(
                    Ok(idx) => Ok(idx),
                    Err(()) => Err(RepresentationCreationError::NoSpace)
                }
            }
        }
    }


    #[ensures(result.is_err() ==> {
        match peek_err(&result) {
            RepresentationCreationError::Overlap(idx) => (idx < list.len()) & list.lookup(idx).overlaps(&resource_id),
            _ => unreachable!()
        }
    })]
    #[ensures(result.is_ok() ==> {
        list.len() >= 1 && snap(list.lookup(0)) === resource_id
        && forall(|i: usize| (i < old(list.len())) ==> !old(list.lookup(i)).overlaps(&resource_id))
    })]
    fn add_representation_info_post_heap(list: &mut List<T>, resource_id: T) -> Result<usize, RepresentationCreationError> {
        let overlap_idx = list.elem_overlaps_in_list(resource_id, 0);
        match overlap_idx {
            Some(idx) => Err(RepresentationCreationError::Overlap(idx)),
            None => {
                list.push(resource_id);
                Ok(0)
            }
        }
    }


    #[trusted]
    // #[ensures(resource_id.equal_to_resource(&result))]
    /// Function pointers are currently unsupported by Prusti, so we have to trust this function.
    fn create_new_representation(&self, resource_id: T) -> R {
        (self.constructor)(&resource_id)
    }


    #[requires(match self.array {
        Some(ref array) => array.ordered_static_array(),
        None => true
    })]
    #[ensures(result.is_ok() ==> self.array.is_none()
        && forall(|i: usize, j: usize| (i < self.list.len() && i < j && j < self.list.len()) 
            ==> !self.list.lookup(j).overlaps(&self.list.lookup(i)))
    )]
    #[ensures(match old(&self.array) { 
        Some(ref array) => result.is_ok() ==> forall(|i: usize| (i < self.list.len()) 
            ==> peek_option(&array.lookup(i)) === self.list.lookup(self.list.len() - 1 - i)),
        None => result.is_err()
    })]
    pub fn switch_to_heap_allocated(&mut self) -> Result<(),()> { 
        if self.list.len() != 0 { // because this is a pub fn, we add a runtime check rather than a precondition
            return Err(());
        }

        match self.array {
            Some(ref array) => {
                let mut i = 0;
                while i < array.len() {
                    body_invariant!(i < array.len());
                    body_invariant!(self.list.len() == i);
                    body_invariant!(forall(|j: usize| ((j < self.list.len()) ==> array.lookup(j).is_some())));
                    body_invariant!(forall(|j: usize| ((j < self.list.len()) ==> peek_option(&array.lookup(j)) == *self.list.lookup(self.list.len() - 1 - j))));
                    body_invariant!(forall(|i: usize, j: usize| (i < self.list.len() && i < j && j < self.list.len()) ==> 
                        !self.list.lookup(j).overlaps(&self.list.lookup(i)))
                    );

                    if let Some(resource_identifier) = array.lookup(i) {
                        match self.list.push_with_unique_precond(resource_identifier) {
                            Ok(()) => (),
                            Err(_) => return Err(())
                        }
                    } else {
                        break;
                    }

                    i += 1;
                }
            },
            None => return Err(())
        }
        self.array = None;
        Ok(())
    }
}