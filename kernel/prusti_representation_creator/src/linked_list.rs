//! Most of the List code is adapted from the Prusti user guide 
//! https://viperproject.github.io/prusti-dev/user-guide/tour/summary.html


use prusti_contracts::*;
use crate::resource_identifier::*;
use prusti_external_spec::{trusted_option::*,trusted_result::*};
use alloc::boxed::Box;

pub struct List<T> {
    head: Link<T>,
}

type Link<T> = Option<Box<Node<T>>>;

struct Node<T> {
    elem: T,
    next: Link<T>,
}

impl<T> List<T> {
    #[pure]
    pub fn len(&self) -> usize {
        link_len(&self.head)
    }

    #[pure]
    fn is_empty(&self) -> bool {
        matches!(self.head, None)
    }

    #[ensures(result.len() == 0)]
    pub const fn new() -> Self {
        List { head: None }
    }

    #[pure]
    #[requires(index < self.len())]
    pub fn lookup(&self, index: usize) -> &T {
        link_lookup(&self.head, index)
    }

    #[ensures(self.len() == old(self.len()) + 1)]
    #[ensures(snap(self.lookup(0)) === elem)]
    #[ensures(forall(|i: usize| (i < old(self.len())) ==> old(self.lookup(i)) === self.lookup(i + 1)))]
    pub fn push(&mut self, elem: T) {
        let new_node = Box::new(Node {
            elem,
            next: self.head.take(),
        });

        self.head = Some(new_node);
    }


    predicate! {
        // two-state predicate to check if the head of a list was correctly removed
        fn head_removed(&self, prev: &Self) -> bool {
            self.len() == prev.len() - 1 // The length will decrease by 1
            && forall(|i: usize| // Every element will be shifted forwards by one
                (1 <= i && i < prev.len())
                    ==> prev.lookup(i) === self.lookup(i - 1))
        }
    }

    #[ensures(old(self.is_empty()) ==>
        result.is_none() &&
        self.is_empty()
    )]
    #[ensures(!old(self.is_empty()) ==>
        self.head_removed(&old(snap(self))) &&
        result === Some(snap(old(snap(self)).lookup(0)))
    )]
    pub fn try_pop(&mut self) -> Option<T> {
        match self.head.take() {
            None => None,
            Some(node) => {
                self.head = node.next;
                Some(node.elem)
            }
        }
    }

    #[requires(!self.is_empty())]
    #[ensures(self.head_removed(&old(snap(self))))]
    #[ensures(result === old(snap(self)).lookup(0))]
    pub fn pop(&mut self) -> T {
        self.try_pop().unwrap()
    }

    // // Not currently possible in Prusti
    // pub fn try_peek(&self) -> Option<&T> {
    //     todo!()
    // }

    #[pure]
    #[requires(!self.is_empty())]
    pub fn peek(&self) -> &T {
        self.lookup(0)
    }

    #[trusted] // required due to unsupported reference in enum
    #[requires(!self.is_empty())]
    #[ensures(snap(result) === old(snap(self.peek())))]
    #[after_expiry(
        old(self.len()) === self.len() // (1. condition)
        && forall(|i: usize| 1 <= i && i < self.len() // (2. condition)
            ==> old(snap(self.lookup(i))) === snap(self.lookup(i)))
        && snap(self.peek()) === before_expiry(snap(result)) // (3. condition)
    )]
    pub fn peek_mut(&mut self) -> &mut T {
        // This does not work in Prusti at the moment:
        // "&mut self.head" has type "&mut Option<T>"
        // this gets auto-dereferenced by Rust into type: "Option<&mut T>"
        // this then gets matched to "Some(node: &mut T)"
        // References in enums are not yet supported, so this cannot be verified by Prusti
        if let Some(node) = &mut self.head {
            &mut node.elem
        } else {
            unreachable!()
        }
        // ...
    }
}

#[pure]
#[requires(index < link_len(link))]
fn link_lookup<T>(link: &Link<T>, index: usize) -> &T {
    match link {
        Some(node) => {
            if index == 0 {
                &node.elem
            } else {
                link_lookup(&node.next, index - 1)
            }
        }
        None => unreachable!(),
    }
}

impl<T: ResourceIdentifier> List<T> {

    #[pure]
    #[requires(index < self.len())]
    // #[ensures(result == snap(self.lookup(index)))]
    #[ensures(result == *self.lookup(index))]
    fn lookup_copy(&self, index: usize) -> T {
        link_lookup_copy(&self.head, index)
    }

    
    #[requires(index <= self.len())]
    #[ensures(
        match result {
            Some(idx) => idx < self.len() && self.lookup(idx).overlaps(&elem),
            None => forall(|i: usize| ((index <= i && i < self.len()) ==> !self.lookup(i).overlaps(&elem)))
        }
    )]
    pub(crate) fn elem_overlaps_in_list(&self, elem: T, index: usize) -> Option<usize> {
        if index == self.len() {
            return None;
        }

        // We need the copy because otherwise prusti compiler error about chaining pure functions
        let ret = if self.lookup_copy(index).overlaps(&elem) { 
            Some(index)
        } else {
            self.elem_overlaps_in_list(elem, index + 1)
        };
        ret
    }


    #[requires(forall(|i: usize, j: usize| (i < self.len() && i < j && j < self.len()) ==> 
        !self.lookup(j).overlaps(&self.lookup(i)))
    )]
    #[ensures(
        match result {
            Ok(()) => {
                self.len() == old(self.len()) + 1 && snap(self.lookup(0)) === elem
                && forall(|i: usize| (1 <= i && i < self.len()) ==> old(self.lookup(i-1)) == self.lookup(i))
                && forall(|i: usize| (1 <= i && i < self.len()) ==> !self.lookup(i).overlaps(&elem))
            },
            Err(idx) => idx < self.len() && self.lookup(idx).overlaps(&elem)
        }
    )]
    #[ensures(forall(|i: usize, j: usize| (i < self.len() && i < j && j < self.len()) ==> 
        !self.lookup(j).overlaps(&self.lookup(i)))
    )]
    pub fn push_unique_with_precond(&mut self, elem: T) -> Result<(),usize> {
        match self.elem_overlaps_in_list(elem, 0) {
            Some(idx) => Err(idx),
            None => {
                let new_node = Box::new(Node {
                    elem,
                    next: self.head.take(),
                });
        
                self.head = Some(new_node);
                Ok(())
            }
        }
    }
}

#[pure]
#[requires(index < link_len(link))]
// #[ensures(result == snap(link_lookup(link, index)))]
#[ensures(result == *link_lookup(link, index))]
fn link_lookup_copy<T: ResourceIdentifier>(link: &Link<T>, index: usize) -> T {
    match link {
        Some(node) => {
            if index == 0 {
                node.elem
            } else {
                link_lookup_copy(&node.next, index - 1)
            }
        }
        None => unreachable!(),
    }
}

#[pure]
fn link_len<T>(link: &Link<T>) -> usize {
    match link {
        None => 0,
        Some(node) => 1 + link_len(&node.next),
    }
}