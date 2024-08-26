#![allow(unused_imports)] // for prusti spec
use prusti_contracts::*;
use prusti_external_spec::partial_ord::*;
use crate::{Frame, Page};

#[pure]
pub(crate) fn min_frame(a: Frame, b: Frame) -> Frame {
    if a < b { a } else { b }
}

#[pure]
pub(crate) fn max_frame(a: Frame, b: Frame) -> Frame {
    if a > b { a } else { b }
}

#[pure]
pub(crate) fn min_page(a: Page, b: Page) -> Page {
    if a < b { a } else { b }
}

#[pure]
pub(crate) fn max_page(a: Page, b: Page) -> Page {
    if a > b { a } else { b }
}

#[pure]
#[trusted]
#[ensures(usize::MAX - a < b ==> result == usize::MAX)]
#[ensures(usize::MAX - a >= b ==> result == a + b)]
#[ensures(usize::MAX - a == b ==> result == usize::MAX)]
pub(crate) fn saturating_add(a: usize, b: usize) -> usize {
     a.saturating_add(b)
}

#[pure]
#[trusted]
#[ensures(a < b ==> result == 0)]
#[ensures(a >= b ==> result == a - b)]
#[ensures(a == b ==> result == 0)]
pub(crate) fn saturating_sub(a: usize, b: usize) -> usize {
     a.saturating_sub(b)
}

// #[pure]
// #[trusted]
// #[ensures(if (a == 0 || b == 0) {
//     result == 0
// } else if (a >= usize::MAX / b) {
//     result == usize::MAX
// } else if (b >= usize::MAX / a) {
//     result == usize::MAX
// } else {
//     result == a*b
// })]
// pub(crate) fn saturating_mul(a: usize, b: usize) -> usize {
//      a.saturating_mul(b)
// }