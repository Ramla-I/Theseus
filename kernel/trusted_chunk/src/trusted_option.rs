#[cfg(feature="prusti")]
use prusti_contracts::*;

use crate::trusted_range_inclusive::*;
use crate::TrustedChunk;

#[cfg(feature="prusti")]
#[extern_spec]
impl<T> core::option::Option<T> {
    #[pure]
    #[ensures(matches!(*self, Some(_)) == result)]
    pub fn is_some(&self) -> bool;

    #[pure]
    #[ensures(self.is_some() == !result)]
    pub fn is_none(&self) -> bool;

    #[requires(self.is_some())]
    pub fn unwrap(self) -> T;
}

#[cfg_attr(feature="prusti", pure)]
#[cfg_attr(feature="prusti", requires(val.is_some()))]
pub(crate) fn peek_range(val: &Option<TrustedRangeInclusive>) -> TrustedRangeInclusive {
    match val {
        Some(val) => *val,
        None => unreachable!(),
    }
}

#[cfg_attr(feature="prusti", pure)]
#[cfg_attr(feature="prusti", requires(val.is_some()))]
pub(crate) fn peek_usize(val: &Option<usize>) -> usize {
    match val {
        Some(val) => *val,
        None => unreachable!(),
    }
}

#[cfg_attr(feature="prusti", pure)]
#[cfg_attr(feature="prusti", requires(val.is_some()))]
pub(crate) fn peek_chunk(val: &Option<TrustedChunk>) -> &TrustedChunk {
    match val {
        Some(val) => val,
        None => unreachable!(),
    }
}

// #[derive(Copy, Clone)]
// pub enum OptionRange {
//     Some(TrustedRangeInclusive),
//     None,
// }

// impl OptionRange {
//     #[pure]
//     pub fn is_none(&self) -> bool {
//         match self {
//             OptionRange::Some(_) => false,
//             OptionRange::None => true,
//         }
//     }

//     #[pure]
//     pub fn is_some(&self) -> bool {
//         !self.is_none()
//     }

//     #[pure]
//     #[requires(self.is_some())]
//     pub fn peek(&self) -> TrustedRangeInclusive {
//         match self {
//             OptionRange::Some(val) => *val,
//             OptionRange::None => unreachable!(),
//         }
//     }
// }
