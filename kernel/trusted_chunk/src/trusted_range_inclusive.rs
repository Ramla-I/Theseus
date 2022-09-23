#[cfg(feature="prusti")]
use prusti_contracts::*;

use core::ops::RangeInclusive;

#[derive(Clone, Copy, PartialEq, Eq, Debug)] // Debug can't be used with Prusti
pub struct TrustedRangeInclusive {
    pub(crate) start: usize,
    pub(crate) end: usize
}

impl TrustedRangeInclusive {
    #[cfg_attr(feature="prusti", requires(start <= end))]
    #[cfg_attr(feature="prusti", ensures(result.start == start))]
    #[cfg_attr(feature="prusti", ensures(result.end == end))]
    pub(crate) const fn new(start: usize, end: usize) -> Self {
        Self{start, end}
    }

    #[cfg_attr(feature="prusti", pure)]
    pub(crate) fn overlap(&self, range2: &Self) -> bool {
        ((self.end >= range2.start) && (self.start <= range2.end)) 
        || 
        ((range2.end >= self.start) && (range2.start <= self.end))
    }

    pub fn to_range_inclusive(&self) -> RangeInclusive<usize> {
        RangeInclusive::new(self.start, self.end)
    }
}


// #[extern_spec]
// impl<Idx> RangeInclusive<Idx> {
//     #[pure]
//     pub const fn start(&self) -> &Idx;

//     #[pure]
//     pub const fn end(&self) -> &Idx;

//     // #[ensures(Self::start(result) == start)]
//     // #[ensures(Self::end(result) == end)]
//     pub const fn new(start: Idx, end: Idx) -> Self;
// }

