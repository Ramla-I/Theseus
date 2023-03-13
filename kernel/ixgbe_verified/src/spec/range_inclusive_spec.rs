use prusti_contracts::*;

#[derive(Copy, Clone, PartialEq, Eq)]
pub struct RangeInclusive<Idx: Clone + PartialOrd> {
    start: Idx,
    end: Idx
}

impl<Idx: Clone + PartialOrd> RangeInclusive<Idx> {
    // #[cfg_attr(feature="prusti", requires(start <= end))]
    // #[cfg_attr(feature="prusti", ensures(result.start == start))]
    // #[cfg_attr(feature="prusti", ensures(result.end == end))]
    #[ensures(result.start == start)]
    #[ensures(result.end == end)]
    pub(crate) const fn new(start: Idx, end: Idx) -> Self {
        Self{start, end}
    }

    #[pure]
    pub const fn start(&self) -> &Idx {
        &self.start
    }

    #[pure]
    pub const fn end(&self) -> &Idx {
        &self.end
    }

    pub fn is_empty(&self) -> bool {
        !(self.start <= self.end)
    }

}
