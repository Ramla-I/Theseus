use core::cmp::{PartialOrd, Ordering};
use prusti_contracts::*;

#[extern_spec]
trait PartialOrd<Rhs> {
    #[pure]
    fn partial_cmp(&self, other: &Rhs) -> Option<core::cmp::Ordering>;

    #[pure]
    #[ensures(result == matches!(self.partial_cmp(other), Some(core::cmp::Ordering::Greater)))]
    #[ensures(result == (self.partial_cmp(other) != Some(core::cmp::Ordering::Less)))]
    #[ensures(result == (self.partial_cmp(other) != Some(core::cmp::Ordering::Equal)))]
    #[ensures(result == !self.le(other))]
    fn gt(&self, other: &Rhs) -> bool;

    #[pure]
    #[ensures(result == matches!(self.partial_cmp(other), Some(core::cmp::Ordering::Greater | core::cmp::Ordering::Equal)))]
    #[ensures(result == (self.partial_cmp(other) != Some(core::cmp::Ordering::Less)))]
    #[ensures(result == !self.lt(other))]
    fn ge(&self, other: &Rhs) -> bool;

    #[pure]
    #[ensures(result == matches!(self.partial_cmp(other), Some(core::cmp::Ordering::Less)))]
    #[ensures(result == (self.partial_cmp(other) != Some(core::cmp::Ordering::Greater)))]
    #[ensures(result == (self.partial_cmp(other) != Some(core::cmp::Ordering::Equal)))]
    #[ensures(result == !self.ge(other))]
    fn lt(&self, other: &Rhs) -> bool;

    #[pure]
    #[ensures(result == matches!(self.partial_cmp(other), Some(core::cmp::Ordering::Less | core::cmp::Ordering::Equal)))]
    #[ensures(result == (self.partial_cmp(other) != Some(core::cmp::Ordering::Greater)))]
    #[ensures(result == !self.gt(other))]
    fn le(&self, other: &Rhs) -> bool;
}

#[extern_spec]
impl PartialOrd<usize> for usize {
    #[pure]
    #[ensures(if *self > *other {
        result === Some(core::cmp::Ordering::Greater)
    } else if *self < *other {
        result === Some(core::cmp::Ordering::Less)
    } else {
        result === Some(core::cmp::Ordering::Equal)
    })]
    fn partial_cmp(&self, other: &usize) -> Option<core::cmp::Ordering>;

    #[pure]
    fn gt(&self, other: &usize) -> bool;

    #[pure]
    fn ge(&self, other: &usize) -> bool;

    #[pure]
    fn lt(&self, other: &usize) -> bool;

    #[pure]
    fn le(&self, other: &usize) -> bool;
}