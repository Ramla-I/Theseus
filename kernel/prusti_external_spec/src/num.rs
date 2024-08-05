use prusti_contracts::*;

#[extern_spec]
impl usize {
    #[pure]
    #[ensures(match result {
        Some(x) => (x == self + rhs) && forall(|y: usize| self % y == 0 && rhs % y == 0 ==> x % y == 0),
        None => true
    })]
    pub fn checked_add(self, rhs: usize) -> Option<usize>;

    #[pure]
    #[ensures(match result {
        Some(x) => x == self * rhs,
        None => true
    })]
    pub fn checked_mul(self, rhs: usize) -> Option<usize>;
}