use prusti_contracts::*;

#[extern_spec]
impl usize {
    #[pure]
    #[ensures(match result {
        Some(x) => x == self + rhs,
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