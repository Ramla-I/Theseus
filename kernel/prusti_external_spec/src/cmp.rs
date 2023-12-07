use prusti_contracts::*;

#[pure]
#[extern_spec(core::cmp)]
#[ensures(result == v1 || result == v2)]
#[ensures(result == v1 ==> v1 <= v2)]
#[ensures(result == v2 ==> v1 >= v2)]
fn min<T: Ord>(v1: T, v2: T) -> T;

#[pure]
#[extern_spec(core::cmp)]
#[ensures(result == v1 || result == v2)]
#[ensures(result == v1 ==> v1 >= v2)]
#[ensures(result == v2 ==> v1 <= v2)]
fn max<T: Ord>(v1: T, v2: T) -> T;