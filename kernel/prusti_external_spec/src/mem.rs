use prusti_contracts::*;

#[pure]
#[extern_spec(core::mem)]
fn align_of<T>() -> usize;

#[pure]
#[extern_spec(core::mem)]
fn size_of<T>() -> usize;