use prusti_contracts::*;

pub trait ResourceIdentifier: Copy + PartialEq {
    // type Resource;

    #[pure]
    fn overlaps(&self, other: &Self) -> bool;

    // #[pure] // this will become an issue when resource identifier and resource are in separate crates because of cyclic dependency
    // fn equal_to_resource(&self, resource: &Self::Resource) -> bool;
}