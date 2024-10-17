#![no_std]

/// Asserts that the type has fields with the given types.
#[macro_export]
macro_rules! assert_fields_type {
    ($t:ty: $($i:ident: $ti:ty),+) => {
        #[allow(unknown_lints, unneeded_field_pattern)]
        const _: () = {
            fn dummy(v: $t) {
                $(let _: $ti = v.$i;)+
            }
        };
    };
}

mod test {
    #[allow(dead_code)]
    struct A {
        x: u32,
        y: u32,
    }
    assert_fields_type!(A: x: u32, y: u32);
    // assert_fields_type!(A: x: i32); //fails

    // TO DO: make this work with tuple structs
    // struct B(A);
    // assert_fields_type!(B: 0: A);
}