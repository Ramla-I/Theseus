use core::{ptr::Unique, num};
use alloc::vec::Vec;
use core::cmp::{max, min};


fn any_vector(bound: u32) -> Vec<(usize, usize)> {
    let size: u32 = kani::any();
    kani::assume(size <= bound);

    let mut inner = Vec::new();

    for _ in 0..size {
        let addr: usize = kani::any();
        let size_bytes: usize= kani::any();

        inner.push((addr, size_bytes));
    }
    inner
}

// #[cfg(kani)]
// #[kani::proof]
// #[kani::unwind(3)]
// fn check_fragment() -> Result<(), ()>{
//     let mut v = any_vector(2);
//     let sa: usize = kani::any();
//     let sb: usize = kani::any();
//     let os: usize = kani::any();

//     let f = fragment::<u128>(sa, sb, os, &mut v).map_err(|_| ())?;

//     assert!((f.as_ptr() as usize) >= sa);
//     assert!((f.as_ptr() as usize) <= sa.checked_add(sb).ok_or(())?);
//     assert!((f.as_ptr() as usize).checked_add(core::mem::size_of::<u128>()).ok_or(())? <= sa.checked_add(sb).ok_or(())?);
//     assert!((f.as_ptr() as usize) % core::mem::align_of::<u128>() == 0);

//     // for prev in &v {

//     // }

//     Ok(())
// }


#[cfg(kani)]
#[kani::proof]
#[kani::unwind(3)]
fn check_fragment2() -> Result<(), ()>{
    let bound = 2;
    let sa: usize = kani::any();
    let sb: usize = kani::any();
    let os: usize = kani::any();
    let nf: usize = kani::any();

    kani::assume(nf <= bound);

    let fragments = fragment2::<u128>(sa, sb, os, nf).map_err(|_| ())?;
    
    assert!(fragments.len() == nf);

    for f in &fragments {
        assert!((f.as_ptr() as usize) >= sa);
        assert!((f.as_ptr() as usize) <= sa.checked_add(sb).ok_or(())?);
        assert!((f.as_ptr() as usize).checked_add(core::mem::size_of::<u128>()).ok_or(())? <= sa.checked_add(sb).ok_or(())?);
        assert!((f.as_ptr() as usize) % core::mem::align_of::<u128>() == 0);
    }

    for i in 0..fragments.len() {
        if i != fragments.len() - 1 {
            assert!((fragments[i].as_ptr() as usize).checked_add(core::mem::size_of::<u128>()).ok_or(())? == fragments[i+1].as_ptr() as usize);
        }
    }

    Ok(())
}

pub fn fragment<T>(
    mp_start_addr: usize, 
    mp_size_in_bytes: usize, 
    starting_offset_in_bytes: usize, 
    allocated: &mut Vec<(usize, usize)>
) -> Result<Unique<T>, &'static str> {
    let size = core::mem::size_of::<T>();
    let alignment = core::mem::align_of::<T>();
    
    let end_offset_in_bytes = starting_offset_in_bytes.checked_add(size).ok_or("overflow")?;

    // check that size of the type T fits within the size of the mapping
    if end_offset_in_bytes > mp_size_in_bytes {
        error!("the requested type doesn't fit in the MappedPages bound at the given offset");
        return Err("the requested type doesn't fit in the MappedPages bound at the given offset");
    }

    let requested_start_addr = mp_start_addr.checked_add(starting_offset_in_bytes).ok_or("overflow")?;
    if requested_start_addr % alignment != 0 {
        error!("the requested start address {} is unaligned with type alignment {}!", requested_start_addr, alignment);
        return Err("the requested start address is unaligned with type alignment");
    }

    // check that the type T at offset does not overlap with an existing allocate fragment
    let requested_end_addr = requested_start_addr.checked_add(size).and_then(|addr| addr.checked_sub(1)).ok_or("overflow")?;
    // debug!("required start addr: {:#X}, required end_addr: {:#X}", start_addr, end_addr);

    for fragment in &mut *allocated {
        let frag_start_addr = fragment.0;
        let frag_end_addr = frag_start_addr.checked_add(fragment.1).and_then(|addr| addr.checked_sub(1)).ok_or("overflow")?;
        // debug!("frag start addr: {:#X}, frag end_addr: {:#X}", frag_start_addr, frag_end_addr);

        let starts = max(requested_start_addr, frag_start_addr);
        let ends   = min(requested_end_addr, frag_end_addr);
        if starts <= ends {
            error!("the requested type at offset overlaps with an allocated fragment");
            return Err("the requested type at offset overlaps with an allocated fragment");
        }
    }

    allocated.push((requested_start_addr, size));

    Unique::new(requested_start_addr as *mut T) 
        .ok_or("Ptr was NULL")
}


pub fn fragment2<T>(start_address: usize, size_in_bytes: usize, mut offset: usize, num_fragments: usize) -> Result<Vec<Unique<T>>, &'static str> {
    let size = core::mem::size_of::<T>();
    let align = core::mem::align_of::<T>();

    let end = offset.checked_add(size.checked_mul(num_fragments).ok_or("overflow")?).ok_or("overflow")?;

    // check that size of the type T fits within the size of the mapping
    if end > size_in_bytes {
        error!("the requested type doesn't fit in the MappedPages bound at the given offset");
        return Err("the requested type doesn't fit in the MappedPages bound at the given offset");
    }

    if (start_address.checked_add(offset).ok_or("overflow")?) % align != 0 {
        // error!("MappedPages::as_type_mut(): requested type {} with size {}, but the byte_offset {} is unaligned with type alignment {}!",
        //     core::any::type_name::<T>(),
        //     size, byte_offset, mem::align_of::<T>()
        // );
        return Err("not aligned");
    }

    let mut fragments = Vec::with_capacity(num_fragments);
    for _ in 0..num_fragments {
        fragments.push(Unique::new((start_address.checked_add(offset).ok_or("overflow")?) as *mut T) 
            .ok_or("Ptr was NULL")?);
        offset += size;
    }

    Ok(fragments)
}

pub fn fragment3<T>(start_address: usize, size_in_bytes: usize, mut offset: usize) -> Result<Unique<T>, &'static str> {
    let size = core::mem::size_of::<T>();
    let align = core::mem::align_of::<T>();

    let end = offset.checked_add(size).ok_or("overflow")?;

    // check that size of the type T fits within the size of the mapping
    if end > size_in_bytes {
        error!("the requested type doesn't fit in the MappedPages bound at the given offset");
        return Err("the requested type doesn't fit in the MappedPages bound at the given offset");
    }

    if (start_address.checked_add(offset).ok_or("overflow")?) % align != 0 {
        // error!("MappedPages::as_type_mut(): requested type {} with size {}, but the byte_offset {} is unaligned with type alignment {}!",
        //     core::any::type_name::<T>(),
        //     size, byte_offset, mem::align_of::<T>()
        // );
        return Err("not aligned");
    }

    Unique::new((start_address.checked_add(offset).ok_or("overflow")?) as *mut T)
        .ok_or("ptr was null")
}