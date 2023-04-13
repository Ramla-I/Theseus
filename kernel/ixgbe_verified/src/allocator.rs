use memory::{EntryFlags, PhysicalAddress, allocate_pages_by_bytes, allocate_frames_by_bytes_at, get_kernel_mmi_ref, MappedPages, create_contiguous_mapping, BorrowedSliceMappedPages, Mutable};
use pci::{PciBaseAddr, PciMemSize};
use crate::{hal::{NumDesc, descriptors::Descriptor}, descriptors::AdvancedTxDescriptor, tx_queue::TransmitHead};
use alloc::{
    boxed::Box,
    vec::Vec,
};
use zerocopy::FromBytes;
use owning_ref::BoxRefMut;
use packet_buffers::{PacketBuffer, MTU, MIN_ETHERNET_FRAME_LEN_IN_BYTES};
use core::ops::Deref;
use volatile::ReadOnly;

/// The mapping flags used for MMIO registers.
/// They include the NO_CACHE flag.
pub const NIC_MAPPING_FLAGS_NO_CACHE: EntryFlags = EntryFlags::from_bits_truncate(
    EntryFlags::PRESENT.bits() |
    EntryFlags::WRITABLE.bits() |
    EntryFlags::NO_CACHE.bits() |
    EntryFlags::NO_EXECUTE.bits()
);

/// The mapping flags used for descriptors and packet buffers
pub const NIC_MAPPING_FLAGS_CACHED: EntryFlags = EntryFlags::from_bits_truncate(
    EntryFlags::PRESENT.bits() |
    EntryFlags::WRITABLE.bits() |
    EntryFlags::NO_EXECUTE.bits()
);


pub fn allocate_device_register_memory(base_addr: &PciBaseAddr, mem_size_in_bytes: &PciMemSize, mapping_flags: EntryFlags) -> Result<MappedPages, &'static str> {
    memory::allocate_memory(*base_addr.deref(), *mem_size_in_bytes.deref() as usize, mapping_flags)
}

/// Returns a buffer pool from where packet buffers are taken and returned
/// 
/// # Arguments
/// * `num_buffers`: number of buffers that are initially added to the pool 
/// * `buffer_size`: size of the receive buffers in bytes
pub fn init_rx_buf_pool(num_buffers: usize) -> Result<Vec<PacketBuffer<{MTU::Standard}>>, &'static str> {
    // let buffer_size_in_bytes = DEFAULT_RX_BUFFER_SIZE_2KB as u16 * 1024;
    let mut buffer_pool = Vec::with_capacity(num_buffers);
    for _i in 0..num_buffers {
        let rx_buf = PacketBuffer::<{MTU::Standard}>::new(MIN_ETHERNET_FRAME_LEN_IN_BYTES)?;
        buffer_pool.push(rx_buf);
    }
    Ok(buffer_pool)
}


/// Allocates the memory for a descriptor ring, maps it to a slice of descriptors `T`, and clears each descriptor in the ring.
pub(crate) fn create_desc_ring<T: Descriptor + FromBytes>(num_desc: NumDesc) -> Result<(BorrowedSliceMappedPages<T, Mutable>, PhysicalAddress), &'static str> {
    
    let size_in_bytes_of_all_tx_descs = num_desc as usize * core::mem::size_of::<T>();
    
    // descriptor rings must be 128 byte-aligned, which is satisfied below because it's aligned to a page boundary.
    let (descs_mapped_pages, descs_starting_phys_addr) = create_contiguous_mapping(size_in_bytes_of_all_tx_descs, NIC_MAPPING_FLAGS_CACHED)?;

    let mut desc_ring = descs_mapped_pages.into_borrowed_slice_mut::<T>(0, num_desc as usize).map_err(|(_mp, err)| err)?;

    for desc in desc_ring.iter_mut() { desc.clear() }

    Ok((desc_ring, descs_starting_phys_addr))
}

/// This wastes a lot of memory, but it's the easiest way to get an aligned physical address
pub(crate) fn create_descriptor_writeback_field() -> Result<(BoxRefMut<MappedPages, TransmitHead>, PhysicalAddress), &'static str> {
    let (desc_wb_mapped_pages, desc_wb_starting_phys_addr) = create_contiguous_mapping(core::mem::size_of::<TransmitHead>(), NIC_MAPPING_FLAGS_CACHED)?;

    Ok((BoxRefMut::new(Box::new(desc_wb_mapped_pages)).try_map_mut(|mp| mp.as_type_mut::<TransmitHead>(0))?, desc_wb_starting_phys_addr))
}