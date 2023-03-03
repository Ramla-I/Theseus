use memory::{EntryFlags, PhysicalAddress, allocate_pages_by_bytes, allocate_frames_by_bytes_at, get_kernel_mmi_ref, MappedPages, create_contiguous_mapping};
use pci::{PciDevice};
use crate::{packet_buffers::{PacketBuffer, MTU}, RxBufferSizeKiB};
use alloc::vec::Vec;


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


/// Allocates memory for the NIC registers
/// 
/// # Arguments 
/// * `dev`: reference to pci device 
/// * `mem_base`: starting physical address of the device's memory mapped registers
pub fn allocate_device_register_memory(dev: &PciDevice, mem_base: PhysicalAddress) -> Result<MappedPages, &'static str> {
    //find out amount of space needed
    let mem_size_in_bytes = dev.determine_mem_size(0) as usize;

    allocate_memory(mem_base, mem_size_in_bytes, NIC_MAPPING_FLAGS_NO_CACHE)
}

/// Helper function to allocate memory at required address
/// 
/// # Arguments
/// * `mem_base`: starting physical address of the region that need to be allocated
/// * `mem_size_in_bytes`: size of the region that needs to be allocated 
pub fn allocate_memory(mem_base: PhysicalAddress, mem_size_in_bytes: usize, mapping_flags: EntryFlags) -> Result<MappedPages, &'static str> {
    // set up virtual pages and physical frames to be mapped
    let pages_nic = allocate_pages_by_bytes(mem_size_in_bytes)
        .ok_or("NicInit::mem_map(): couldn't allocate virtual page!")?;
    let frames_nic = allocate_frames_by_bytes_at(mem_base, mem_size_in_bytes)
        .map_err(|_e| "NicInit::mem_map(): couldn't allocate physical frames!")?;

    // debug!("NicInit: memory base: {:#X}, memory size: {}", mem_base, mem_size_in_bytes);

    let kernel_mmi_ref = get_kernel_mmi_ref().ok_or("NicInit::mem_map(): KERNEL_MMI was not yet initialized!")?;
    let mut kernel_mmi = kernel_mmi_ref.lock();
    let nic_mapped_page = kernel_mmi.page_table.map_allocated_pages_to(pages_nic, frames_nic, mapping_flags)?;

    Ok(nic_mapped_page)
}


/// Returns a buffer pool from where packet buffers are taken and returned
/// 
/// # Arguments
/// * `num_buffers`: number of buffers that are initially added to the pool 
/// * `buffer_size`: size of the receive buffers in bytes
pub fn init_rx_buf_pool(num_buffers: usize, buffer_size_in_kbytes: RxBufferSizeKiB) -> Result<Vec<PacketBuffer<{MTU::Standard}>>, &'static str> {
    let buffer_size_in_bytes = buffer_size_in_kbytes as u16 * 1024;
    let mut buffer_pool = Vec::with_capacity(num_buffers);
    for _i in 0..num_buffers {
        let rx_buf = PacketBuffer::<{MTU::Standard}>::new(buffer_size_in_bytes)?;
        buffer_pool.push(rx_buf);
    }
    Ok(buffer_pool)
}