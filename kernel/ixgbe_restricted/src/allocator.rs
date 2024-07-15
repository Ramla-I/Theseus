use memory::{EntryFlags, MappedPages};
use pci::{PciBaseAddr, PciMemSize};
use core::ops::Deref;

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