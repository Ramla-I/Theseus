use range_inclusive::RangeInclusive;
use memory_structs::{Page, PageRange, VirtualAddress};
use kernel_config::memory::PAGE_SIZE;

pub(crate) fn into_page_range(pages: &RangeInclusive<usize>) -> PageRange {
    let start = into_page(*pages.start())
        .expect("Verified chunk start was not a valid page");
    
    let end = into_page(*pages.end())
        .expect("Verified chunk end was not a valid page");
    PageRange::new(start, end)
}

fn into_page(page_num: usize) -> Option<Page> {
    VirtualAddress::new(page_num * PAGE_SIZE)
        .map(Page::containing_address)
}