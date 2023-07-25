use memory_structs::{Frame, FrameRange, PhysicalAddress};
use kernel_config::memory::PAGE_SIZE;
use range_inclusive::RangeInclusive;

pub(crate) fn into_frame_range(frames: &RangeInclusive<usize>) -> FrameRange {
    let start = into_frame(*frames.start())
        .expect("Verified chunk start was not a valid frame");
    
    let end = into_frame(*frames.end())
        .expect("Verified chunk end was not a valid frame");
    FrameRange::new(start, end)
}

fn into_frame(frame_num: usize) -> Option<Frame> {
    PhysicalAddress::new(frame_num * PAGE_SIZE)
        .map(Frame::containing_address)
}