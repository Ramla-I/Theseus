
use crate::regs::TDTWritten;

pub struct ProcessedDelimiter(u16);
impl ProcessedDelimiter {
    pub fn new() -> Self {
        ProcessedDelimiter(0)
    }

    #[inline(always)]
    pub fn increment(&mut self, ring_size: u16) -> ProcDelimInc {
        self.0 = (self.0 + 1) & (ring_size - 1);
        ProcDelimInc()
    }

    #[inline(always)]
    pub fn value(&self) -> u16 {
        self.0
    }
}

pub struct ProcDelimInc();

pub struct FlushCounter(u64);
impl FlushCounter {
    pub fn new() -> Self {
        FlushCounter(0)
    }

    #[inline(always)]
    pub fn increment(&mut self, _proc_delim_inc: ProcDelimInc) {
        self.0 += 1;
    }

    #[inline(always)]
    pub fn clear(&mut self, _tdt_written: TDTWritten) {
        self.0 = 0;
    }

    #[inline(always)]
    pub fn value(&self) -> u64 {
        self.0
    }
}
