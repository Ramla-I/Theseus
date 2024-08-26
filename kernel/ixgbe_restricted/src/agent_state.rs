
use crate::regs::TDTWritten;

pub struct AgentState {
    processed_delimiter: u16, // The latest descriptor that needs to be checked for packet reception
    flush_counter: u64, // the number of packets that have been sent since the last flush (write to TDT)
}

impl AgentState {
    pub fn new() -> Self {
        AgentState {
            processed_delimiter: 0,
            flush_counter: 0,
        }
    }

    #[inline(always)]
    pub fn increment(&mut self, ring_size: u16) {
        self.processed_delimiter = (self.processed_delimiter + 1)  & (ring_size - 1);
        self.flush_counter += 1;
    }

    #[inline(always)]
    pub fn clear_flush_counter(&mut self, _tdt_written: TDTWritten) {
        self.flush_counter = 0;
    }

    #[inline(always)]
    pub fn processed_delimiter(&self) -> u16 {
        self.processed_delimiter
    }

    #[inline(always)]
    pub fn flush_counter(&self) -> u64 {
        self.flush_counter
    }
}

