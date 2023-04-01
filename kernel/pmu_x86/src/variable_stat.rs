//! This module implements the equivalent of "perf stat".
//! Currently only 7 events are recorded.
//! 
//! 
//! # Example
//! ```
//! pmu_x86::init();
//! 
//! let counters = pmu_x86::stat::PerformanceCounters::new()?;
//! counters.start();
//! 
//! ...
//! // code to be measured
//! ...
//! 
//! let results = counters.end();
//! 
//! ```


use core::fmt;
use crate::*;
use core::ops::{Sub, SubAssign};

pub struct PerformanceCounters {
    inst_retired: Counter,
    core_cycles: Counter,
    ref_cycles: Counter,
    programmable_events: Vec<Counter>
}

#[derive(Default, Clone, Copy)]
pub struct PMUResults {
    pub inst_retired: u64,
    pub core_cycles: u64,
    pub ref_cycles: u64,
    pub programmable_events: [u64; 8],
}

impl fmt::Debug for PMUResults {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "PMU stat \n 
        instructions retired:   {} \n 
        core cycles:    {} \n 
        reference cycles:   {} \n 
        programmable_events: {:?} \n", 
        self.inst_retired, self.core_cycles, self.ref_cycles, self.programmable_events)
    }
}

impl Sub for PMUResults {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        PMUResults {
            inst_retired: self.inst_retired - rhs.inst_retired,
            core_cycles: self.core_cycles - rhs.core_cycles,
            ref_cycles: self.ref_cycles - rhs.ref_cycles,
            programmable_events: {
                let mut result = [0; 8];
                self.programmable_events.iter().zip(rhs.programmable_events.iter()).enumerate().for_each(|(i, (a, b))| result[i] = *a - *b);
                result
            }
        }
    }
}

impl SubAssign for PMUResults {
    fn sub_assign(&mut self, other: Self)  {
        self.inst_retired -= other.inst_retired;
        self.core_cycles -= other.core_cycles;
        self.ref_cycles -= other.ref_cycles;
        self.programmable_events.iter_mut().zip(other.programmable_events.iter()).for_each(|(a, b)| *a -= *b);
    }
}


impl PerformanceCounters {
    /// Initialize seven performance monitoring counters. They will measure:
    /// - Instructions retired
    /// - Core cycles
    /// - Reference cycles
    /// - LLC references
    /// - LLC misses 
    /// - Branch instructions retired
    /// - Branch misses retired
    pub fn new(programmable_events: [EventType; 8]) -> Result<PerformanceCounters, &'static str> {                
        Ok(PerformanceCounters {
            inst_retired:  Counter::new(EventType::InstructionsRetired)?,
            core_cycles: Counter::new(EventType::UnhaltedCoreCycles)?,    
            ref_cycles: Counter::new(EventType::UnhaltedReferenceCycles)?,    
            programmable_events: vec!(
                Counter::new(programmable_events[0])?,
                Counter::new(programmable_events[1])?,
                Counter::new(programmable_events[2])?,
                Counter::new(programmable_events[3])?,
                Counter::new(programmable_events[4])?,
                Counter::new(programmable_events[5])?,
                Counter::new(programmable_events[6])?,
                Counter::new(programmable_events[7])?,
            )       

        } )
    }

    /// Start running all the counters 
    pub fn start(&mut self) -> Result<(), &'static str>{
        self.ref_cycles.start()?;
        self.core_cycles.start()?;
        self.inst_retired.start()?;        
        for counter in self.programmable_events.iter_mut() {
            counter.start()?;
        }
        Ok(())
    }

    /// Start running all the counters 
    pub fn read(&mut self) -> PMUResults{
        PMUResults {
            inst_retired: self.inst_retired.diff(),
            core_cycles: self.core_cycles.diff(), 
            ref_cycles: self.ref_cycles.diff(), 
            programmable_events: [
                self.programmable_events[0].diff(),
                self.programmable_events[1].diff(),
                self.programmable_events[2].diff(),
                self.programmable_events[3].diff(),
                self.programmable_events[4].diff(),
                self.programmable_events[5].diff(),
                self.programmable_events[6].diff(),
                self.programmable_events[7].diff(),
            ]
        }
    }

    /// Stop the counters and return the counter values.
    /// The `PerformanceCounters` object is consumed since the counters are freed in this function
    /// and should not be accessed again.
    pub fn end(self) -> Result<PMUResults, &'static str> {
        let mut result = [0; 8];
        self.programmable_events.into_iter().enumerate().for_each(|(i, counter)| {result[i] = counter.end().unwrap();});

        Ok( PMUResults {
            inst_retired: self.inst_retired.end()?,
            core_cycles: self.core_cycles.end()?, 
            ref_cycles: self.ref_cycles.end()?, 
            programmable_events: result
        } )
    }
}
