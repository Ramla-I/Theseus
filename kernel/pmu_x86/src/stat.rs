//! This module allows you to choose the events recorded by the programmable performance counters.
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
use alloc::vec;

pub struct PerformanceCounters {
    inst_retired: Counter,
    core_cycles: Counter,
    ref_cycles: Counter,
    programmable_events: [Counter; PMCS_SUPPORTED_BY_PMU as usize]
}

#[derive(Clone, Copy)]
pub struct PMUResults {
    pub inst_retired: u64,
    pub core_cycles: u64,
    pub ref_cycles: u64,
    pub programmable_events: [(EventType, u64); PMCS_SUPPORTED_BY_PMU as usize],
}

impl PMUResults {
    pub fn empty(counters: &PerformanceCounters) -> PMUResults {
        PMUResults {
            inst_retired: 0,
            core_cycles: 0,
            ref_cycles: 0,
            programmable_events: [
                (counters.programmable_events[0].event_type, 0),
                (counters.programmable_events[1].event_type, 0),
                (counters.programmable_events[2].event_type, 0),
                (counters.programmable_events[3].event_type, 0),
                (counters.programmable_events[4].event_type, 0),
                (counters.programmable_events[5].event_type, 0),
                (counters.programmable_events[6].event_type, 0),
                (counters.programmable_events[7].event_type, 0),
            ]
        }
    }
}
impl fmt::Debug for PMUResults {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "PMU stat \n 
        instructions retired:   {} \n 
        core cycles:    {} \n 
        reference cycles:   {} \n 
        {:?}: {:?} \n
        {:?}: {:?} \n 
        {:?}: {:?} \n 
        {:?}: {:?} \n 
        {:?}: {:?} \n 
        {:?}: {:?} \n 
        {:?}: {:?} \n 
        {:?}: {:?} \n", 
        self.inst_retired, self.core_cycles, self.ref_cycles, 
        self.programmable_events[0].0, self.programmable_events[0].1,
        self.programmable_events[1].0, self.programmable_events[1].1,
        self.programmable_events[2].0, self.programmable_events[2].1,
        self.programmable_events[3].0, self.programmable_events[3].1,
        self.programmable_events[4].0, self.programmable_events[4].1,
        self.programmable_events[5].0, self.programmable_events[5].1,
        self.programmable_events[6].0, self.programmable_events[6].1,
        self.programmable_events[7].0, self.programmable_events[7].1)
    }
}

impl Sub for PMUResults {
    type Output = Self;

    // Warning! User should make sure that counter events match!
    fn sub(self, rhs: Self) -> Self::Output {
        PMUResults {
            inst_retired: self.inst_retired - rhs.inst_retired,
            core_cycles: self.core_cycles - rhs.core_cycles,
            ref_cycles: self.ref_cycles - rhs.ref_cycles,
            programmable_events: [ 
                // so ugly but I don't want to make a default EventType so that we can update an array in an iterator
                // array must be set with a default value first
                (self.programmable_events[0].0, self.programmable_events[0].1 - rhs.programmable_events[0].1),
                (self.programmable_events[1].0, self.programmable_events[1].1 - rhs.programmable_events[1].1),
                (self.programmable_events[2].0, self.programmable_events[2].1 - rhs.programmable_events[2].1),
                (self.programmable_events[3].0, self.programmable_events[3].1 - rhs.programmable_events[3].1),
                (self.programmable_events[4].0, self.programmable_events[4].1 - rhs.programmable_events[4].1),
                (self.programmable_events[5].0, self.programmable_events[5].1 - rhs.programmable_events[5].1),
                (self.programmable_events[6].0, self.programmable_events[6].1 - rhs.programmable_events[6].1),
                (self.programmable_events[7].0, self.programmable_events[7].1 - rhs.programmable_events[7].1),
            ]
        }
    }
}

impl SubAssign for PMUResults {
    // Warning! User should make sure that counter events match!
    fn sub_assign(&mut self, other: Self)  {
        self.inst_retired -= other.inst_retired;
        self.core_cycles -= other.core_cycles;
        self.ref_cycles -= other.ref_cycles;
        self.programmable_events.iter_mut().zip(other.programmable_events.iter()).for_each(|((_, a), (_, b))| *a -= *b)
    }
}


impl PerformanceCounters {

    /// Initialize 11 performance monitoring counters with the events given by "perf stat"
    pub fn new_perf_stat() -> Result<PerformanceCounters, &'static str> {                
        Ok(PerformanceCounters {
            inst_retired:  Counter::new(EventType::InstructionsRetired)?,
            core_cycles: Counter::new(EventType::UnhaltedCoreCycles)?,    
            ref_cycles: Counter::new(EventType::UnhaltedReferenceCycles)?,    
            programmable_events: [
                Counter::new(EventType::LastLevelCacheReferences)?,       
                Counter::new(EventType::LastLevelCacheMisses)?,
                Counter::new(EventType::BranchInstructionsRetired)?,   
                Counter::new(EventType::BranchMissesRetired)?,
                Counter::new(EventType::MemInstRetiredAllLoads)?,
                Counter::new(EventType::MemInstRetiredAllStores)?,
                Counter::new(EventType::DTLBLoadMissesMissCausesAWalk)?,
                Counter::new(EventType::DTLBStoreMissesMissCausesAWalk)?,
            ]    
        } )
    }

    /// Initialize 11 performance monitoring counters.
    pub fn new(programmable_events: [EventType; 8]) -> Result<PerformanceCounters, &'static str> {                
        Ok(PerformanceCounters {
            inst_retired:  Counter::new(EventType::InstructionsRetired)?,
            core_cycles: Counter::new(EventType::UnhaltedCoreCycles)?,    
            ref_cycles: Counter::new(EventType::UnhaltedReferenceCycles)?,    
            programmable_events: [
                Counter::new(programmable_events[0])?,
                Counter::new(programmable_events[1])?,
                Counter::new(programmable_events[2])?,
                Counter::new(programmable_events[3])?,
                Counter::new(programmable_events[4])?,
                Counter::new(programmable_events[5])?,
                Counter::new(programmable_events[6])?,
                Counter::new(programmable_events[7])?,
            ]    

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
                (self.programmable_events[0].event_type, self.programmable_events[0].diff()),
                (self.programmable_events[1].event_type, self.programmable_events[1].diff()),
                (self.programmable_events[2].event_type, self.programmable_events[2].diff()),
                (self.programmable_events[3].event_type, self.programmable_events[3].diff()),
                (self.programmable_events[4].event_type, self.programmable_events[4].diff()),
                (self.programmable_events[5].event_type, self.programmable_events[5].diff()),
                (self.programmable_events[6].event_type, self.programmable_events[6].diff()),
                (self.programmable_events[7].event_type, self.programmable_events[7].diff()),
            ]
        }
    }

    /// Stop the counters and return the counter values.
    /// The `PerformanceCounters` object is consumed since the counters are freed in this function
    /// and should not be accessed again.
    pub fn end(self) -> Result<PMUResults, &'static str> {
        // so ugly but I don't want to make a default EventType
        let events = [self.programmable_events[0].event_type, self.programmable_events[1].event_type, self.programmable_events[2].event_type, self.programmable_events[3].event_type, self.programmable_events[4].event_type, self.programmable_events[5].event_type, self.programmable_events[6].event_type, self.programmable_events[7].event_type];
        
        // can't use collect because end consumes the iterator
        let mut end_counter = [0; PMCS_SUPPORTED_BY_PMU as usize];
        self.programmable_events.into_iter().enumerate().for_each(|(i, counter)| end_counter[i] = counter.end().unwrap());

        Ok( PMUResults {
            inst_retired: self.inst_retired.end()?,
            core_cycles: self.core_cycles.end()?, 
            ref_cycles: self.ref_cycles.end()?, 
            programmable_events: [
                (events[0], end_counter[0]),
                (events[1], end_counter[1]),
                (events[2], end_counter[2]),
                (events[3], end_counter[3]),
                (events[4], end_counter[4]),
                (events[5], end_counter[5]),
                (events[6], end_counter[6]),
                (events[7], end_counter[7]),
            ]
        } )
    }
}