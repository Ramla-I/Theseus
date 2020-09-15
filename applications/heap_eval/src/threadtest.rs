//! A Rust version of the threadtest heap microbenchmark
//! 
//! The original version was presented in the Hoard paper
//! https://github.com/emeryberger/Hoard/tree/master/benchmarks/threadtest

use alloc::{
    vec::Vec,
    string::String,
    alloc::Layout
};
#[cfg(not(direct_access_to_multiple_heaps))]
use alloc::alloc::GlobalAlloc;
use core::sync::atomic::{Ordering, AtomicUsize};
use core::ptr;
use hpet::get_hpet;
use libtest::{hpet_2_us, calculate_stats, hpet_timing_overhead};
use crate::{NTHREADS, ALLOCATOR, TRIES};
#[cfg(direct_access_to_multiple_heaps)]
use crate::overhead_of_accessing_multiple_heaps;

pub(crate) static NITERATIONS: AtomicUsize = AtomicUsize::new(1000);
/// Sum total of objects to be allocated by all threads
pub(crate) static NOBJECTS: AtomicUsize = AtomicUsize::new(100_000);
/// Size of the objects we're allocating in bytes
pub(crate) static OBJSIZE: AtomicUsize = AtomicUsize::new(REGULAR_SIZE);
/// The default size of objects to allocate
const REGULAR_SIZE: usize = 8;
/// The size allocated when the large allocations option is chosen
pub const LARGE_SIZE: usize = 8192;



cfg_if! {
if #[cfg(heap_fragmentation_eval)] {
    use heap::accounting::HeapAccounting;
    use spin::Mutex;

    static MAX_ALLOCATED: AtomicUsize = AtomicUsize::new(0);
    static MAX_USED: AtomicUsize = AtomicUsize::new(0);
    static MAX_FRAG_ALLOCATED: AtomicUsize = AtomicUsize::new(0);
    static MAX_FRAG_USED: AtomicUsize = AtomicUsize::new(1);
    static RECORD_FRAGMENTATION: AtomicUsize = AtomicUsize::new(0);

    lazy_static! {
        pub static ref HEAPS_FOR_FRAGMENTATION: Mutex<Vec<usize>> = Mutex::new(Vec::new());
    }

    pub fn do_threadtest() -> Result<(), &'static str> {

        let nthreads = NTHREADS.load(Ordering::SeqCst);
        // NTHREADS.store(1, Ordering::SeqCst);

        let hpet_overhead = hpet_timing_overhead()?;
        let hpet = get_hpet().ok_or("couldn't get HPET timer")?;

        println!("Running threadtest (FRAGMENTATION) for {} threads, {} iterations, {} total objects allocated every iteration by all threads, {} obj size ...", 
            nthreads, NITERATIONS.load(Ordering::SeqCst), NOBJECTS.load(Ordering::SeqCst), OBJSIZE.load(Ordering::SeqCst));

        let my_core = apic::get_my_apic_id();
        let worker_core = runqueue::get_least_busy_core().unwrap() as usize;

        let mut threads = Vec::with_capacity(nthreads);
        for _ in 0..nthreads {
            let task = spawn::new_task_builder(worker, ()).name(String::from("worker thread")).pin_on_core(worker_core as u8).block().spawn()?;
            // println!("task id: {}", task.id());
            threads.push(task);
        }  

        let mut heap_ids = Vec::with_capacity(nthreads);

        #[cfg(task_heaps)] 
        {
            println!("using per-TASK heaps");
            for thread in &threads {
                heap_ids.push(thread.id());
            }
        }

        #[cfg(not(task_heaps))]
        {
            println!("using per-CORE heaps");
            println!("start allocated: {}, start used: {}", ALLOCATOR.allocated(worker_core), ALLOCATOR.used(worker_core));
            heap_ids.push(worker_core as usize);
        }

        *HEAPS_FOR_FRAGMENTATION.lock() = heap_ids;

        let start = hpet.get_counter();

        for thread in &threads {
            thread.unblock();
        }

        for i in 0..nthreads {
            threads[i].join()?;
        }

        let end = hpet.get_counter() - hpet_overhead;

        // Don't want this to be part of the timing measurement
        for thread in threads {
            thread.take_exit_value();
        }

        let diff = hpet_2_us(end - start);
        println!("threadtest time: {} us", diff);
        unsafe{ println!("{} {}", MAX_ALLOCATED.load(Ordering::SeqCst), MAX_USED.load(Ordering::SeqCst)); }
        unsafe{ println!("{} {} {}", MAX_FRAG_ALLOCATED.load(Ordering::SeqCst), MAX_FRAG_USED.load(Ordering::SeqCst), MAX_FRAG_ALLOCATED.load(Ordering::SeqCst) as f32/MAX_FRAG_USED.load(Ordering::SeqCst) as f32); }


        Ok(())
    }

    fn worker(_:()) {
        let heap_ids = HEAPS_FOR_FRAGMENTATION.lock().clone();

        #[cfg(not(direct_access_to_multiple_heaps))]
        let allocator = &ALLOCATOR;

        // In the case of directly accessing the multiple heaps, we do have to access them through the Once wrapper
        // at the beginning, but the time it takes to do this once at the beginning of thread is
        // insignificant compared to the number of iterations we run. It also printed above.
        #[cfg(direct_access_to_multiple_heaps)]
        let allocator = match ALLOCATOR.try() {
            Some(allocator) => allocator,
            None => {
                error!("Multiple heaps not initialized!");
                return;
            }
        };

        let niterations = NITERATIONS.load(Ordering::SeqCst);
        let nobjects = NOBJECTS.load(Ordering::SeqCst);
        let nthreads = NTHREADS.load(Ordering::SeqCst);
        let obj_size = OBJSIZE.load(Ordering::SeqCst);

        let mut allocations = Vec::with_capacity(nobjects/nthreads);
        // initialize the vector so we do not measure the time of `push` and `pop`
        for _ in 0..(nobjects / nthreads) {
            allocations.push(ptr::null_mut());
        }
        let layout = Layout::from_size_align(obj_size, 8).unwrap();

        RECORD_FRAGMENTATION.fetch_add(1, Ordering::SeqCst);
        for _ in 0..niterations {
            for i in 0..(nobjects/nthreads) {
                let ptr = unsafe{ allocator.alloc(layout) };
                allocations[i] = ptr;
                update_allocated_and_used(&heap_ids);
            }
            for i in 0..(nobjects/nthreads) {
                unsafe{ allocator.dealloc(allocations[i], layout); }
                update_allocated_and_used(&heap_ids);
            }
        }  
    }

    fn update_allocated_and_used(heap_ids: &Vec<usize>) {
        if RECORD_FRAGMENTATION.load(Ordering::SeqCst) != NTHREADS.load(Ordering::SeqCst) { return; }
        let mut allocated = 0;
        let mut used = 0;

        for id in heap_ids {
            let(a,u) = ALLOCATOR.allocated_and_used(*id);
            allocated += a;
            used += u;
        }

        unsafe {
            if allocated > MAX_ALLOCATED.load(Ordering::SeqCst) {
                MAX_ALLOCATED.store(allocated, Ordering::SeqCst);
            }
            if used > MAX_USED.load(Ordering::SeqCst) {
                MAX_USED.store(used, Ordering::SeqCst);
            }
            if allocated > MAX_FRAG_ALLOCATED.load(Ordering::SeqCst) {
                MAX_FRAG_ALLOCATED.store(allocated, Ordering::SeqCst);
                MAX_FRAG_USED.store(used, Ordering::SeqCst);
            }                
        }
    }

} else {

    pub fn do_threadtest() -> Result<(), &'static str> {

        let nthreads = NTHREADS.load(Ordering::SeqCst);
        let mut tries = Vec::with_capacity(TRIES as usize);

        let hpet_overhead = hpet_timing_overhead()?;
        let hpet = get_hpet().ok_or("couldn't get HPET timer")?;

        println!("Running threadtest for {} threads, {} iterations, {} total objects allocated every iteration by all threads, {} obj size ...", 
            nthreads, NITERATIONS.load(Ordering::SeqCst), NOBJECTS.load(Ordering::SeqCst), OBJSIZE.load(Ordering::SeqCst));

        #[cfg(direct_access_to_multiple_heaps)]
        {
            let overhead = overhead_of_accessing_multiple_heaps()?;
            println!("Overhead of accessing multiple heaps is: {} ticks, {} ns", overhead, hpet_2_us(overhead));
        }

        for try in 0..TRIES {
            let mut threads = Vec::with_capacity(nthreads);

            let start = hpet.get_counter();

            for _ in 0..nthreads {
                threads.push(spawn::new_task_builder(worker, ()).name(String::from("worker thread")).spawn()?);
            }  

            for i in 0..nthreads {
                threads[i].join()?;
            }

            let end = hpet.get_counter() - hpet_overhead;

            // Don't want this to be part of the timing measurement
            for thread in threads {
                thread.take_exit_value();
            }

            let diff = hpet_2_us(end - start);
            println!("[{}] threadtest time: {} us", try, diff);
            tries.push(diff);
        }

        println!("threadtest stats (us)");
        println!("{:?}", calculate_stats(&tries));

        Ok(())
    }

    fn worker(_:()) {
        #[cfg(not(direct_access_to_multiple_heaps))]
        let allocator = &ALLOCATOR;

        // In the case of directly accessing the multiple heaps, we do have to access them through the Once wrapper
        // at the beginning, but the time it takes to do this once at the beginning of thread is
        // insignificant compared to the number of iterations we run. It also printed above.
        #[cfg(direct_access_to_multiple_heaps)]
        let allocator = match ALLOCATOR.try() {
            Some(allocator) => allocator,
            None => {
                error!("Multiple heaps not initialized!");
                return;
            }
        };

        let niterations = NITERATIONS.load(Ordering::SeqCst);
        let nobjects = NOBJECTS.load(Ordering::SeqCst);
        let nthreads = NTHREADS.load(Ordering::SeqCst);
        let obj_size = OBJSIZE.load(Ordering::SeqCst);

        let mut allocations = Vec::with_capacity(nobjects/nthreads);
        // initialize the vector so we do not measure the time of `push` and `pop`
        for _ in 0..(nobjects / nthreads) {
            allocations.push(ptr::null_mut());
        }
        let layout = Layout::from_size_align(obj_size, 8).unwrap();

        for _ in 0..niterations {
            for i in 0..(nobjects/nthreads) {
                let ptr = unsafe{ allocator.alloc(layout) };
                allocations[i] = ptr;
            }
            for i in 0..(nobjects/nthreads) {
                unsafe{ allocator.dealloc(allocations[i], layout); }
            }
        }  
    }
}
} //end cfg if





