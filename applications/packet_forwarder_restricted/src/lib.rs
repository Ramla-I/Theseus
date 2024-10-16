#![no_std]
#![allow(dead_code)]

extern crate alloc;

use alloc::{vec::Vec, string::String};
use ixgbe_restricted::{get_ixgbe_nics_list, IxgbeStats, agent::IxgbeAgent};
use getopts::{Matches, Options};
use hpet::get_hpet;
use pmu_x86::{stat::{PMUResults, PerformanceCounters}, EventType};
use app_io::println;
use libtest::pick_free_core_with_cpu_id;
use log::{info, error};

// const DEST_MAC_ADDR: [u8; 6] = [0xa0, 0x36, 0x9f, 0x1d, 0x94, 0x4c];
// const DESC_RING_SIZE: usize = 512;

pub fn main(args: Vec<String>) -> isize {

    let mut opts = Options::new();
    opts.optflag("h", "help", "print this help menu");
    opts.optflag("s", "stats", "collect tx/rx stats and print them out.");
    opts.optflag("p", "pmu", "enable PMU monitoring. You can only enable stats or pmu, not both.");
    opts.optopt("c", "core", "core to run the packet forwarder thread on", "CPU_ID");
    opts.optopt("b", "batch_size", "batch size for Rx and Tx", "BATCH");
    opts.optopt("l", "length", "length of packets to be received/ transmitted in bytes", "LENGTH");


    let matches = match opts.parse(&args) {
        Ok(m) => m,
        Err(_f) => {
            println!("{}", _f);
            print_usage(&opts);
            return -1; 
        }
    };

    if matches.opt_present("h") {
        print_usage(&opts);
        return 0;
    }

    // Enable either stats or pmu, not both
    if matches.opt_present("s") && matches.opt_present("p") {
        print_usage(&opts);
        return 0;
    }

    match rmain(&matches, &opts) {
        Ok(()) => {
            println!("PACKET FORWARDER TEST COMPLETED");
            return 0;
        }
        Err(e) => {
            println!("PACKET FORWARDER FAILED WITH ERROR : {:?}", e);
            return -1;
        }
    }
}

fn rmain(matches: &Matches, _opts: &Options) -> Result<(), &'static str> {
    let core = matches.opt_default("c", "0").unwrap_or(String::from("0")).parse::<u32>().map_err(|_e| "couldn't parse core ID")?;
    let batch_size = matches.opt_default("b", "32").unwrap_or(String::from("32")).parse::<usize>().map_err(|_e| "couldn't parse batch size")?;
    let packet_length_in_bytes = matches.opt_default("l", "60").unwrap_or(String::from("60")).parse::<u16>().map_err(|_e| "couldn't parse packet length")?;
    let collect_stats = matches.opt_present("s");
    let pmu = matches.opt_present("p");

    let args = PacketForwarderArgs {
        batch_size,
        packet_length_in_bytes,
        collect_stats,
        pmu,
    };

    let cpu_id = pick_free_core_with_cpu_id(core)?;
    println!("STARTING PACKET FORWARDER ON CORE {}", cpu_id);

    let taskref = spawn::new_task_builder(packet_forwarder, args)
        .name(String::from("packet_forwarder"))
        .pin_on_cpu(cpu_id)
        .spawn()?;

    taskref.join()?;

    Ok(())
}
struct PacketForwarderArgs {
    batch_size: usize,
    packet_length_in_bytes: u16,
    collect_stats: bool,
    pmu: bool,
}

fn packet_forwarder(mut args: PacketForwarderArgs) {
    // get a handle on the ixgbe nics, there should be at least 2.
    let ixgbe_devs = get_ixgbe_nics_list().expect("ixgbe NICs list not initialized");
    if ixgbe_devs.len() < 2 {  
        error!("Fewer than 2 ixgbe devices available. Can't run packet forwarder."); 
        return;
    }
    let mut dev0 = ixgbe_devs[0].lock(); // To Do: Make this variable
    let mut dev1 = ixgbe_devs[2].lock();
    info!("Link speed: {} Mbps", dev0.link_speed() as usize);
    info!("Link speed: {} Mbps", dev1.link_speed() as usize);

    // calculate the #hpet cycles in one second and store
    const NANO_TO_FEMTO: u64 = 1_000_000;
    let hpet = get_hpet().unwrap();
	let hpet_period = hpet.counter_period_femtoseconds() as u64;
    let cycles_in_one_sec = (1_000_000_000 * NANO_TO_FEMTO) / hpet_period;

    // stats variables
    let mut dev0_stats = IxgbeStats::default();
    let mut dev1_stats = IxgbeStats::default();
    let mut self_stats = SelfCalculatedPacketStats::default();

    // clear the stats
    if args.collect_stats {
        dev0.clear_stats();
        dev1.clear_stats();
    }

    // Initialize the ixgbe agents
    let mut agent0 = IxgbeAgent::new(&mut dev0, &mut dev1).expect("failed to init agent 0");
    let mut agent1 = IxgbeAgent::new(&mut dev1, &mut dev0).expect("failed to init agent 1");

    // start the PMU if enabled
    let mut counters = None;
    if args.pmu {
        pmu_x86::init().expect("PMU not available!");
        counters = Some(create_and_start_counters());
    }

    // timer variables 
    let mut iterations = 0;
    let mut start_hpet: u64 = hpet.get_counter();
    let mut delta_hpet: u64;

    loop {
        if args.collect_stats && (iterations & 0xFFFF == 0){
            delta_hpet = hpet.get_counter() - start_hpet;

            if delta_hpet >= cycles_in_one_sec { // print once every second
                dev0.get_stats(&mut dev0_stats);
                dev1.get_stats(&mut dev1_stats);
                print_stats(0, &dev0_stats,  &mut self_stats.rx_packets_dev0, &mut self_stats.tx_packets_dev0);
                print_stats(1, &dev1_stats, &mut self_stats.rx_packets_dev1, &mut self_stats.tx_packets_dev1);
                start_hpet = hpet.get_counter();
            }
        }

        if args.pmu && (iterations & 0xFFFF == 0) { 
            delta_hpet = hpet.get_counter() - start_hpet;

            if delta_hpet >= 60 * cycles_in_one_sec { // print after a minute
                let seconds = delta_hpet as f64 / cycles_in_one_sec as f64;
                print_pmu(&counters.unwrap().end().unwrap(), &mut self_stats, seconds);

                counters = Some(create_and_start_counters());
                start_hpet = hpet.get_counter();
            }
        }


    //     /*** bidirectional forwarder 2 ports (tested till 8.8 Mpps)***/

    //     // rx_packets_dev0 += agent0.rx(&mut length0);
    //     // rx_packets_dev1 += agent1.rx(&mut length1);

    //     // tx_packets_dev0 += agent0.tx();
    //     // tx_packets_dev1 += agent1.tx();

    //     // rx_packets_dev0 += agent0.run(print);
    //     // rx_packets_dev1 += agent1.run(print);

        agent0.run();
        agent1.run();
        
        if args.collect_stats {
            iterations += 1;
        }
    }
}

#[derive(Debug, Default)]
struct SelfCalculatedPacketStats {
    rx_packets_dev0: usize,
    tx_packets_dev0: usize,
    rx_packets_dev1: usize,
    tx_packets_dev1: usize
}
impl SelfCalculatedPacketStats {
    fn reset(&mut self) {
        self.rx_packets_dev0 = 0;
        self.tx_packets_dev0 = 0;
        self.rx_packets_dev1 = 0;
        self.tx_packets_dev1 = 0;
    }
}

fn create_and_start_counters() -> PerformanceCounters {
    let mut counters = PerformanceCounters::new(
        [
            EventType::L2RqstsMiss, EventType::L2RqstsReferences,
            EventType::LastLevelCacheMisses, EventType::LastLevelCacheReferences,
            EventType::MemInstRetiredSTLBMissLoads, EventType::MemInstRetiredSTLBMissStores,
            EventType::MemInstRetiredAllLoads, EventType::MemInstRetiredAllStores,
        ]
    ).expect("Couldn't get performance counters");
    counters.start().expect("failed to start counters");
    counters
}

fn print_pmu(pmu_result: &PMUResults, self_stats: &mut SelfCalculatedPacketStats, seconds: f64) {
    let l2_miss_rate = pmu_result.programmable_events[0].1 as f64 / pmu_result.programmable_events[1].1 as f64;
    let l3_miss_rate = pmu_result.programmable_events[2].1 as f64 / pmu_result.programmable_events[3].1 as f64;
    let tlb_miss_rate = (pmu_result.programmable_events[4].1 as f64 + pmu_result.programmable_events[5].1 as f64) / (pmu_result.programmable_events[6].1 as f64 + pmu_result.programmable_events[7].1 as f64);
    
    info!("l2 miss rate = {:.2}", l2_miss_rate);
    info!("l3 miss rate = {:.2}", l3_miss_rate);
    info!("stlb miss rate = {:.2}", tlb_miss_rate);

    info!("{:?}", pmu_result);
    info!("0: rx pkts = {}, tx pkts = {}", self_stats.rx_packets_dev0, self_stats.tx_packets_dev0);
    info!("1: rx pkts = {}, tx pkts = {}", self_stats.rx_packets_dev1, self_stats.tx_packets_dev1);

    let rate = (self_stats.tx_packets_dev0 + self_stats.tx_packets_dev1) as f64 / (1_000_000.0 * seconds);
    info!("rate = {:.2} Mpps", rate);
    info!("cycles / packet = {:.2}", pmu_result.ref_cycles as f64 / (self_stats.tx_packets_dev0 + self_stats.tx_packets_dev1) as f64);
    info!("inst / packet = {:.2}", pmu_result.inst_retired as f64 / (self_stats.tx_packets_dev0 + self_stats.tx_packets_dev1) as f64);

    self_stats.reset();
}

fn print_stats(device_id: usize, dev_stats: &IxgbeStats, rx_packets: &mut usize, tx_packets: &mut usize) {
    info!("DEV{}: RX reg: {} Gb/s | {} Mpps all | {} Mpps", 
        device_id, (dev_stats.rx_bytes  as f32 * 8.0) / (1_000_000_000.0), dev_stats.rx_packets as f32/ (1_000_000.0), *rx_packets as f32/ (1_000_000.0));
    info!("DEV{}: TX reg: {} Gb/s | {} Mpps all | {} Mpps", 
        device_id, (dev_stats.tx_bytes  as f32 * 8.0) / (1_000_000_000.0), dev_stats.tx_packets as f32/ (1_000_000.0), *tx_packets as f32/ (1_000_000.0));

    *rx_packets = 0; *tx_packets = 0;
}

fn print_usage(opts: &Options) {
    println!("{}", opts.usage(USAGE));
}

const USAGE: &'static str = "Usage: packet_forwarder [ARGS]
Runs a a basic throughput test for the ixgbe NIC.";