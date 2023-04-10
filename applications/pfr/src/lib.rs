//! Application which checks the functionality of the ixgbe driver by sending 1 packet on each enabled queue,
//! and then (optionally) endlessly polling the receive queues.
//! There are two ways to test the nic:
//! 
//! 1. Directly accessing the queues through the main ixgbe nic object. This is the default test.
//! In this case all transmit queues can be tested by sending a packet on each transmit queue, but receive functionality will be tested randomly.
//! If Receive Side Scaling is enabled then received packets will be randomly forwarded to different receive queues.
//! If it's disabled, then all packets will be received on queue 0.
//! 
//! 2. Creating multiple virtual NICs and sending and receiving packets with them.
//! In this case all transmit queues can be tested by sending a packet on each transmit queue, and all receive queues can be tested.
//! Each receive queue is associated with a unique IP address, and packets with that destination IP address will be forwarded to that queue.
//! 
//! For the receiving functionality, you will have to set up a program on another machine that sends packets to the IP addresses assigned
//! to the virtual NICs. Since these are static IP addresses and won't be resolved through ARP, I manually add the (mac address, ip address)
//! pair to the ARP table of the sending machine:  "sudo arp -i interface_name -s ip_address mac_address" 
//! e.g."sudo arp -i eno1 -s 192.168.0.20 0c:c4:7a:d2:ee:1a"

#![no_std]
#![allow(unused_variables)]
#![allow(unused_assignments)]
#![allow(unused_mut)]


extern crate alloc;
#[macro_use] extern crate log;
#[macro_use] extern crate terminal_print;
extern crate network_interface_card;
extern crate ixgbe_restricted;
extern crate spawn;
extern crate getopts;
extern crate hpet;
extern crate libtest;
extern crate pmu_x86;


use alloc::vec::Vec;
use alloc::string::String;
use ixgbe_restricted::{
    get_ixgbe_nics_list,
    IxgbeStats, agent::IxgbeAgent
};
// use packet_buffers::{PacketBufferS, EthernetFrame};
use getopts::{Matches, Options};
use hpet::get_hpet;
use pmu_x86::EventType;

// const DEST_MAC_ADDR: [u8; 6] = [0xa0, 0x36, 0x9f, 0x1d, 0x94, 0x4c];
// const DESC_RING_SIZE: usize = 512;

pub fn main(args: Vec<String>) -> isize {

    let mut opts = Options::new();
    opts.optflag("h", "help", "print this help menu");
    opts.optflag("s", "stats", "collect tx/rx stats and print them out.");
    opts.optflag("p", "pmu", "enable PMU monitoring. The PMU counter values are only printed if the -s flag is enabled as well.");
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

    match rmain(&matches, &opts) {
        Ok(()) => {
            println!("Ixgbe test was successful");
            return 0;
        }
        Err(e) => {
            println!("Ixgbe test failed with error : {:?}", e);
            return -1;
        }
    }
}

fn rmain(matches: &Matches, _opts: &Options) -> Result<(), &'static str> {
    let mut core = 0;
    let mut batch_size = 32;
    let mut packet_length_in_bytes = 64;

    if let Some(i) = matches.opt_default("c", "0") {
        core = i.parse::<u8>().map_err(|_e| "couldn't parse core ID")?;
    }
    
    if let Some(i) = matches.opt_default("b", "32") {
        batch_size = i.parse::<usize>().map_err(|_e| "couldn't parse batch size")?;
    }

    if let Some(i) = matches.opt_default("l", "60") {
        packet_length_in_bytes = i.parse::<u16>().map_err(|_e| "couldn't parse packet length")?;
    }

    let collect_stats = if matches.opt_present("s") { true } else { false };

    let pmu = if matches.opt_present("p") { true } else { false };

    println!("Running packet forwarder on core {}", core);

    let taskref = spawn::new_task_builder(packet_forwarder, (batch_size, packet_length_in_bytes, collect_stats, pmu))
        .name(String::from("packet_forwarder"))
        .pin_on_core(core)
        .spawn()?;

    taskref.join()?;

    Ok(())
}


/// Args are (ixgbe devices, batch size, pmu_enabled, print stats)
fn packet_forwarder(args: (usize, u16, bool, bool)) {
    let batch_size = args.0;
    let _packet_length_in_bytes = args.1;
    let collect_stats = args.2;
    let pmu = args.3;

    // get a handle on the ixgbe nics, there should be at least 2.
    let ixgbe_devs = get_ixgbe_nics_list().expect("ixgbe NICs list not initialized");
    if ixgbe_devs.len() < 2{ 
        error!("Less than 2 ixgbe devices available. Can't run packet forwarder."); 
        return; 
    }
    let mut dev0 = ixgbe_devs[0].lock();
    let mut dev1 = ixgbe_devs[2].lock();


    let src_addr0 = dev0.mac_addr();
    let src_addr1 = dev1.mac_addr();

    // clear the stats registers, and create an object to store the NIC stats during the benchmark
    dev0.clear_stats();
    dev1.clear_stats();
    let mut dev0_stats = IxgbeStats::default();
    let mut dev1_stats = IxgbeStats::default();

    // variables to store the total number of RX and TX packets in each interval
    // these should match what is returned from the NIC stat regsiters.
    let mut rx_packets_dev0 = 0;
    let mut tx_packets_dev0 = 0;
    let mut rx_packets_dev1 = 0;
    let mut tx_packets_dev1 = 0;

    // calculate the #hpet cycles in one second and store
    const NANO_TO_FEMTO: u64 = 1_000_000;
    let hpet = get_hpet().unwrap();
	let hpet_period = hpet.counter_period_femtoseconds() as u64;
    let cycles_in_one_sec = (1_000_000_000 * NANO_TO_FEMTO) / hpet_period;

    // timer variables 
    let mut iterations = 0;
    let mut start_hpet: u64 = hpet.get_counter();
    let mut delta_hpet: u64;
    
    // In the case of the pseudo tx function we go ahead and store a packet buffer for each descriptor,
    // since we aren't doing any buffer management during the benchmark
    // dev0.tx_populate(0, &mut pool0);
    // dev1.tx_populate(0, &mut pool1);

    error!("Link speed: {} Mbps", dev0.link_speed() as usize);
    error!("Link speed: {} Mbps", dev1.link_speed() as usize);
    error!("dev0 fctrl: {:#X} Mbps", dev0.fctrl_flags());
    error!("dev1 fctrl: {:#X} Mbps", dev1.fctrl_flags());

    // start the PMU if enabled
    let mut counters = None;
    if pmu {
        pmu_x86::init().unwrap();
        counters = Some(pmu_x86::variable_stat::PerformanceCounters::new(
            [
                EventType::L2RqstsMiss, EventType::L2RqstsReferences,
                EventType::LastLevelCacheMisses, EventType::LastLevelCacheReferences,
                EventType::MemInstRetiredSTLBMissLoads, EventType::MemInstRetiredSTLBMissStores,
                EventType::MemInstRetiredAllLoads, EventType::MemInstRetiredAllStores,
            ]
        ).expect("Couldn't get performance counters"));
        counters.as_mut().unwrap().start().expect("failed to start counters");
    }

    let src_addr = dev0.mac_addr();
    let mut old_counters = pmu_x86::variable_stat::PMUResults::default();

    // for _ in 0..(DESC_RING_SIZE* 2) / batch_size {
    //     tx_packets_dev1 += dev1.tx_batch(0, batch_size, &mut pool0, &mut pool1) as usize;   
    // }
    // dev1.get_stats(&mut dev1_stats);
    // print_stats(1, &dev1_stats, rx_packets_dev1, tx_packets_dev1);
    // error!("total of {} packets sent", tx_packets_dev1);
    
    let mut pmu_round = 0;
    let mut num_times_zero = 0;
    let mut print = false;

    let mut agent0 = IxgbeAgent::new(&mut dev0, &mut dev1).expect("failed to init agent 0");
    let mut agent1 = IxgbeAgent::new(&mut dev1, &mut dev0).expect("failed to init agent 1");

    // let mut agent0 = IxgbeAgent::new_single(&mut dev0).expect("failed to init agent 0");
    // let mut agent1 = IxgbeAgent::new_single(&mut dev1).expect("failed to init agent 1");
    let mut length0 = 0;
    let mut length1 = 0;

    loop {

        if collect_stats && (iterations & 0xFFF == 0){
            delta_hpet = hpet.get_counter() - start_hpet;

            if delta_hpet >= cycles_in_one_sec {
                dev0.get_stats(&mut dev0_stats);
                dev1.get_stats(&mut dev1_stats);
                print_stats(0, &dev0_stats,  rx_packets_dev0, tx_packets_dev0);
                print_stats(1, &dev1_stats, rx_packets_dev1, tx_packets_dev1);
                rx_packets_dev0 = 0; tx_packets_dev0 = 0; rx_packets_dev1 = 0; tx_packets_dev1 = 0;
                start_hpet = hpet.get_counter();
            }
        }

        // if pmu && rx_packets_dev0 >= 7_400_000 { // as 14.8 MPPS, should get around  7.4 M packets in 30 seconds
        //     delta_hpet = hpet.get_counter() - start_hpet;

        //     if delta_hpet >= 60 * cycles_in_one_sec {
        //         let seconds = delta_hpet as f64 / cycles_in_one_sec as f64;

        //         if pmu_round == 0 {
        //             let new_counters = counters.unwrap().end().unwrap();
        //             let l2_miss_rate = new_counters.programmable_events[0] as f64 / new_counters.programmable_events[1] as f64;
        //             let l3_miss_rate = new_counters.programmable_events[2] as f64 / new_counters.programmable_events[3] as f64;
                    
        //             error!("l2 miss rate = {:.2}", l2_miss_rate);
        //             error!("l3 miss rate = {:.2}", l3_miss_rate);
        //             error!("inst retired = {}", new_counters.inst_retired);
        //             error!("core cycles = {}", new_counters.core_cycles);
        //             error!("ref cycles = {}", new_counters.ref_cycles);
        //             error!("l2 misses = {}",new_counters.programmable_events[0]);
        //             error!("l2 references = {}", new_counters.programmable_events[1]);
        //             error!("l3 misses = {}", new_counters.programmable_events[2]);
        //             error!("l3 references = {}", new_counters.programmable_events[3]);

        //             error!("0: rx pkts = {}, tx pkts = {}", rx_packets_dev0, tx_packets_dev0);
        //             error!("1: rx pkts = {}, tx pkts = {}", rx_packets_dev1, tx_packets_dev1);

        //             let rate = (tx_packets_dev0 + tx_packets_dev1) as f64 / (1_000_000.0 * seconds);
        //             error!("rate = {:.2} Mpps", rate);
        //             error!("cycles / packet = {:.2}", new_counters.ref_cycles as f64 / (tx_packets_dev0 + tx_packets_dev1) as f64);
        //             error!("inst / packet = {:.2}", new_counters.inst_retired as f64 / (tx_packets_dev0 + tx_packets_dev1) as f64);

        //             pmu_round += 1;
        //             rx_packets_dev0 = 0; tx_packets_dev0 = 0; rx_packets_dev1 = 0; tx_packets_dev1 = 0;
        //             start_hpet = hpet.get_counter();

        //             counters = Some(pmu_x86::variable_stat::PerformanceCounters::new(
        //                 [
        //                     EventType::MemInstRetiredSTLBMissLoads, EventType::MemInstRetiredSTLBMissStores,
        //                     EventType::MemInstRetiredAllLoads, EventType::MemInstRetiredAllStores,
        //                     EventType::L2RqstsMiss, EventType::L2RqstsReferences,
        //                     EventType::LastLevelCacheMisses, EventType::LastLevelCacheReferences,
        //                 ]
        //             ).expect("Couldn't get performance counters"));
        //             counters.as_mut().unwrap().start().expect("failed to start counters");

        //         } else {

        //             let new_counters = counters.unwrap().end().unwrap();
        //             let tlb_miss_rate = (new_counters.programmable_events[0] as f64 + new_counters.programmable_events[1] as f64) / (new_counters.programmable_events[2] as f64 + new_counters.programmable_events[3] as f64);
                    
        //             error!("tlb miss rate = {:.2}", tlb_miss_rate);
        //             error!("inst retired = {}", new_counters.inst_retired);
        //             error!("core cycles = {}", new_counters.core_cycles);
        //             error!("ref cycles = {}", new_counters.ref_cycles);
        //             error!("stlb miss loads = {}", new_counters.programmable_events[0]);
        //             error!("stlb miss stores = {}", new_counters.programmable_events[1]);
        //             error!("loads = {}", new_counters.programmable_events[2]);
        //             error!("stores = {}", new_counters.programmable_events[3]);
        //             error!("0: rx pkts = {}, tx pkts = {}", rx_packets_dev0, tx_packets_dev0);
        //             error!("1: rx pkts = {}, tx pkts = {}", rx_packets_dev1, tx_packets_dev1);

        //             let rate = (tx_packets_dev0 + tx_packets_dev1) as f64 / (1_000_000.0 * seconds);
        //             error!("rate = {:.2} Mpps", rate);
        //             error!("cycles / packet = {:.2}", new_counters.ref_cycles as f64 / (tx_packets_dev0 + tx_packets_dev1) as f64);
        //             error!("inst / packet = {:.2}", new_counters.inst_retired as f64 / (tx_packets_dev0 + tx_packets_dev1) as f64);

        //             break;
        //         }
        //     }
        // }


        /*** bidirectional forwarder 2 ports (tested till 8.8 Mpps)***/

        // rx_packets_dev0 += agent0.rx(&mut length0);
        // rx_packets_dev1 += agent1.rx(&mut length1);

        // tx_packets_dev0 += agent0.tx();
        // tx_packets_dev1 += agent1.tx();

        // rx_packets_dev0 += agent0.run(print);
        // rx_packets_dev1 += agent1.run(print);

        agent0.run();
        agent1.run();
        
        // print = false;
        // if rx_packets_dev0 == 0 {
        //     num_times_zero += 1;
        //     if num_times_zero == 1_000_000 {
        //         print = true;
        //         num_times_zero = 0;
        //     }
        // }

        // if collect_stats {
        //     iterations += 1;
        // }

    }
}


fn print_stats(device_id: usize, dev_stats: &IxgbeStats, rx_packets: usize, tx_packets: usize) {
    error!("DEV{}: RX reg: {} Gb/s | {} Mpps all | {} Mpps", 
        device_id, (dev_stats.rx_bytes  as f32 * 8.0) / (1_000_000_000.0), dev_stats.rx_packets as f32/ (1_000_000.0), rx_packets as f32/ (1_000_000.0));
    error!("DEV{}: TX reg: {} Gb/s | {} Mpps all | {} Mpps", 
        device_id, (dev_stats.tx_bytes  as f32 * 8.0) / (1_000_000_000.0), dev_stats.tx_packets as f32/ (1_000_000.0), tx_packets as f32/ (1_000_000.0));
}

fn print_usage(opts: &Options) {
    println!("{}", opts.usage(USAGE));
}

const USAGE: &'static str = "Usage: packet_forwarder [ARGS]
Runs a abasic throughput test for the ixgbe NIC.";
