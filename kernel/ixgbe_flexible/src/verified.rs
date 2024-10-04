use crate::hal::descriptors::*;
use crate::mempool::PktBuff;
use crate::spec::*;
use prusti_external_spec::{vec_wrapper::*, vecdeque_wrapper::*};
use prusti_contracts::*;

struct RDTUpdate(u16);
const U16_MAX: usize = 65535;


// https://viperproject.github.io/prusti-dev/user-guide/verify/pledge.html
// https://viperproject.github.io/prusti-dev/user-guide/tour/pledges.html#writing-the-pledge
#[trusted] // has to be trusted to deal with mutable borrow
#[requires(index < s.len())]
#[after_expiry(
    old(s.len()) === s.len() // (1. condition)
    && forall(|i: usize| i < s.len()  &&  i != index // (2. condition)
        ==> old(snap(&s[i])) === snap(&s[i]))
    && snap(&s[index]) === before_expiry(snap(result)) // (3. condition)
)]
fn index_mut<T>(s: &mut [T], index: usize) -> &mut T {
    &mut s[index]
}



#[trusted] // to deal with incomplete bitvector support
#[requires(num_descs > 0)]
#[ensures(result < num_descs)]
#[ensures(result == ((curr_desc + 1) % num_descs))]
fn update_desc(curr_desc: u16, num_descs: u16) -> u16 {
    (curr_desc + 1) & (num_descs as u16 - 1)
}

#[pure] // only used in spec
#[verified]
#[requires(num_descs > 0)]
fn calc_descriptor_rec(curr_desc: u16, add: u16, num_descs: u16) -> u16 {
    if add == 0 {
        curr_desc
    } else {
        (calc_descriptor_rec(curr_desc, add - 1, num_descs) + 1) % num_descs
    }
}


#[verified]
#[requires(desc_ring.len() > 0)]
#[requires((*curr_desc_stored as usize) < desc_ring.len())]
#[requires((*curr_desc_stored as usize) < buffs_in_use.len())]
#[requires(desc_ring.len() == buffs_in_use.len())]
#[requires(desc_ring.len() < U16_MAX)]
#[requires(buffs_in_use.len() < U16_MAX)]
#[requires(batch_size < 32)]
#[ensures(result.0 <= batch_size)]
#[ensures(*curr_desc_stored < desc_ring.len() as u16)]
#[ensures(*curr_desc_stored == calc_descriptor_rec(old(*curr_desc_stored), result.0, desc_ring.len() as u16))]
// #[ensures(forall (|i: u16| i < result.0 ==> {
//     let curr_desc = calc_descriptor_rec(*curr_desc_stored, i, desc_ring.len() as u16);
//     desc_ring[curr_desc as usize].packet_address() == value(pktbuff_addr(&buffs_in_use[curr_desc as usize])) as u64
// }))]
fn receive(
    curr_desc_stored: &mut u16, 
    desc_ring: &mut [AdvancedRxDescriptor],
    buffs_in_use: &mut [PktBuff], 
    buffers: &mut VecWrapper<PktBuff>,
    mempool: &mut VecDequeWrapper<PktBuff>,
    batch_size: u16,
) -> (u16, RDTUpdate) {
    let mut curr_desc = *curr_desc_stored;
    let mut prev_curr_desc = curr_desc;

    let mut rcvd_pkts = 0;
    let _buffers_len_old = buffers.len();
    let mut _buffers_len = buffers.len();

    let mut i = 0;
    while i < batch_size {
        body_invariant!(i < batch_size);
        body_invariant!(desc_ring.len() == buffs_in_use.len());
        body_invariant!(curr_desc < desc_ring.len() as u16);
        body_invariant!(curr_desc < buffs_in_use.len() as u16);
        body_invariant!(rcvd_pkts == i);
        body_invariant!((curr_desc == prev_curr_desc) || curr_desc == (prev_curr_desc + 1) % desc_ring.len() as u16);
        body_invariant!(i > 0 ==> desc_ring[prev_curr_desc as usize].packet_address() == value(pktbuff_addr(&buffs_in_use[prev_curr_desc as usize])) as u64);
        body_invariant!(curr_desc == calc_descriptor_rec(*curr_desc_stored, i, desc_ring.len() as u16));

        let desc = index_mut(desc_ring, curr_desc as usize);
        let (dd, length) = desc.rx_metadata();

        if !dd { break; }

        // if !desc.end_of_packet() {
        //     error!("intel_ethernet::rx_batch(): multi-descriptor packets are not supported yet!");
        //     panic!();
        // }

        // Now that we are "removing" the current receive buffer from the list of receive buffers that the NIC can use,
        // (because we're saving it for higher layers to use),
        // we need to obtain a new `PktBuff` and set it up such that the NIC will use it for future receivals.
        if let Some(new_receive_buf) = mempool.pop_front() {
            let mut current_rx_buf = core::mem::replace(index_mut(buffs_in_use, curr_desc as usize), new_receive_buf);
            
            // actually tell the NIC about the new receive buffer, and that it's ready for use now
            desc.set_packet_address(pktbuff_addr(&buffs_in_use[curr_desc as usize]));

            // unsafe{ core::arch::x86_64::_mm_prefetch(current_rx_buf.buffer.as_ptr() as *const i8, _MM_HINT_ET0);}
            current_rx_buf.length = length as u16; // set the ReceiveBuffer's length to the size of the actual packet received
            buffers.push(current_rx_buf);

            rcvd_pkts += 1;
            prev_curr_desc = curr_desc;
            curr_desc = update_desc(curr_desc, desc_ring.len() as u16);
        } else {
            break;
        }

        i += 1;
    }

    *curr_desc_stored = curr_desc; // should put if condition in case rcvd_pkts = 0, but make sure there's no overhead
    (rcvd_pkts, RDTUpdate(prev_curr_desc) )
}
