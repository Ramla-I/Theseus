use crate::hal::{regs::ReportStatusBit, descriptors::*};
use crate::mempool::{PktBuff, pktbuff_addr};
use crate::{FilterParameters, FilterError};
#[allow(unused_imports)]
use prusti_external_spec::{vec_wrapper::*, vecdeque_wrapper::*, trusted_option::*, trusted_result::*};
use prusti_contracts::*;

pub struct RDTUpdate(u16);
impl RDTUpdate {
    pub fn value(&self) -> u16 {
        self.0
    }
}

pub struct TDTUpdate(u16);
impl TDTUpdate {
    pub fn value(&self) -> u16 {
        self.0
    }
}

const U16_MAX: usize = 65535;


#[verified]
#[requires(desc_ring.len() > 0)]
#[requires((*curr_desc_stored as usize) < desc_ring.len() && (*curr_desc_stored as usize) < buffs_in_use.len())]
#[requires(desc_ring.len() == buffs_in_use.len())]
#[requires(desc_ring.len() < U16_MAX)]
#[requires(buffs_in_use.len() < U16_MAX)]
#[ensures(desc_ring.len() == old(desc_ring.len()))]
#[ensures(*curr_desc_stored < desc_ring.len() as u16)]
pub(crate) fn receive(
    curr_desc_stored: &mut u16, 
    desc_ring: &mut [AdvancedRxDescriptor],
    buffs_in_use: &mut [PktBuff], 
    buffers: &mut VecWrapper<PktBuff>,
    mempool: &mut VecDequeWrapper<PktBuff>,
    batch_size: u16,
) -> (u16, RDTUpdate) {
    let _orig_curr_desc = *curr_desc_stored;
    let _orig_buffs_in_use_len = buffs_in_use.len();
    let _orig_desc_ring_len = desc_ring.len();


    let mut curr_desc = *curr_desc_stored;
    let mut prev_curr_desc = curr_desc;
    let mut rcvd_pkts = 0;

    let mut i = 0;
    while i < batch_size {
        body_invariant!(i < batch_size);
        body_invariant!(desc_ring.len() == buffs_in_use.len());
        body_invariant!(desc_ring.len() == _orig_desc_ring_len && buffs_in_use.len() == _orig_buffs_in_use_len);
        body_invariant!(curr_desc < desc_ring.len() as u16 && curr_desc < buffs_in_use.len() as u16);
        // body_invariant!(i > 0 ==> curr_desc == (prev_curr_desc + 1) % desc_ring.len() as u16);
        // body_invariant!(i > 0 ==> desc_ring[prev_curr_desc as usize].packet_address() == pktbuff_addr(&buffs_in_use[prev_curr_desc as usize]).value() as u64);

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

#[verified]
#[requires(desc_ring.len() > 0)]
#[requires((*curr_desc_stored as usize) < desc_ring.len())]
#[requires(desc_ring.len() < U16_MAX && buffs_in_use.len() < U16_MAX)]
#[ensures(desc_ring.len() == old(desc_ring.len()))]
#[ensures(*curr_desc_stored < desc_ring.len() as u16)]
#[ensures(buffs_in_use.len() == old(buffs_in_use.len()) + result.0 as usize)]
pub(crate) fn transmit(
    curr_desc_stored: &mut u16, 
    tx_clean: u16,
    desc_ring: &mut [AdvancedTxDescriptor],
    buffs_in_use: &mut VecWrapper<PktBuff>,
    buffers: &mut VecWrapper<PktBuff>,
    batch_size: u16,
    rs_bit: ReportStatusBit
) -> (u16, TDTUpdate) {
    const TX_CLEAN_THRESHOLD: u16 = 64; // make sure this is less than and an even divisor to the queue size

    let _orig_buffs_in_use_len = buffs_in_use.len();
    let _orig_desc_ring_len = desc_ring.len();

    let mut curr_desc = *curr_desc_stored;
    let mut next_desc = curr_desc;
    let mut _prev_curr_desc = curr_desc;

    let mut sent_pkts = 0;
    
    let mut i = 0;
    while i < batch_size {
        body_invariant!(i < batch_size && i == sent_pkts);
        body_invariant!(desc_ring.len() == _orig_desc_ring_len);
        body_invariant!(curr_desc == next_desc);
        body_invariant!(next_desc < desc_ring.len() as u16 && curr_desc < desc_ring.len() as u16);
        body_invariant!(buffs_in_use.len() == _orig_buffs_in_use_len + i as usize);
        // body_invariant!(i > 0 ==> curr_desc == (_prev_curr_desc + 1) % desc_ring.len() as u16);
        // body_invariant!(i > 0 ==> desc_ring[_prev_curr_desc as usize].packet_address() == pktbuff_addr(&buffs_in_use.index(_orig_buffs_in_use_len + (i as usize - 1))).value() as u64);

        next_desc = update_desc(curr_desc, desc_ring.len() as u16);
        if next_desc == tx_clean {
            break;
        }

        if let Some(packet) = buffers.pop() {
            let desc = index_mut(desc_ring, curr_desc as usize);

            let rs_bit = if (curr_desc % TX_CLEAN_THRESHOLD) == TX_CLEAN_THRESHOLD - 1 { rs_bit } else { ReportStatusBit::zero() };
            desc.send(packet.paddr, packet.length, rs_bit);
            buffs_in_use.push(packet);

            _prev_curr_desc = curr_desc;
            curr_desc = next_desc;
            sent_pkts += 1;
        } else {
            break;
        }
        i += 1;
    }
    *curr_desc_stored = curr_desc; // should put if condition in case sent_pkts = 0, but make sure there's no overhead
    (sent_pkts, TDTUpdate(curr_desc) )
}


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


#[ensures(result.is_ok() ==> {
    forall(|i: usize| i < 128 ==> {
        if i == peek_result(&result) {
            filters[i].is_some() && peek_option(&filters[i]) == new_filter
        } else {
            filters[i] == old(filters[i]) && (filters[i].is_some() ==> !peek_option(&filters[i]).parameters_equal(&new_filter))
        }
    } )
})]
#[ensures(result.is_err() ==> {
    match peek_err(&result) {
        FilterError::NoneAvailable => forall(|i: usize|( i < 128 ==> filters[i].is_some())),
        FilterError::IdenticalFilter(idx) => filters[idx].is_some() && peek_option(&filters[idx]).parameters_equal(&new_filter),
    } && forall(|i: usize|( i < 128 ==> filters[i] == old(filters[i])))
})]
pub(crate) fn check_and_add_filter(filters: &mut [Option<FilterParameters>; 128], new_filter: FilterParameters) -> Result<usize, FilterError> {
    let mut i = 0;
    let mut unused_filter = None ;

    while i < 128 {
        body_invariant!(i < 128);
        body_invariant!(unused_filter.is_some() ==> peek_option(&unused_filter) < filters.len());
        body_invariant!(forall( |x: usize| x < i ==> {filters[x].is_some() ==> !peek_option(&filters[x]).parameters_equal(&new_filter)}));
        body_invariant!(unused_filter.is_none() ==> forall( |x: usize| x < i ==> filters[x].is_some()));

        if filters[i].is_some() {
            if filters[i].unwrap().parameters_equal(&new_filter) {
                return Err(FilterError::IdenticalFilter(i));
            }
        } else if unused_filter.is_none(){
            unused_filter = Some(i);
        }
        i += 1;
    }
    if unused_filter.is_some() {
        let filter_idx = unused_filter.unwrap();
        filters[filter_idx] = Some(new_filter);
        Ok(filter_idx)
    } else {
        Err(FilterError::NoneAvailable)
    }
}
