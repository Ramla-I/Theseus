#[verified]
#[requires(desc_ring.len() > 0)]
#[requires((*curr_desc_stored as usize) < desc_ring.len())]
#[requires((*curr_desc_stored as usize) < buffs_in_use.len())]
#[requires(desc_ring.len() == buffs_in_use.len())]
#[requires(desc_ring.len() < U16_MAX)]
#[requires(buffs_in_use.len() < U16_MAX)]
#[requires(batch_size < 4)]
#[ensures(desc_ring.len() == old(desc_ring.len()))]
#[ensures(result.0 <= batch_size)]
#[ensures(*curr_desc_stored < desc_ring.len() as u16)]
#[ensures(*curr_desc_stored == calc_descriptor_rec(old(*curr_desc_stored), result.0, desc_ring.len() as u16))]
fn receive(
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
        body_invariant!(rcvd_pkts == i);

        body_invariant!(desc_ring.len() == buffs_in_use.len());
        body_invariant!(desc_ring.len() == _orig_desc_ring_len && buffs_in_use.len() == _orig_buffs_in_use_len);
        body_invariant!(curr_desc < desc_ring.len() as u16 && curr_desc < buffs_in_use.len() as u16);
        
        body_invariant!((curr_desc == prev_curr_desc) || curr_desc == (prev_curr_desc + 1) % desc_ring.len() as u16);
        body_invariant!(i > 0 ==> desc_ring[prev_curr_desc as usize].packet_address() == value(pktbuff_addr(&buffs_in_use[prev_curr_desc as usize])) as u64);
        body_invariant!(curr_desc == calc_descriptor_rec(*curr_desc_stored, i, desc_ring.len() as u16));
        // body_invariant!(forall (|j: u16| j < i ==> {
        //     let desc = calc_descriptor_rec(*curr_desc_stored, j, desc_ring.len() as u16);
        //     desc < desc_ring.len() as u16
        //     && desc < buffs_in_use.len() as u16
        //     && desc_ring[desc as usize].packet_address() == value(pktbuff_addr(&buffs_in_use[desc as usize])) as u64 // this line is taking forever
        // }));

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
#[requires((*curr_desc_stored as usize) < buffs_in_use.len())]
#[requires(desc_ring.len() == buffs_in_use.len())]
#[requires(desc_ring.len() < U16_MAX)]
#[requires(buffs_in_use.len() < U16_MAX)]
#[requires(batch_size < 4)]
#[requires(desc_list.len() as u16 == batch_size)]
#[ensures(desc_ring.len() == old(desc_ring.len()))]
#[ensures(result.0 <= batch_size)]
#[ensures(*curr_desc_stored < desc_ring.len() as u16)]
#[ensures(*curr_desc_stored == calc_descriptor_rec(old(*curr_desc_stored), result.0, desc_ring.len() as u16))]
// #[ensures(forall (|i: u16| i < result.0 ==> {
//     let curr_desc = calc_descriptor_rec(old(*curr_desc_stored), i, desc_ring.len() as u16);
//     (curr_desc as usize) < desc_ring.len() 
    //&& (curr_desc as usize) < buffs_in_use.len()
    // && desc_ring[curr_desc as usize].packet_address() == value(pktbuff_addr(&buffs_in_use[curr_desc as usize])) as u64
    // true
// }))]
fn receive(
    curr_desc_stored: &mut u16, 
    desc_ring: &mut [AdvancedRxDescriptor],
    buffs_in_use: &mut [PktBuff], 
    buffers: &mut VecWrapper<PktBuff>,
    mempool: &mut VecDequeWrapper<PktBuff>,
    batch_size: u16,
    desc_list: &mut [u16]
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
        body_invariant!(rcvd_pkts == i);

        body_invariant!(desc_ring.len() == buffs_in_use.len());
        body_invariant!(desc_ring.len() == _orig_desc_ring_len && buffs_in_use.len() == _orig_buffs_in_use_len);
        body_invariant!(curr_desc < desc_ring.len() as u16 && curr_desc < buffs_in_use.len() as u16);
        
        body_invariant!((curr_desc == prev_curr_desc) || curr_desc == (prev_curr_desc + 1) % desc_ring.len() as u16);
        body_invariant!(i > 0 ==> desc_ring[prev_curr_desc as usize].packet_address() == value(pktbuff_addr(&buffs_in_use[prev_curr_desc as usize])) as u64);
        body_invariant!(curr_desc == calc_descriptor_rec(*curr_desc_stored, i, desc_ring.len() as u16));
        body_invariant!(desc_list.len() as u16 == batch_size);
        body_invariant!(forall (|j: u16| j < i ==> {
            let desc = desc_list[j as usize];
            j < desc_list.len() as u16 && desc < desc_ring.len() as u16
            && desc_ring[desc as usize].packet_address() == value(pktbuff_addr(&buffs_in_use[desc as usize])) as u64
        }));

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
            *index_mut(desc_list, i as usize) = curr_desc;
            prev_curr_desc = curr_desc;
            curr_desc = update_desc(curr_desc, desc_ring.len() as u16);
        } else {
            break;
        }

        i += 1;
    }

    *curr_desc_stored = curr_desc; // should put if condition in case rcvd_pkts = 0, but make sure there's no overhead
    (rcvd_pkts, RDTUpdate(prev_curr_desc))
}