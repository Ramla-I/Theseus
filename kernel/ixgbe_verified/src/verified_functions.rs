use prusti_contracts::*;
cfg_if::cfg_if! {
if #[cfg(prusti)] {
    use crate::spec::memory_spec::PacketBufferS;
} else {
    use packet_buffers::PacketBufferS;
}}

#[allow(unused_imports)]
use crate::{
    spec::result_spec::*,
    vec_wrapper::VecWrapper, 
    hal::descriptors::*,
    queue_registers::*,
};

#[requires(0 <= *rx_cur_stored && (*rx_cur_stored as usize) < rx_descs.len())]
#[requires(0 <= *rx_cur_stored && (*rx_cur_stored as usize) < rx_bufs_in_use.len())]
#[requires(rx_descs.len() > 0)]
#[requires((num_rx_descs as usize) == rx_descs.len())]
#[requires(rx_bufs_in_use.len() == rx_descs.len())]
#[ensures((result.is_ok() && peek_result(&result) != 0) ==> (old(*rx_cur_stored) + peek_result(&result)) % num_rx_descs == *rx_cur_stored)]
#[ensures((result.is_ok() && peek_result(&result) == 0) ==> old(*rx_cur_stored) == *rx_cur_stored)]
#[ensures(result.is_ok()  ==> buffers.len() == old(buffers.len()) + peek_result(&result) as usize)]
#[ensures(result.is_ok()  ==> rx_bufs_in_use.len() == old(rx_bufs_in_use.len()))]
#[after_expiry(result.is_ok() ==> forall (|i: usize| 0<= i && i < peek_result(&result) as usize ==> {
    let rx_cur = (old(*rx_cur_stored) + i as u16) % num_rx_descs;
    let old_buffer_len = old(buffers.len());
    buffers.index(old_buffer_len + i).phys_addr.value() == old(rx_bufs_in_use.index(rx_cur as usize)).phys_addr.value()
}))]
#[after_expiry(result.is_ok() ==> forall (|i: usize| 0<= i && i < peek_result(&result) as usize ==> {
    let rx_cur = (old(*rx_cur_stored) + i as u16) % num_rx_descs;
    let old_buffer_len = old(pool.len());
    pool.index(old_buffer_len - i).phys_addr.value() == rx_bufs_in_use.index(rx_cur as usize).phys_addr.value()
}))]
pub fn rx_batch(
    rx_descs: &mut [AdvancedRxDescriptor], 
    rx_cur_stored: &mut u16, 
    rx_bufs_in_use: &mut VecWrapper<PacketBufferS>,
    regs: &mut RxQueueRegisters,
    num_rx_descs: u16,
    buffers: &mut VecWrapper<PacketBufferS>, 
    batch_size: usize, 
    pool: &mut VecWrapper<PacketBufferS>
) -> Result<u16, ()> {
    let mut rx_cur = *rx_cur_stored;
    let mut last_rx_cur = *rx_cur_stored;

    // have to add this in for verification because the verifier can't reason that taking the remainder after each increment, 
    // or taking the remainder after all increments is equivalent
    let mut rx_cur_total = *rx_cur_stored; 
    
    let mut rcvd_pkts = 0;
    let mut i = 0;
    let _buffers_len = buffers.len();

    while i < batch_size {
        body_invariant!(num_rx_descs as usize == rx_descs.len());
        body_invariant!(rx_cur  < num_rx_descs);
        body_invariant!(rx_bufs_in_use.len() == rx_descs.len());
        body_invariant!(rcvd_pkts as usize == i);
        body_invariant!((rx_cur == last_rx_cur) || rx_cur == (last_rx_cur + 1) % num_rx_descs);
        body_invariant!(*rx_cur_stored + rcvd_pkts == rx_cur_total);
        // body_invariant!((rx_cur == rx_cur_total) || (rx_cur_total % num_rx_descs == rx_cur));
        body_invariant!(buffers.len() == _buffers_len + rcvd_pkts as usize);

        let desc = index_mut(rx_descs, rx_cur as usize);

        if !desc.descriptor_done() {
            break;
        }

        if !desc.end_of_packet() {
            // return Err("Currently do not support multi-descriptor packets");
            return Err(());
        }

        let length = desc.length();

        // Now that we are "removing" the current receive buffer from the list of receive buffers that the NIC can use,
        // (because we're saving it for higher layers to use),
        // we need to obtain a new `ReceiveBuffer` and set it up such that the NIC will use it for future receivals.
        if let Some(new_receive_buf) = pool.pop() {
            // actually tell the NIC about the new receive buffer, and that it's ready for use now
            desc.set_packet_address(new_receive_buf.phys_addr());
            desc.reset_status();
            
            let mut current_rx_buf = replace(rx_bufs_in_use.index_mut(rx_cur as usize), new_receive_buf);
            current_rx_buf.length = length as u16; // set the ReceiveBuffer's length to the size of the actual packet received
            buffers.push(current_rx_buf);

            rcvd_pkts += 1;
            rx_cur_total += 1;
            last_rx_cur = rx_cur;
            rx_cur = (rx_cur + 1) % num_rx_descs;
            // prusti_assert!((last_rx_cur + rcvd_pkts) % rx_descs.len() == rx_cur)
        } else {
            // return Err("Ran out of packet buffers");
            return Err(());
        }
        i += 1;
    }

    prusti_assert!(rx_cur_total == *rx_cur_stored + rcvd_pkts);
    // if last_rx_cur != rx_cur {
        // *rx_cur_stored = rx_cur as u16;
    if rcvd_pkts != 0 {
        *rx_cur_stored = rx_cur_total % num_rx_descs;
        regs.rdt_write(last_rx_cur); 
    }


    Ok(rcvd_pkts)
}


#[trusted]
#[requires(0 <= index && index < s.len())]
#[after_expiry(s.len() == old(s.len()))]
fn index_mut<T>(s: &mut [T], index: usize) -> &mut T {
    &mut s[index]
}

#[trusted]
#[ensures(old(dest).phys_addr.value() == result.phys_addr.value())]
#[after_expiry(dest.phys_addr.value() == src.phys_addr.value())]
fn replace(dest: &mut PacketBufferS, src: PacketBufferS) -> PacketBufferS{
    core::mem::replace(dest, src)
}


#[requires(tx_descs.len() > 0)]
#[requires((num_tx_descs as usize) == tx_descs.len())]
#[requires(0 <= *tx_cur_stored && *tx_cur_stored < num_tx_descs)]
#[ensures(result.is_ok() ==> (old(*tx_cur_stored) + peek_result(&result).0) % num_tx_descs == *tx_cur_stored)]
#[ensures(result.is_ok()  ==> buffers.len() == old(buffers.len()) - peek_result(&result).0 as usize)]
#[ensures(result.is_ok()  ==> tx_bufs_in_use.len() == old(tx_bufs_in_use.len()) - peek_result(&result).1 + peek_result(&result).0 as usize)]
#[ensures(result.is_ok()  ==> used_buffers.len() == old(used_buffers.len()) + peek_result(&result).1 )]
#[after_expiry(result.is_ok() ==> forall (|i: usize| 0<= i && i < peek_result(&result).0 as usize ==> {
    let pkts_removed = peek_result(&result).1;
    let tx_bufs_length_old = old(tx_bufs_in_use.len()) - pkts_removed;
    old(buffers.index(buffers.len() - i)).phys_addr.value() == tx_bufs_in_use.index(tx_bufs_length_old + i).phys_addr.value()
}))]
pub(crate) fn tx_batch(
    tx_descs: &mut [AdvancedTxDescriptor], 
    tx_bufs_in_use: &mut VecWrapper<PacketBufferS>,
    num_tx_descs: u16,
    tx_clean_stored: &mut u16,
    tx_cur_stored: &mut u16,
    regs: &mut TxQueueRegisters,
    batch_size: usize,  
    buffers: &mut VecWrapper<PacketBufferS>, 
    used_buffers: &mut VecWrapper<PacketBufferS>,
    rs_bit: u8
) -> Result<(u16, usize), &'static str> {
    let pkts_removed = tx_clean(tx_descs, tx_bufs_in_use, tx_clean_stored, tx_cur_stored, num_tx_descs, used_buffers);
    
    let mut pkts_sent = 0;
    let tx_clean = *tx_clean_stored;
    let mut tx_cur = *tx_cur_stored;

    // have to add this in for verification because the verifier can't reason that taking the remainder after each increment, 
    // or taking the remainder after all increments is equivalent
    let mut tx_cur_total = *tx_cur_stored; 


    // debug!("tx_cur = {}, tx_clean ={}", tx_cur, tx_clean);
    
    let _buffers_len = buffers.len();
    let _buffers_in_use_len = tx_bufs_in_use.len();

    let mut i = 0;
    while i < batch_size {
        body_invariant!(num_tx_descs as usize == tx_descs.len());
        body_invariant!(tx_cur < num_tx_descs);
        body_invariant!(pkts_sent as usize == i);
        body_invariant!(*tx_cur_stored + pkts_sent == tx_cur_total);
        // // body_invariant!((rx_cur == rx_cur_total) || (rx_cur_total % num_rx_descs == rx_cur));
        body_invariant!(buffers.len() == _buffers_len - pkts_sent as usize);
        body_invariant!(tx_bufs_in_use.len() == _buffers_in_use_len + pkts_sent as usize);
        // body_invariant!(tx_cur == (*tx_cur_stored + pkts_sent) % num_tx_descs);

        if let Some(packet) = buffers.pop() {
            let tx_next = (tx_cur + 1) % num_tx_descs;

            if tx_clean == tx_next {
                // tx queue of device is full, push packet back onto the
                // queue of to-be-sent packets
                buffers.push(packet);
                break;
            }

            index_mut(tx_descs, tx_cur as usize).send(packet.phys_addr(), packet.length, rs_bit);
            tx_bufs_in_use.push(packet);

            tx_cur = tx_next;
            pkts_sent += 1;
            tx_cur_total += 1;

        } else {
            break;
        }
        i += 1;
    }


    // *tx_cur_stored = tx_cur;
    *tx_cur_stored = tx_cur_total % num_tx_descs;
    regs.tdt_write(tx_cur);

    Ok((pkts_sent, pkts_removed))
}

 /// Removes multiples of `TX_CLEAN_BATCH` packets from `queue`.    
/// (code taken from https://github.com/ixy-languages/ixy.rs/blob/master/src/ixgbe.rs#L1016)
#[trusted]
#[ensures(old(tx_bufs_in_use.len()) == tx_bufs_in_use.len() + result)]
#[ensures(old(used_buffers.len()) + result == used_buffers.len())]
#[after_expiry( forall (|i: usize| 0 <= i && i < result as usize ==> {
    let old_used_buffer_len = old(used_buffers.len());
    used_buffers.index(old_used_buffer_len + i).phys_addr.value() == old(tx_bufs_in_use.index(i)).phys_addr.value()
}))]
fn tx_clean(    
    tx_descs: &[AdvancedTxDescriptor], 
    tx_bufs_in_use: &mut VecWrapper<PacketBufferS>,
    tx_clean_stored: &mut u16, 
    tx_cur_stored: &u16, 
    num_tx_descs: u16, 
    used_buffers: &mut VecWrapper<PacketBufferS>
)  -> usize {
    const TX_CLEAN_BATCH: usize = 32;

    let mut tx_clean = *tx_clean_stored as usize;
    let tx_cur = *tx_cur_stored;
    let mut pkts_removed = 0;

    loop {
        let mut cleanable = tx_cur as i32 - tx_clean as i32;

        if cleanable < 0 {
            cleanable += num_tx_descs as i32;
        }

        if cleanable < TX_CLEAN_BATCH as i32 {
            break;
        }

        let mut cleanup_to = tx_clean + TX_CLEAN_BATCH - 1;

        if cleanup_to >= num_tx_descs as usize {
            cleanup_to -= num_tx_descs as usize;
        }

        if tx_descs[cleanup_to].desc_done() {
            if TX_CLEAN_BATCH >= tx_bufs_in_use.len() {
                used_buffers.v.extend(tx_bufs_in_use.v.drain(..));
                pkts_removed += tx_bufs_in_use.len();
            } else {
                used_buffers.v.extend(tx_bufs_in_use.v.drain(..TX_CLEAN_BATCH));
                pkts_removed += TX_CLEAN_BATCH;
            };

            tx_clean = (cleanup_to + 1) % num_tx_descs as usize;
        } else {
            break;
        }
    }

    *tx_clean_stored = tx_clean as u16;
    pkts_removed
}
