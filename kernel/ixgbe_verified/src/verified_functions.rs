use crate::{QueueID, rx_queue::{RxQueueE}};
use alloc::vec::Vec;

pub(crate) fn extract_rss_queues(reta: &[[QueueID; 4]; 32], enabled_queues: &mut Vec<RxQueueE>) -> Result<Vec<RxQueueE>, &'static str> {
    let mut used_queue_ids = Vec::new();
    for reg in reta {
        for i in 0..4 {
            if !used_queue_ids.contains(&reg[i]) {
                used_queue_ids.push(reg[i]);
            }
        }
    }

    let mut queues_for_rss = Vec::with_capacity(used_queue_ids.len());
    for qid in used_queue_ids {
        let index = enabled_queues.iter().position(|x| x.id == qid as u8)
            .ok_or("Required Queue for RSS is not in the enabled list")?;
        queues_for_rss.push(enabled_queues.remove(index));
    }

    Ok(queues_for_rss)
}