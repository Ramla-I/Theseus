use prusti_contracts::*;
use alloc::collections::VecDeque;

pub struct VecDequeWrapper<T>( pub VecDeque<T> );

impl<T> VecDequeWrapper<T> {

    #[trusted]
    #[ensures(result.len() == 0)]
    pub fn new() -> Self {
        VecDequeWrapper( VecDeque::new() )
    }

    #[trusted]
    #[ensures(result.len() == 0)]
    pub fn with_capacity(capacity: usize) -> Self {
        VecDequeWrapper( VecDeque::with_capacity(capacity) )
    }

    #[trusted]
    #[pure]
    pub fn len(&self) -> usize {
        self.0.len()
    }

    #[trusted]
    #[pure]
    #[requires(index < self.len())]
    pub fn index(&self, index: usize) -> &T {
        &self.0[index]
    }

    #[trusted]
    #[ensures(self.len() == old(self.len()) + 1)]
    #[ensures(forall (|i: usize| i < old(self.len()) ==> {
        self.index(i) === old(self.index(i))
    }))]
    #[ensures({
        let idx = self.len() - 1;
        *self.index(idx) === value
    })]
    pub fn push_back(&mut self, value: T) {
        self.0.push_back(value);
    }


    // #[trusted]
    // #[requires(0 <= index && index < self.len())]
    // #[after_expiry(self.len() == old(self.len()))]
    // pub fn index_mut(&mut self, index: usize) -> &mut T {
    //     &mut self.v[index]
    // }
}

// impl VecWrapper<PacketBufferS> {
//     /// Ideally this should be a generic function, but I get the error
//     /// "[Prusti: invalid specification] use of impure function "std::cmp::PartialEq::eq" in pure code is not allowed"
//     /// even though I implemented PartialEq for PacketBufferS and declared it as a pure fn
//     #[trusted]
//     #[requires(0 <= index && index < self.len())]
//     #[after_expiry(
//         self.len() == old(self.len()) &&
//         self.index(index).phys_addr.value() == before_expiry(result).phys_addr.value() &&
//         forall(
//             |i: usize| (0 <= i && i < self.len() && i != index) ==>
//             self.index(i).phys_addr.value() == old(self.index(i).phys_addr.value())
//         )
//     )]
//     pub fn index_mut(&mut self, index: usize) -> &mut PacketBufferS {
//         &mut self.v[index]
//     }

//     #[trusted]
//     #[ensures(self.len() == old(self.len()) + 1)]
//     #[ensures(forall (|i: usize| 0 <= i && i < old(self.len()) ==> {
//         self.index(i).phys_addr.value() == old(self.index(i)).phys_addr.value()
//     }))]
//     #[after_expiry({
//         let idx = self.len() - 1;
//         self.index(idx).phys_addr.value() == value.phys_addr.value()
//     })]
//     pub fn push(&mut self, value: PacketBufferS) {
//         self.v.push(value);
//     }

//     #[trusted]
//     #[ensures(result.is_some() ==> self.len() == old(self.len()) - 1)]
//     #[ensures(result.is_none() ==> self.len() == old(self.len()))]
//     #[ensures(forall (|i: usize| 0 <= i && i < self.len() ==> {
//         self.index(i).phys_addr.value() == old(self.index(i)).phys_addr.value()
//     }))]
//     #[ensures(result.is_some() ==> peek_option_ref(&result).phys_addr.value() == old(self.index(self.len() - 1)).phys_addr.value())]
//     pub fn pop(&mut self) -> Option<PacketBufferS> {
//         self.v.pop()
//     }
// }