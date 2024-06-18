extern crate std;

use self::std::dbg;

use super::*;

struct TestRep {
    value: usize
}

#[derive(Clone, Copy, PartialEq, Debug)]
struct TestResId{
    value: usize
}

impl TestResId{
    fn create_rep(&self) -> TestRep {
        TestRep {
            value: self.value
        }
    }
}

impl ResourceIdentifier for TestResId{
    #[pure]
    fn overlaps(&self, other: &Self) -> bool{
        self.value == other.value
    }
}

#[test]
fn create_representations() {
    let mut creator = RepresentationCreator::new(TestResId::create_rep, true);
    assert!(creator.array.is_some());
    assert!(creator.list.len() == 0);

    for i in 0..creator.array.as_ref().unwrap().len() {
        assert!(creator.array.as_ref().unwrap().lookup(i).is_none());
    }

    for i in 0..creator.array.as_ref().unwrap().len() {
        match creator.create_unique_representation(TestResId{value: i}) {
            Ok((rep, idx)) => {
                assert!(rep.value == i);
                assert!(idx == i);
            },
            Err(_) => assert!(false)
        }

        match creator.create_unique_representation(TestResId{value: i}) {
            Ok(_) => assert!(false),
            Err(x) => { match x {
                RepresentationCreationError::Overlap(idx) => assert!(idx == i),
                _ => assert!(false)
            
            }}
        }
    }

    match creator.create_unique_representation(TestResId{value: 64}) {
        Ok(_) => assert!(false),
        Err(x) => { match x {
            RepresentationCreationError::NoSpace => assert!(true),
            _ => assert!(false)
        
        }}
    }

    assert!(creator.switch_to_heap_allocated().is_ok());
    assert!(creator.array.is_none());

    assert!(creator.create_unique_representation(TestResId{value: 64}).is_ok());
    assert!(creator.create_unique_representation(TestResId{value: 64}).is_err());

    dbg!(creator.array);

    for i in 0.. creator.list.len() {
        dbg!(creator.list.lookup(i));
    }
}