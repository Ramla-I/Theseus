#!/bin/bash

LIB_CRATES=(
    range_inclusive 
    port_io
)

KERNEL_CRATES=(
    prusti_external_spec 
    kernel_config 
    prusti_representation_creator 
    prusti_frame_chunk 
    prusti_page_chunk 
    memory_structs 
    memory 
    prusti_memory_buffer 
    prusti_borrowed_shared_mp
    pci 
    ixgbe_flexible
)

cd libs/
for crate_name in "${LIB_CRATES[@]}"
do
    echo "USING PRUSTI TO VERIFY: $crate_name"
    cd $crate_name
    ../../../prusti-release-v-2023-08-22/cargo-prusti 
    cd ..
    sleep 5
done

cd ../kernel/

for crate_name in "${KERNEL_CRATES[@]}"
do
    echo "USING PRUSTI TO VERIFY: $crate_name"
    cd $crate_name
    ../../../prusti-release-v-2023-08-22/cargo-prusti 
    cd ..
    sleep 5
done
