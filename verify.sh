#!/bin/bash

LIB_CRATES=(range_inclusive port_io)
KERNEL_CRATES=(kernel_config memory_structs prusti_external_spec prusti_representation_creator pci prusti_frame_chunk prusti_page_chunk memory)

cd libs/
for crate_name in "${LIB_CRATES[@]}"
do
    echo "using Prusti to verify: $crate_name"
    cd $crate_name
    ../../../prusti-release-v-2023-08-22/cargo-prusti 
    cd ..
done

cd ../kernel/

for crate_name in "${KERNEL_CRATES[@]}"
do
    echo "using Prusti to verify: $crate_name"
    cd $crate_name
    ../../../prusti-release-v-2023-08-22/cargo-prusti 
    cd ..
done
