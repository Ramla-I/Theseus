# Combining Formal and Informal Techniques for Lightweight Correctness Guarantees in Systems Software
This is the Theseus source code with the changes made for our hybrid approach.
Here we list the crates that we have added or made changes to with a description of the changes:

- range_inclusive: external specifications
- port_io: external specifications
- prusti_external_spec: external specifications 
- kernel_config: external specifications 
- prusti_representation_creator: the RepCreator code
- prusti_frame_chunk: changes for the frame allocator, we verify the FrameChunk type and then add it as a field to Frames 
- prusti_page_chunk: changes for the page allocator, we verify the PageChunk type and then add it as a field to Pages
- memory_structs: verify FrameRange and PageRange types and add specification
- memory: cast functions and compiler-checked specification
- prusti_memory_buffer: cast functions
- prusti_borrowed_shared_mp: cast functions
- pci: to make PCIDevice a representation 
- ixgbe_flexible: full-featured flexible version of the driver to compare with DPDK
- ixgbe_restricted: restricted version of the driver to compare with TinyNF

The test applications are:
- mm_eval
- bm: run with the memory_map option
- packet_forwarder_flexible
- packet_forwarder_restricted

The measurement scripts are in the verification_and_measurement_scripts folder. The paths to Prusti are hardcoded so will need to be updated if you run them.
- loc_dafny.sh: measure LOC for a dafny file
- spec_cc.sh: measure compiler-checked specification in a file
- spec_dafny.sh: measure lines of Dafny specification 
- spec_prusti.sh: measure lines of Prusti specification
- verify.py: verification timing measurements
