# RISC-V Single Cycle Processor
This repository implements a single-cycle RISC-V processor based on the RV32I instruction set architecture (ISA). Designed for simplicity and clarity, it serves as an excellent resource for learning and understanding the basics of RISC-V processor design and architecture.
The processor supports all instruction types defined in the RV32I ISA:
* R-type
* I-type
* S-type
* B-type
* U-type
* J-type

## Features
* Supports the RV32I base integer instruction set.
* Implements a single-cycle datapath and control logic.
* Written in basic SystemVerilog.
* Simulated and tested using industry-standard tools.
* Tested with two different assembly programs.

## Project Highlights
This project provides a single-cycle schematic to help users understand the architecture easily.
### Datapath Components:
* Program Counter (PC)
* Instruction Memory
* Register File
* ALU
* Data Memory
### Control Unit:
* Instruction Decode
* Control Signal Generation
* Branch Comparator

## Additional Details:
1. Implements arithmetic, logical, memory access, and branch instructions.
2. Modular structure, allowing for easy modification and scalability.

## Tools
Venus: Used to convert RISC-V assembly code into machine code.
Vivado: Used for writing, simulating, and testing the SystemVerilog design.

## Additional Resources:
Register File Initialization: The RegFile_Data.mem file contains dummy values, allowing users to observe how the code overwrites register values during execution.
Data Memory Initialization: The memory_init.mem file contains sample data for testing memory-related instructions.

## How to Use
### Generate Machine Code:
1. Run the provided assembly files (###*testcase1.asm and testcase2.asm* ) in the Venus simulator.
2. Generate the machine code and paste it into the instruction memory file (InstMem_Data.mem).
### Run Simulation:
1. Simulate the design in Vivado.
2. Compare the output with the expected results provided as comments in the .asm files.

All RTL modules are present in one file.

top_module (RISCV_SingleCycle) ---> all the conenctions are made here <br>
       |___ program_counter <br>
       |___PC_adder <br>
       |___muxPC <br>
       |___Instruction_Memory <br>
       |___instruction_decoder <br>
       |___Register_File <br>
       |___Immediate_Gen <br>
       |___mux_control1 <br>
       |___mux_control2 <br>
       |___alu_control <br>
       |___control_unit <br>
       |___alu <br>
       |___Data_Memory <br>
       |___Mux_control_DMEM <br>
       |___branch_comparator <br>

Memory File <br>
       |___InstMem_Data.mem <br> 
       |___RegFile_Data.mem <br>
       |___memory_init.mem <br>

TestBench File: <br>
       |___tb_top_module <br>
