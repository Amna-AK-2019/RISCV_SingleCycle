`timescale 1ns / 1ps
////working properly RISCV

////`timescale 1ns / 1ps

//designing the PC
module program_counter(
  input logic clk,               // Clock signal
  input logic reset,             // Reset signal
  input logic [31:0] pc_next,
  output logic [31:0] pc// Program Counter Out
  //input logic [31:0] pc_in // Program Counter In 
);

  // Program Counter Register
  always_ff @(posedge clk or negedge reset) begin
    if (reset) 
      pc <= 32'd0;  // Reset the PC to zero
    else 
      pc <= pc_next;  // Increment the PC on each clock cycle
  end

endmodule


//--------------------------------------------------------------------------------------------------------------

//This module is for adding 4 in the PC instructions and for jumps
module PC_adder (
	input logic [31:0] a,b,
	output logic [31:0] c
);

	assign c = a+b;
	
endmodule

//--------------------------------------------------------------------------------------------------------------

module muxPC(
	input logic [31:0] input1,input2,
	output logic [31:0] muxPC_out,
	input logic muxPC_Sel
	
);

assign muxPC_out = (muxPC_Sel) ? input2 : input1;

endmodule


//--------------------------------------------------------------------------------------------------------------

//designing the Instruction memory
module Instruction_Memory (
  input logic [31:0] Read_Address,  // Address input (byte-addressed)
  output logic [31:0] Instruction_Out  // Output instruction at the address
);

logic [31:0] Memory [0:40];  // Memory with 21 instructions (adjust size as needed)

// Initialize the memory with instructions from a binary file
initial begin
  $readmemh("InstMem_Data.mem", Memory);
end

// Read the instruction from memory (convert byte-address to word index)
assign Instruction_Out = Memory[Read_Address >> 2];

endmodule

//--------------------------------------------------------------------------------------------------------------

module instruction_decoder(
    input logic [31:0] instruction,  // 32-bit instruction input
    output logic [4:0] rs1,          // 5-bit source register 1
    output logic [4:0] rs2,          // 5-bit source register 2
    output logic [4:0] rd,           // 5-bit destination register
    output logic [2:0] funct3,       // 3-bit function code (funct3)
    output logic [6:0] funct7,        // 7-bit function code (funct7)
    output logic [6:0] Opcode        //Opcode for the selection of the types
);

    assign Opcode = instruction [6:0]; //first 7 bits are for the opcode of isntruction type
    assign rs1   = instruction[19:15];  // Extract rs1 from bits [19:15]
    assign rs2   = instruction[24:20];  // Extract rs2 from bits [24:20]
    assign rd    = instruction[11:7];   // Extract rd from bits [11:7]
    assign funct3 = instruction[14:12]; // Extract funct3 from bits [14:12]
    assign funct7 = instruction[31:25]; // Extract funct7 from bits [31:25]
    
endmodule
//--------------------------------------------------------------------------------------------------------------

module Register_File( 
	input logic reset,clk,RegWrite,   //single bit input
	input logic [4:0] Rs1,
	input logic [4:0] Rs2,
	input logic [4:0]  Rd,  
	
	input logic [31:0]  Write_data,  //32 bits
	
	output logic [31:0] Read_Data1,Read_Data2
	);
	
	logic  [31:0] Registers [31:0];  //32 registers each of 32 bits wide
	
//   Initialize the register file from a binary file
  initial begin 
    $readmemb("RegFile_Data.mem", Registers); 
  end
	
	assign Read_Data1 = Registers[Rs1];
	assign Read_Data2 = Registers[Rs2];
	
    always_ff @(posedge clk) begin
      if (reset == 1'b1) begin
 //       for (integer i = 0; i <32; i++) begin 
            Registers[Rd] <= 32'd0;
 //       end
       end
      else if (RegWrite == 1'b1 && Rd != 0) begin
        Registers[Rd] <= Write_data;
        //Registers[4] <= 32'hdeadbeef;
      end
    end


endmodule

//--------------------------------------------------------------------------------------------------------------
module Immediate_Gen(
    input logic [31:0] Imme_input,  // 32-bit input
    input logic [2:0] ImmSel,       // Selection between I-type, S-type, or J-type immediate
    output logic [31:0] Imme_output // 32-bit extended output
);

logic [11:0] imm_ISbits;   // For I-type and S-type
logic [19:0] imm_Jbits;   // For J-type
logic [12:0] imm_Btype;    // 13-bit B-type immediate
logic [19:0] imm_Utype;    // 20 btis for the U type

always_comb begin
    case (ImmSel)
        3'b000: begin
            // I-Type: Extract the 12-bit immediate value from bits [31:20]
            imm_ISbits = Imme_input[31:20];

            // Perform sign extension if the MSB (bit 11) is 1
            if (imm_ISbits[11] == 1'b1) 
                Imme_output = {20'b11111111111111111111, imm_ISbits}; // Sign extension
            else
                Imme_output = {20'b00000000000000000000, imm_ISbits}; // Zero extension
        end

        3'b001: begin
            // S-Type: Extract the 12-bit immediate value from bits [31:25] and [11:7]
            imm_ISbits = {Imme_input[31:25], Imme_input[11:7]};

            // Perform sign extension if the MSB (bit 11) is 1
            if (imm_ISbits[11] == 1'b1)
                Imme_output = {20'b11111111111111111111, imm_ISbits}; // Sign extension
            else
                Imme_output = {20'b00000000000000000000, imm_ISbits}; // Zero extension
        end

        3'b010: begin
            // J-Type: Extract the 20-bit immediate value from [31], [30:21], [20], [19:12]
            imm_Jbits = {Imme_input[31], Imme_input[19:12], Imme_input[20], Imme_input[30:21]};

            // Perform sign extension if the MSB (bit 19) is 1
            if (imm_Jbits[19] == 1'b1)
                Imme_output = {12'b111111111111, imm_Jbits}; // Sign extension
            else
                Imme_output = {12'b000000000000, imm_Jbits}; // Zero extension
        end
		
		 3'b011: begin
            // B-Type: Extract the bits immediate value from [12|10:5], [4:1|11]
            imm_Btype = {Imme_input[31],          // imm[12]
                            Imme_input[7],           // imm[11]
                            Imme_input[30:25],       // imm[10:5]
                            Imme_input[11:8],        // imm[4:1]
                            1'b0};                    // imm[0] (always 0)

            // Perform sign extension if the MSB (bit 19) is 1
            if (imm_Btype[12] == 1'b1)
                Imme_output = {19'b1111111111111111111, imm_Btype}; // Sign extension
            else
                Imme_output = {19'b0000000000000000000, imm_Btype}; // Zero extension
        end
		
		3'b100: begin
            // U-Type: Extract the bits immediate value from [31:12]
            imm_Utype =   Imme_input[31:12] ;             // imm[0] (always 0)

            // Perform sign extension if the MSB (bit 19) is 1
            if (Imme_input[31] == 1'b1)
                Imme_output = {20'b1111111111111111111, imm_Utype}; // Sign extension
            else
                Imme_output = {20'b0000000000000000000, imm_Utype}; // Zero extension
        end
		

        default: Imme_output = 32'b0;  // Default case if none is selected
    endcase
end

endmodule

//--------------------------------------------------------------------------------------------------------------

module mux_control1(
    input logic [31:0] Read_Data2_input,
    input logic BSel,
    input logic [31:0] Immediate_input,
    
    output logic [31:0] BSel_muxout
);    
    assign BSel_muxout = (BSel) ? Immediate_input : Read_Data2_input;
    
endmodule

//--------------------------------------------------------------------------------------------------------------

module mux_control2(
    input logic [31:0] Read_Data1_input,
    input logic BSel2,
    input logic [31:0] pc_input_mux2,
    
    output logic [31:0] BSel2_muxout
);    
    assign BSel2_muxout = (BSel2) ? pc_input_mux2 : Read_Data1_input;
    
endmodule


//--------------------------------------------------------------------------------------------------------------

module alu_control(
  input logic [6:0] funct7,
  input logic [2:0] funct3,
  input logic [6:0] Opcode,
  output logic [3:0] alu_sel,
  output logic [3:0] branch_sel
);

typedef enum logic [6:0] {
    Unknown_Type = 7'bzzzzzzz,
    R_Type = 7'b0110011,
    I_Type = 7'b0010011,
    Load_Type = 7'b0000011,
    S_Type = 7'b0100011,
    J_Type = 7'b1101111,
    JALR_Itype = 7'b1100111,
    B_Type = 7'b1100011,
    U_Type = 7'b0110111,
    U_AUIPC_Type = 7'b0010111
} ISA_Type;

ISA_Type instruction_type;

always_comb begin
  // Default values
  alu_sel = 4'bxxxx; // Unknown by default
  branch_sel = 4'b0000; // No branching by default

  // Determine instruction type based on Opcode
  case (Opcode)
    R_Type: instruction_type = R_Type;
    I_Type: instruction_type = I_Type;
    Load_Type: instruction_type = Load_Type;
    S_Type: instruction_type = S_Type;
    J_Type: instruction_type = J_Type;
    JALR_Itype: instruction_type = JALR_Itype;
    B_Type: instruction_type = B_Type;
    U_Type: instruction_type = U_Type;
    U_AUIPC_Type: instruction_type = U_AUIPC_Type;
    default: instruction_type = Unknown_Type;
  endcase

  // Handle ALU and branch selection based on instruction type and funct fields
  case (instruction_type)
    R_Type: begin
      case (funct3)
        3'b000: alu_sel = (funct7[5] == 1'b0) ? 4'd0 : 4'd1; // ADD/SUB
        3'b001: alu_sel = 4'd5; // SLL
        3'b010: alu_sel = 4'd8; // SLT
        3'b011: alu_sel = 4'd9; // SLTU
        3'b100: alu_sel = 4'd4; // XOR
        3'b111: alu_sel = 4'd2; // AND
        3'b110: alu_sel = 4'd3; // OR
        3'b101: alu_sel = (funct7[5] == 1'b0) ? 4'd6 : 4'd7; // SRL/SRA
        default: alu_sel = 4'bxxxx; // Unknown operation
      endcase
    end
    
    I_Type: begin
      case (funct3)
        3'b000: alu_sel = 4'd0; // ADDI
        3'b001: alu_sel = 4'd5; // SLLI
        3'b010: alu_sel = 4'd8; // SLTI
        3'b011: alu_sel = 4'd9; // SLTIU
        3'b100: alu_sel = 4'd4; // XORI
        3'b101: alu_sel = (funct7[5] == 1'b0) ? 4'd6 : 4'd7; // SRLI/SRAI
        3'b110: alu_sel = 4'd3; // ORI
        3'b111: alu_sel = 4'd2; // ANDI
        default: alu_sel = 4'bxxxx; // Unknown operation
      endcase
    end
    
    Load_Type: begin
      alu_sel = 4'd0; // Use ADD for load, store, jump, etc.
    end
	
	S_Type:begin
      alu_sel = 4'd0; // Use ADD for load, store, jump, etc.
    end
	
	J_Type:begin
      alu_sel = 4'd0; // Use ADD for load, store, jump, etc.
    end
	
	JALR_Itype: begin
      alu_sel = 4'd0; // Use ADD for load, store, jump, etc.
    end
    
    B_Type: begin
      case (funct3)
        3'b000: begin
            branch_sel = 4'b0000;  // BEQ (Branch if Equal)
            alu_sel = 4'd0;        // ALU operation add for rs1+rs2
         end
            
        3'b001:begin
            branch_sel = 4'b0001; // BNE
            alu_sel = 4'd0;        // ALU operation add for rs1+rs2
         end
         
        3'b100:begin 
            branch_sel = 4'b0010; // BLT
            alu_sel = 4'd0;        // ALU operation add for rs1+rs2
         end
        3'b101:
            begin
            branch_sel = 4'b0011; // BGE
            alu_sel = 4'd0;        // ALU operation add for rs1+rs2
         end
        3'b110:
            begin
            branch_sel = 4'b0100; // BLTU
            alu_sel = 4'd0;        // ALU operation add for rs1+rs2
         end
        default: branch_sel = 4'bxxxx; // Unknown branch operation
      endcase
    end
    
    U_Type: begin
      alu_sel = 4'd10; // Immediate output (LUI)
    end
    
    U_AUIPC_Type: begin
      alu_sel = 4'd0; // AUIPC uses ADD
    end
  endcase
end

endmodule

//--------------------------------------------------------------------------------------------------------------

module control_unit(
  input logic [6:0] Opcode,
  input logic [2:0] funct3,
  output logic RegWrite,
  output logic BSel,BSel2,
  output logic load,
  output logic store,
  output logic [2:0] ImmSel,
  output logic [1:0]DMEM_Sel,
  output logic muxPC_Sel,
  output logic [2:0] data_size_sel,
  input logic branch_taken
);
  always_comb begin
    // Default values
    RegWrite = 1'b0;
    BSel = 1'b0;
    load = 1'b0;
    store = 1'b0;
	ImmSel=3'b000;
    DMEM_Sel=2'b00;
	muxPC_Sel=1'b0;
    
    case (Opcode)
      7'b0110011: begin  // R-type
        RegWrite = 1'b1;
        BSel = 1'b0;  
        BSel2= 1'b0; 
        DMEM_Sel = 2'b01;
      end
      
      7'b0010011: begin  // I-type
        RegWrite = 1'b1;
        BSel = 1'b1;
        BSel2= 1'b0; 
		ImmSel=3'b000;
		DMEM_Sel = 2'b01;  
//		load=1'bz;
//		store=1'bz;
//		muxPC_Sel=1'bz;  
		  
      end
     
		7'b0000011: begin  // Load instructions
        RegWrite = 1'b1;
        BSel = 1'b1;
        load = 1'b1;
        //store = 1'b0;
        DMEM_Sel = 2'b00;
        case (funct3)
          3'b000: data_size_sel = 3'b000; // Load byte
          3'b001: data_size_sel = 3'b001; // Load halfword
          3'b010: data_size_sel = 3'b010; // Load word
          3'b011: data_size_sel = 3'b011; // byte load unsigned
          3'b100: data_size_sel = 3'b100; // halfword load usigned
          default: data_size_sel = 3'b000; // Default: byte load
        endcase
		end
      
		7'b0100011: begin  // Store instructions
        RegWrite = 1'b0;
        BSel = 1'b1;
        BSel2 = 1'b0;
        store = 1'b1;
        //load=1'b0;
        ImmSel=3'b001;
        DMEM_Sel = 2'b01;
        case (funct3)
          3'b000: data_size_sel = 3'b000; // Load byte
          3'b001: data_size_sel = 3'b001; // Load halfword
          3'b010: data_size_sel = 3'b010; // Load word
          default: data_size_sel =3'b000; // Default: byte store
        endcase
		end
	   
	  
	    7'b1101111: begin  // Jump instruction
        BSel = 1'b1;
		BSel2 = 1'b1; // this selction is for the 2nd mux used in between the register nad the ALU operan 1
		ImmSel=3'b010; //for store type immediate
		muxPC_Sel = 1'b1;  //PC update based on jump
		RegWrite = 1'b1;
		store = 1'b1;
		load = 1'b0;
		DMEM_Sel=2'b10;		
		
      end
      
      7'b1100111:
      begin
      //control signal for the JALR as it is the I type instructions
        RegWrite = 1'b1;
        BSel = 1'b1;  // Use immediate for operand2
		ImmSel=3'b000;  //for I type immediate
        muxPC_Sel=1'b1;
        BSel2 = 1'b0;
        DMEM_Sel=2'b10;
        
      end
      
      7'b1100011: begin  // B-type Branch instructions
        BSel = 1'b1;  // Use immediate for operand2
        BSel2 = 1'b1;  // Use branch comparison logic for mux
        ImmSel = 3'b011;  // B-type immediate
        RegWrite = 1'b0;
		
	  if (branch_taken == 1'b1) begin
          muxPC_Sel = 1'b1;  // Branch taken, update PC
        end else begin
          muxPC_Sel = 1'b0;  // No branch, continue normal execution
        end
      end
	  
	  7'b0110111:    //U type instruction
      begin
      //control signal for the JALR as it is the I type instructions
        RegWrite = 1'b1;
        BSel = 1'b1;  // Use immediate for operand2
        BSel2 = 1'bx;
		ImmSel=3'b100;  //for U type immediate
        muxPC_Sel=1'b0;
        DMEM_Sel=2'b00;
		load = 1'b1;

      end
      
      7'b0010111:    //AUIPC type instruction
      begin
      //control signal for the JALR as it is the I type instructions
        RegWrite = 1'b1;
        BSel = 1'b1;  // Use immediate for operand2
        BSel2=1'b1;
		ImmSel=3'b100;  //for U type immediate
        muxPC_Sel=1'b0;
        DMEM_Sel=2'b01;
		load = 1'b1;

      end
      default: begin // Default case for undefined opcodes
        RegWrite = 1'b0;
        BSel = 1'b0;
        BSel2 = 1'b0;
        load = 1'b0;
        store = 1'b0;
        ImmSel = 3'b000;
        DMEM_Sel = 2'b00;
        muxPC_Sel = 1'b0;
      end
    endcase
  end
endmodule

//--------------------------------------------------------------------------------------------------------------

module alu(
    input  logic [31:0] operand1,       // Operand 1 (from rs1)
    input  logic [31:0] operand2,       // Operand 2 (from rs2 or immediate)
    input  logic [3:0] alu_sel,         // ALU operation selection
    output logic [31:0] alu_result      // ALU result
);

    always_comb begin
        case (alu_sel)
            4'd0: alu_result = operand1 + operand2; // ADD / ADDI
            4'd1: alu_result = operand1 - operand2; // SUB
            4'd2: alu_result = operand1 & operand2; // AND / ANDI
            4'd3: alu_result = operand1 | operand2; // OR / ORI
            4'd4: alu_result = operand1 ^ operand2; // XOR / XORI
            4'd5: alu_result = operand1 << operand2[4:0]; // SLL / SLLI
            4'd6: alu_result = operand1 >> operand2[4:0]; // SRL / SRLI
            4'd7: alu_result = $signed(operand1) >>> operand2[4:0]; // SRA / SRAI
            4'd8: alu_result = ($signed(operand1) < $signed(operand2)) ? 1 : 0; // SLT / SLTI
            4'd9: alu_result = (operand1 < operand2) ? 1 : 0;  // SLTU / SLTIU
			4'd10: alu_result = operand2; //this operation is for the U type instruction
            default: alu_result = 32'd0; // Unknown operation
        endcase
    end

endmodule

//--------------------------------------------------------------------------------------------------------------

module Data_Memory(
  input logic clk,
  input logic load,       // New load signal
  input logic store,      // New store signal
  input logic [2:0] data_size_sel, // New data size signal (00: byte, 01: halfword, 10: word)
  input logic [31:0] DMEM_address,
  input logic [31:0] DMEM_data_in,
  output logic [31:0] DMEM_data_out
);

  // Memory array
  logic [7:0] Data_Mem [0:1023];  // Example 1KB memory
  
     // Load the memory with data from a .mem file
    initial begin
        $readmemh("memory_init.mem", Data_Mem);  // Initialize memory from a hex file
    end
//  initial begin 
//    for (longint i = 0; i< 1024; i=i+1)
//    Data_Mem[i] = 0;
//  end 

  always_ff @(posedge clk) begin
      if (store) begin
      case (data_size_sel)
        3'b000: Data_Mem[DMEM_address] <= DMEM_data_in[7:0];  // Byte store
        3'b001: begin
          Data_Mem[DMEM_address] <= DMEM_data_in[7:0];
          Data_Mem[DMEM_address+1] <= DMEM_data_in[15:8];  // Halfword store
        end
        3'b010: begin
          Data_Mem[DMEM_address] <= DMEM_data_in[7:0];
          Data_Mem[DMEM_address+1] <= DMEM_data_in[15:8];
          Data_Mem[DMEM_address+2] <= DMEM_data_in[23:16];
          Data_Mem[DMEM_address+3] <= DMEM_data_in[31:24];  // Word store
        end
        default: ;
      endcase
   end
 end
  always_comb begin
    if (load) begin
      case (data_size_sel)
        3'b000: DMEM_data_out = {{24{Data_Mem[DMEM_address][7]}}, Data_Mem[DMEM_address]};         // Byte load
        3'b001: DMEM_data_out = {{16{Data_Mem[DMEM_address+1][7]}}, Data_Mem[DMEM_address+1], Data_Mem[DMEM_address]};  // Halfword load
        3'b010: DMEM_data_out = {Data_Mem[DMEM_address+3], Data_Mem[DMEM_address+2], Data_Mem[DMEM_address+1], Data_Mem[DMEM_address]};  // Word load
        3'b011:DMEM_data_out = {24'b0, Data_Mem[DMEM_address]}; // Zero-extend
        3'b100:DMEM_data_out = {16'b0, Data_Mem[DMEM_address + 1], Data_Mem[DMEM_address]};
        default: 
        DMEM_data_out = 32'b0; // Default case to avoid latches
      endcase
    end else begin
        DMEM_data_out = 32'b0;
    end
  end
endmodule

//--------------------------------------------------------------------------------------------------------------
module Mux_control_DMEM(
    input logic [31:0] DMEM_input_zero,  //output of the ALU_Result will be the input of the mux 
    input logic [31:0] DMEM_input_one,  //output of the data memory will be the input of the mux 
    input logic [31:0] DMEM_input_two, //output of the pc adder will be the input of the DMEME mux
	output logic [31:0] DMEM_output,    //the output of the mux will be connected to the Reg file 
    input logic [1:0] DMEM_Sel             //for selection
    );

    // DMEM_Sel = 00 -> DMEM_input_zero
    // DMEM_Sel = 01 -> DMEM_input_one
	// DMEM_Sel = 10 -> DMEM_input_two
	
    assign DMEM_output = (DMEM_Sel == 2'b00) ? DMEM_input_zero : 
                         (DMEM_Sel == 2'b01) ? DMEM_input_one : 
                         (DMEM_Sel == 2'b10) ? DMEM_input_two : 32'b0;

endmodule

//--------------------------------------------------------------------------------------------------------------
module branch_comparator (
    input logic [31:0] B_input1,
    input logic [31:0] B_input2,
    input logic BrUn,
    input logic [3:0] branch_sel,
    
    output logic branch_taken
);
    always_comb begin
        // Initialize outputs
        branch_taken = 1'b0;
        
        case(branch_sel)
            4'b0000: begin
                // Equality check
                if (B_input1 == B_input2) begin
                    branch_taken = 1'b1;  // Set branch_taken to 1 if B_input1 == B_input2
                end
                else
                begin
                    branch_taken=1'b0;
                end
            end
            
            4'b0001: begin
                // Inequality check (for not equal)
                if (B_input1 != B_input2) begin
                    branch_taken = 1'b1;  // Set branch_taken to 1 if B_input1 != B_input2
                end
                else
                begin
                    branch_taken=1'b0;
                end
            end
            
            4'b0010: begin
                // Signed comparison: greater or equal (A >= B)
                if ($signed(B_input1) >= $signed(B_input2)) begin
                    branch_taken = 1'b1;  // Set branch_taken to 1 if B_input1 >= B_input2 (signed)
                end
                else
                begin
                    branch_taken=1'b0;
                end
            end
            
            4'b0011: begin
                // Signed comparison: less than (A < B)
                if ($signed(B_input1) < $signed(B_input2)) begin
                    branch_taken = 1'b1;  // Set branch_taken to 1 if B_input1 < B_input2 (signed)
                end
                else
                begin
                    branch_taken=1'b0;
                end
            end

            4'b0100: begin
                // Unsigned comparison (BrUn flag determines signed/unsigned)
                if (B_input1 < B_input2) begin
                        branch_taken = 1'b1;  // Set Lt to 1 if B_input1 < B_input2 (unsigned)
                    end
                    else
                begin
                    branch_taken=1'b0;
                end
            end

            default: begin
                branch_taken=1'b0;// Default case: No operation, branch_taken remains 0
            end
        endcase
    end
endmodule

//--------------------------------------------------------------------------------------------------------------

module RISCV_SingleCycle(
    input logic clk, 
    input logic reset
);

    logic [31:0] Instruction_Out_wire; 
    logic [4:0] rs1_wire, rs2_wire, rd_wire;
    logic [2:0] funct3_wire;
    logic [6:0] funct7_wire;
    logic [6:0] Opcode_wire;

    logic [31:0] Read_Data1_wire, Read_Data2_wire;
    logic RegWrite_wire, BSel;
    logic [2:0] ImmSel_wire;
    logic load_wire, store_wire;
    logic [1:0] DMEM_Sel_wire;

    logic [3:0] alu_sel_wire;
    logic [31:0] alu_result_wire, DMEM_to_Writedata_wire;
    logic [31:0] Imme_output_wire;
    logic [31:0] mux2alu_wire;
    
    logic [31:0] DMEM_input_zero_wire, BSel2_muxout_wire;
    logic BSel2;
    logic [31:0] pc_out, pcPlus4, muxPC_out_wire, pc_next;
    logic muxPC_Sel_wire, branch_taken;
    logic [3:0] branch_sel;
    logic [31:0] Write_data_wire;
	logic [2:0] data_size_sel;
    
    program_counter program_counter_inst (
        .clk(clk),
        .reset(reset),
        .pc(pc_out),
        .pc_next(pc_next)
    );

    PC_adder PC_adder_inst(
        .a(pc_out),
        .b(32'd4),
        .c(pcPlus4)
    );

    muxPC muxPC_inst(
        .input1(pcPlus4),
        .input2(alu_result_wire),
        .muxPC_out(muxPC_out_wire),
        .muxPC_Sel(muxPC_Sel_wire)
    );

    assign pc_next = muxPC_out_wire;

    Instruction_Memory Instruction_Memory_inst (
        .Read_Address(pc_out),
        .Instruction_Out(Instruction_Out_wire)
    );

    instruction_decoder instruction_decoder_inst (
        .instruction(Instruction_Out_wire),
        .rs1(rs1_wire),
        .rs2(rs2_wire),
        .rd(rd_wire),
        .funct3(funct3_wire),
        .funct7(funct7_wire),
        .Opcode(Opcode_wire)
    );

    Register_File Register_File_inst (
        .clk(clk),
        .reset(reset),
        .RegWrite(RegWrite_wire),
        .Rs1(rs1_wire),
        .Rs2(rs2_wire),
        .Rd(rd_wire),
        .Write_data(DMEM_to_Writedata_wire),
        .Read_Data1(Read_Data1_wire),
        .Read_Data2(Read_Data2_wire)
    );
    
    //assign Write_data=alu_result_wire;
    // Inline Multiplexer logic for Write_data
    
//assign Write_data_wire = DMEM_Sel_wire ? DMEM_to_Writedata_wire : alu_result_wire;

    Immediate_Gen Immediate_Gen_inst(
        .Imme_input(Instruction_Out_wire),
        .ImmSel(ImmSel_wire),
        .Imme_output(Imme_output_wire)
    );

    mux_control1 mux_control1_inst(
        .Read_Data2_input(Read_Data2_wire),
        .BSel(BSel),
        .Immediate_input(Imme_output_wire),
        .BSel_muxout(mux2alu_wire)
    );

    mux_control2 mux_control2_inst(
        .Read_Data1_input(Read_Data1_wire),
        .BSel2(BSel2),
        .pc_input_mux2(pc_out),
        .BSel2_muxout(BSel2_muxout_wire)
    );

    alu_control alu_control_inst (
        .funct7(funct7_wire),
        .funct3(funct3_wire),
        .alu_sel(alu_sel_wire),
        .Opcode(Opcode_wire),
        .branch_sel(branch_sel)
    );

    control_unit control_unit_inst(
        .Opcode(Opcode_wire),
		.funct3(funct3_wire),
        .RegWrite(RegWrite_wire),
        .BSel(BSel),
        .BSel2(BSel2),
        .load(load_wire),
        .store(store_wire),
        .ImmSel(ImmSel_wire),
        .DMEM_Sel(DMEM_Sel_wire),
		.data_size_sel(data_size_sel), // Updated: Add data_size_sel
        .muxPC_Sel(muxPC_Sel_wire),
        .branch_taken(branch_taken)
    );

    alu alu_inst (
        .operand1(BSel2_muxout_wire),
        .operand2(mux2alu_wire),
        .alu_sel(alu_sel_wire),
        .alu_result(alu_result_wire)
    );

    Data_Memory Data_Memory_inst(
        .clk(clk),
        .DMEM_address(alu_result_wire),
        .DMEM_data_in(Read_Data2_wire),  
        .load(load_wire),
        .store(store_wire),
		.data_size_sel(data_size_sel), // Updated: Pass data_size_sel
        .DMEM_data_out(DMEM_input_zero_wire)
    );

    Mux_control_DMEM Mux_control_DMEMinst(
        .DMEM_input_one(alu_result_wire),
        .DMEM_input_zero(DMEM_input_zero_wire),
        .DMEM_input_two(pcPlus4),
		.DMEM_Sel(DMEM_Sel_wire),
        .DMEM_output(DMEM_to_Writedata_wire)
    );

    branch_comparator branch_comparator_inst(
        .B_input1(Read_Data1_wire),
        .B_input2(Read_Data2_wire),
        .BrUn(1'b0),
        .branch_sel(branch_sel),
        .branch_taken(branch_taken)
    );

endmodule

