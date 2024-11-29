`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/03/2024 10:20:21 AM
// Design Name: 
// Module Name: tb_top_module
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
//test bench

module tb_top_module;

    // Testbench signals
    logic clk;
    logic reset;

    // Instantiate the top module
    top_module top_module_inst (
        .clk(clk),
        .reset(reset)
    );

 // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns clock period
    end

   
    initial begin
        // Initialize inputs
        reset = 1;
        //$readmemb("RegFile_Data.mem", top_module_inst.Register_File_inst.Registers);
/////////////////////////////////////////////////////
       
        #10;
        reset = 0;

//////////////////////////////////////////////////////
        #10;
        //reset = 1;

        
   $finish;
 end
endmodule
