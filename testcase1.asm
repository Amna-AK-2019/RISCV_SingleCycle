###***RISC TEST CASE 1***###
nop
addi x2, x0, 5         # x2 = 5            
addi x3, x0, 12        # x3 = 12           
addi x4, x0, 0         # x4 = 0            
addi x5, x0, 20        # x5 = 20           
addi x6, x0, 30        # x6 = 30           
addi x7, x3, -9        # x7 = (12 - 9) = 3 

or x8, x2, x5          # x8 = 5 | 20 = 21  
and x9, x6, x5         # x9 = 30 & 20 = 20 
xor x10, x9, x7        # x10 = 20 ^ 3 = 23 
sll x11, x2, x3        # x11 = 5 << 12     
srl x12, x6, x2        # x12 = 30 >> 5 = 0 
sra x13, x6, x2        # x13 = 30 >> 5 (arithmetic)

slt x14, x3, x5        # x14 = (12 < 20) = 1 
beq x14, x0, skip1     # Branch if x14 == 0 (not taken) 
addi x15, x0, 7        # x15 = 7             
skip1: addi x15, x15, 1# x15 = x15 + 1 = 8   

sw x5, 100(x0)         # MEM[100] = 20       
lw x16, 100(x0)        # x16 = MEM[100] = 20
sw x6, 104(x0)         # MEM[104] = 30       
lw x17, 104(x0)        # x17 = MEM[104] = 30 

add x18, x2, x5        # x18 = 5 + 20 = 25   
sub x19, x6, x2        # x19 = 30 - 5 = 25   
bne x18, x19, skip2    # Branch if x18 != x19 (not taken) 
addi x20, x0, 50       # x20 = 50            
skip2: sub x21, x20, x5 # x21 = 50 - 20 = 30 

jal x1, target        
addi x22, x0, 99       
target: addi x23, x0, 42 # x23 = 42          
jalr x24, x1, 0        

