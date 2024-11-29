###***RISCV TEST CASE2***###
nop
addi x1, x0, 5          
addi x2, x0, 10         
addi x3, x0, 15         

addi x10, x0, 0         

add x1, x2, x3          
sub x3, x1, x2          
add x2, x2, x2          
sub x2, x2, x3          

add x10, x10, x1       

addi x5, x0, 1000       

sw x2, 0(x5)           
lw x6, 0(x5)           

sw x3, 4(x5)            
lw x6, 4(x5)           

add x10, x10, x6        

sw x2, 0(x5)            
lw x7, 0(x5)           

addi x7, x7, 1          
addi x6, x7, 1          
beq x6, x7, label_skip  
addi x6, x6, 2         

sw x6, 8(x5)            
lw x2, 8(x5)           

label_skip:
addi x1, x0, 5          
bne x1, x2, label_end   

addi x2, x2, -2         

label_end:
addi x3, x3, -1         
beq x3, x0, done        

lw x6, 4(x5)            
sw x6, 12(x5)          

add x10, x10, x6       

add x0, x0, x1          

done:
