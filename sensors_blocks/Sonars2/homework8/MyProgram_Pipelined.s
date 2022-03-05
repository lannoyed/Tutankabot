/************* CODE SECTION *************/
@Name Here
.text   @ the following is executable assembly

@ Ensure code section is 4-byte aligned:
.balign 4

@ main is the entry point and must be global
.global main

B main          @ begin at main
.balign 128

/************* MAIN SECTION *************/

/*
   Test ARM processor
   ADD, SUB, AND, ORR, LDR, STR, B
   If successful, it should write the value 7 to address 100
*/

main:   SUB R0, R15, R15 	@ R0 = 0
        ADD R2, R0, #5      @ R2 = 5
        ADD R3, R0, #12    	@ R3 = 12
        SUB R7, R3, #9    	@ R7 = 3
        ORR R4, R7, R2    	@ R4 = 3 OR 5 = 7
        MOV R5, #4      	@ R5 = 12 AND 7 = 4
        ADD R5, R5, R4    	@ R5 = 4 + 7 = 11
        SUBS R8, R5, R7    	@ R8 <= 11 - 3 = 8, set Flags
        BEQ END        		@ shouldn't be taken
        SUBS R8, R3, R4    	@ R8 = 12 - 7  = 5
        BGE AROUND       	@ should be taken
        ADD R5, R0, #0     	@ should be skipped
AROUND: SUBS R8, R7, R2   	@ R8 = 3 - 5 = -2, set Flags
        ADDLT R7, R5, #1  	@ R7 = 11 + 1 = 12
        SUB R7, R7, R2    	@ R7 = 12 - 5 = 7
        STR R7, [R3, #84]  	@ mem[12+84] = 7
        LDR R2, [R0, #96]  	@ R2 = mem[96] = 7
        ADD R15, R15, R0	@ PC <- PC + 8 (skips next)
        ADD R2, R0, #14    	@ shouldn't happen
        B END             	@ always taken
        ADD R2, R0, #13   	@ shouldn't happen
        ADD R2, R0, #10		@ shouldn't happen
END:    STR R2, [R0, #0x64] @ mem[100] = 7   - 0x64 = 100

/*
    Loop on a counter and display the 8 LSB on the 8 LED of the DE0-Nano
    The counting frequency is fixed by the RaspberryPI via the SPI

    Address MAP : 0x0000 - 0x03FF : RAM (255 words of 32 bits)
                  0x0400 - 0x043F : SPI - 16 reg. of 32 bits
                  0x0500 :        : LED Reg

    SPI Registers : 0x0400 : SPI_Reg0 : Counting Frequency
                    0x0404 : SPI_Reg1
                    0x0408 : SPI_Reg2
                    0x040C : SPI_Reg3
                    0x0410 : SPI_Reg4
                    0x0414 : SPI_Reg5

    (the 16 MSB of the address are set to 0x0000 !)
*/
        STR R0, [R0, #0x0500]   @ Clear the LED
LOOP:   LDR R1, [R0, #0x0400]   @ R1 = counting frequency
DELAY:  SUBS R1, R1, #1         @ R1 = R1 - 1
        BNE DELAY               @ Branch if R1 != 0
        LDR R2, [R0, #0x0500]   @ R2 = led_reg
        ADD R2, R2, #1          @ R2 = R2 + 1
        STR R2, [R0, #0x0500]   @ led_reg = R2
/*
    Do SPI_Reg4 = (SPI_Reg1 + SPI_Reg2) ^2 + (SPI_Reg2 - SPI_Reg3) ^2
    Do SPI_Reg5 = (SPI_Reg1 + SPI_Reg2) ^2 - (SPI_Reg2 - SPI_Reg3) ^2
    SPI_Reg2 > SPI_Reg3
*/
        LDR R3, [R0, #0x0404]   @ R3 = SPI_Reg1 = 1
        LDR R4, [R0, #0x0408]   @ R4 = SPI_Reg2 = 3
        LDR R5, [R0, #0x040C]   @ R5 = SPI_Reg3 = 2
        
        ADD R10, R3, R4         @R10 = (SPI_Reg1 + SPI_Reg2) = 4
        B SQUARE1

ENDSQUARE1 :                   @R6 = (SPI_Reg1 + SPI_Reg2) ^2 = 16

        SUB R10, R4, R5         @R10 =(SPI_Reg2 - SPI_Reg3) = 1
        B SQUARE2
        
ENDSQUARE2 :                    @R7 = (SPI_Reg2 - SPI_Reg3)^2 = 1

        ADD R8, R6, R7          @R8 = (SPI_Reg1 + SPI_Reg2) ^2 + (SPI_Reg2 - SPI_Reg3) ^2 = 17
        SUB R9, R6, R7          @R9 = (SPI_Reg1 + SPI_Reg2) ^2 - (SPI_Reg2 - SPI_Reg3) ^2 = 15

        STR R8, [R0, #0x0410]   @ SPI_Reg4 = R8 = 17
        STR R9, [R0, #0x0414]   @ SPI_Reg5 = R9 = 15

        B   LOOP                @ infinite loop

  @Square utilise R10 et R11  
SQUARE1 :
    MOV R6, R10 
    MOV R11, R10
    FOR1 :
        SUBS R11, R11, #1
        BLE ENDSQUARE1
        ADD R6, R6, R10
        B FOR1

SQUARE2 :
    MOV R7, R10 
    MOV R11, R10
    FOR2 :
        SUBS R11, R11, #1
        BLE ENDSQUARE2
        ADD R7, R7, R10
        B FOR2             @END