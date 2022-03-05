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
        AND R5, R3, R4    	@ R5 = 12 AND 7 = 4
        MOV R3, #8         @ R3 = 8
        MOV R10, #9        @ R10 = 9
        ADD R10, R3, R10    @ R10 = 17
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
END:    STR R10, [R0, #100] 	@ mem[100] = 17

/*
    Loop on a counter and display the 8 LSB on the 8 LED of the DE0-Nano
    The counting frequency is fixed by the RaspberryPI via the SPI

    Counter = LED at the address 0x800
    Counting frequency at the address 0x400
*/
        STR R2, [R0, #0x200] 	@ R = 7
        STR R0, [R0, #0x800]    @ Clear the LED
LOOP:   LDR R1, [R0, #0x400]    @ R1 = counting frequency
DELAY:  SUBS R1, R1, #1         @ R1 = R1 - 1
        BNE DELAY               @ Branch if R1 != 0
        LDR R2, [R0, #0x800]    @ R2 = led_reg
        ADD R2, R2, #1          @ R2 = R2 + 1
        STR R2, [R0, #0x800]    @ led_reg = R2
        B   LOOP                @ infinite loop
