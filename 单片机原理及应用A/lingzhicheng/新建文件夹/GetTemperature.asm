        ORG 0000H
        LJMP MAIN
        ORG 000BH
        LJMP IT0P
        /*P3.6 DCLK
          P3.5 /CS
          P3.7 MISO
          P3.4 MOSI
          P2.0-2.2  3-8decoder
          P0   display data
        */
MAIN:   MOV SP,#60H
        MOV PSW,#00H
        MOV TMOD,#01H
        MOV TL0,#00H
        MOV TH0,#00H    //use max delay
        SETB ET0
        SETB EA
        SETB TR0
        MOV DPTR,#TAB 
        MOV R7,#4
        AJMP DISPLAY
//---------------Dynamic Display Part-----------------
DISPLAY:        MOV A,R2
                MOV P2,A
                MOV A,R3 
                MOVC A,@A+DPTR
                MOV P0,A 
                INC R3      //data address +1
                LCALL DELAY
                INC R2      //select address+1
                DJNZ R7,DISPLAY
                LJMP LOOP 
LOOP:   MOV R4,#0F8H //3-8decoder
        MOV R3,#40H  //display start data
DELAY:  MOV R5,#5       //CALL 1MS DELAY
D1:     MOV R6,#100
        DJNZ R6,$
        DJNZ R5,D1
        RET

//--------------------------------IT0 Part-----------
IT0P:   PUSH ACC
        PUSH PSW
        PUSH DPL
        PUSH DPH
        MOV TL0,#00H
        MOV TH0,#00H
        MOV PSW,#01H
        ACALL READ_AD_DATA
        POP DPH
        POP DPL
        POP PSW
        POP ACC
        RET
//--------------------------------SPI Part---------------
//--------------------------------spi write
SPI_WRITE:  MOV A,#0D4H //register data to choose temperature-sensor
            CLR P3.6
            MOV R0,#08H //send register data
            ACALL LOOP1
            RET
LOOP1:   DJNZ R0,SEND
         RET

SEND:   RRC A            //send the lowest bit
        MOV P3.4,C	 // There may be a bug!!!!!!
        CLR P3.6
        NOP
        SETB P3.6
        RET
//-----------------------------SPI READ
SPI_READ:   CLR A
            CLR P3.6
            MOV DPH,#00H//CLEAR
	    MOV DPL,#00H
            MOV R0,#04H
            ACALL LOOP2
            MOV DPH,A
            CLR A
            MOV R0,#08H
            MOV DPL,A   //move 12bits to DPTR
            RET
LOOP2:  DJNZ R0,RECEIVE
        RET
RECEIVE:    RL A
            SETB P3.6
            NOP
            CLR P3.6
            MOV 50H,P3
            ANL 50H,#01H        //only choose P3.7
            ORL A,50H    //receive bit from P3.7
            RET    
//-------------------READ AD DATA
READ_AD_DATA:   CLR P3.6
                CLR P3.5        //set '/CS' = 0,which means select this chip
                ACALL SPI_WRITE //order temperature-sensor to transform
                ACALL DELAY     //wait sensor transform,use dynamic display dalay directly
                CLR P3.6
                NOP
                SETB P3.6
                NOP
                CLR P3.6
                NOP
                NOP             //ready to read
                ACALL SPI_READ  //read data
                SETB P3.5
                ACALL HEX2BCD
                RET        
//--------------HEX To BCD Part-------------------------
HEX2BCD:        MOV R1,#0
                MOV R2,#0
                MOV R7,#16      //loop for 16 times
                ACALL BYTS4D
                MOV 41H,R1
                MOV 43H,R2
                MOV A,41H
                ANL 41H,#0FH
//----------------------------------100
                SWAP A
                ANL A,#0FH
                MOV 40H,A
//-----------------------------------1000
                MOV A,43H
                ANL 43H,#0FH
//-----------------------------------1
                SWAP A
                ANL A,#0FH
                MOV 42H,A
//-----------------------------------10
                RET
BYTS4D:         MOV A,DPL        //lowest bit -> A
                RLC A            //left move
                MOV DPL,A
                MOV A,DPH
                RLC A
                MOV DPH,A
                MOV A,R2 
                ADDC A,R2
                DA A 
                MOV R2,A
                MOV A,R1
                ADDC A,R1
                DA A
                MOV R1,A 
                DJNZ R7,BYTS4D  // finish 16bits? 
                RET
				
				
/*TAB:     //HIGH
 DB 0F9H,0A4H,0B0H,99H,92H,82H,0F8H,80H,90H,0C0H
 DB 0FFH    */
TAB:     //LOW
        DB 3FH,06H,5BH,4FH,66H,6DH,7DH,07H,7FH,6FH,77H,7CH,39H,5EH,79H,71H,73H,3EH,6EH,76H,38H,40H
        DB 00H 
END
