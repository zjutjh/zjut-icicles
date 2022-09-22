led   EQU   P0.0;

	  ORG   0000H
	  LJMP  MAIN

	  ORG	0050H
MAIN:
	  MOV	SP,#60H
LOOP: 
	  SETB	led
	  LCALL DELAY1S
	  CLR	led
	  LCALL DELAY1S
	  SJMP	LOOP

DELAY1S:
	  MOV	R7,#0A7H
DL1:
	  MOV	R6,#0ABH
DL0:
	  MOV	R5,#10H
	  DJNZ	R5,$
	  DJNZ	R6,DL0
	  DJNZ	R7,DL1
	  NOP
	  RET

	  END