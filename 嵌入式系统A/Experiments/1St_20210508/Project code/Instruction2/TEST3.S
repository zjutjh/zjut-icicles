X		EQU			11						; Define a name X=11
Y		EQU 		8						; Define a name Y=8
BIT23	EQU			(1<<23)					; Define a name BIT23=1<<23

		AREA		Example3,CODE,READONLY 	; Pseudo-instruction,code snippet name 
		ENTRY								; Program entry point
		CODE32								; 32-bit ARM instructions

START
		MOV			R0,#X					; X(there is 11)->R0
		MOV			R1,#Y					; Y(there is 8)->R1
		ADD			R3,R0,R1				; R3=R0+R1
		MOV			R8,R3					; R3->R8
		
		MVN			R0,#0xA0000007			; Store the inversion result of #0xA00000007 in R0 
		SUB			R5,R0,R8,LSL#2			; R5=R0-R8<<2
		
		MOV			R0,#Y					; Y(there is 8)->R0
		ADD			R0,R0,R0,LSL#2			; R0=R0+R0<<2=R0*5
		MOV			R0,R0,LSR#1				; R0>>1 -> R0
		MOV			R1,#X					; X(there is 11)->R1
		MOV			R1,R1,LSL#1				; R1<<1 -> R1
		CMP			R0,R1					; R0 is compared with R1, set NZCV bit 
		LDRHI		R2,=0xFFFF0000			; If R0>R1, then R2 load 0xFFFF0000R2
		ANDHI		R5,R5,R2				; If R0>R1, then R5=R5&R2
		ORRLS		R5,R5,#0x000000FF		; If R0<=R1(else),then R5=R5|#0x000000FF
		
		TST			R5,#BIT23				; R0 and R1 Bitwise logical AND operation, set NZCV bit
		BICNE		R5,R5,#0x00000040		; If z=0,then Save the logical AND result of R5 and #x00000040 inverse code to R5
		B			START					; Jump to the START label
		END									; End of the source program
