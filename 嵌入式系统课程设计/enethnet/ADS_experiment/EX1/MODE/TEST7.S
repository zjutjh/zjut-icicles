USR_STACK_LEGTH			EQU		64					; Define the stack length of different modes 
SVC_STACK_LEGTH			EQU		0
FIQ_STACK_LEGTH			EQU		16
IRQ_STACK_LEGTH			EQU		64
ABT_STACK_LEGTH			EQU		0
UND_STACK_LEGTH			EQU		0

			AREA	Example7,CODE,READONLY			; Pseudo-instruction,code snippet name
			ENTRY									; Program entry point
			CODE32									; 32-bit ARM instructions
START		

			MOV		R0,#0							; 0->R0
			MOV		R1,#1							; 1->R1
			MOV		R2,#2							; 2->R2
			MOV		R3,#3							; 3->R3
			MOV		R4,#4							; 4->R4
			MOV		R5,#5							; 5->R5
			MOV		R6,#6							; 6->R6
			MOV		R7,#7							; 7->R7
			MOV		R8,#8							; 8->R8
			MOV		R9,#9							; 9->R9
			MOV		R10,#10							; 10->R10
			MOV		R11,#11							; 11->R11
			MOV		R12,#12							; 12->R12
			
			BL		InitStack						; Jump to InitStack, and copy the address of the next instruction to LR 
			
			MRS		R0,CPSR							; CPSR->R0
			BIC		R0,R0,#0x80						; R0 = R0 & #01111111
			MSR		CPSR_cxsf,R0					; R0->CPSR_cxsf
			
			MSR		CPSR_c,#0xd0					; #0xd0->CPSR_c
			MRS		R0,CPSR							; CPSR->R0
			
			MSR		CPSR_c,#0xdf					; #0xdf->CPSR_c
			MRS		R0,CPSR							; CPSR->R0
			
HALT		B		HALT							; Jump in place

InitStack	
			MOV			R0,LR						; LR->R0
			MSR		CPSR_c,#0xd3					; #0xd3(10011 svc)->CPSR_c
			LDR		SP,StackSvc						; Load StackSvc to SP
			
			MSR		CPSR_c,#0xd2					; #0xd2(10010 irq)->CPSR_c
			LDR		SP,StackIrq						; Load StackIrq to SP		
			
			MSR		CPSR_c,#0xd1					; #0xd1(10001 fiq)->CPSR_c
			LDR		SP,StackFiq						; Load StackFiq to SP
			
			MSR		CPSR_c,#0xd7					; #0xd7(10111 abt)->CPSR_c
			LDR		SP,StackAbt						; Load StackAbt to SP
			
			MSR		CPSR_c,#0xd7					; #0xd7(11011 und)->CPSR_c
			LDR		SP,StackUnd						; Load StackUnd to SP
			
			MSR		CPSR_c,#0xd0					; #0xd(10000)->CPSR_c
			LDR		SP,StackUsr						; Load StackUsr to SP
			MOV			PC,R0						; R0->PC
			
StackUsr DCD	UsrStackSpace+(USR_STACK_LEGTH)*4	; Allocate a word memory unit(&)
StackSvc DCD	SvcStackSpace+(SVC_STACK_LEGTH)*4
StackIrq DCD	IrqStackSpace+(IRQ_STACK_LEGTH)*4
StackFiq DCD	FiqStackSpace+(FIQ_STACK_LEGTH)*4
StackAbt DCD	AbtStackSpace+(ABT_STACK_LEGTH)*4
StackUnd DCD	UndStackSpace+(UND_STACK_LEGTH)*4

			AREA	MyStACK,DATA,NOINIT,ALIGN=2		; Pseudo-instruction,code snippet name
			
UsrStackSpace		SPACE	USR_STACK_LEGTH			; Allocate a memory unit and initialize it with 0(%)
SvcStackSpace		SPACE	SVC_STACK_LEGTH
IrqStackSpace		SPACE	IRQ_STACK_LEGTH
FiqStackSpace		SPACE	FIQ_STACK_LEGTH
AbtStackSpace		SPACE	ABT_STACK_LEGTH
UndStackSpace		SPACE	UND_STACK_LEGTH

			END										; End of the source program
