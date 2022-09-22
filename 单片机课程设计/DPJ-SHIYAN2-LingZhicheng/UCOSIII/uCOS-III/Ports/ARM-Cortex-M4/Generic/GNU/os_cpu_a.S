@
@********************************************************************************************************
@                                                uC/OS-III
@                                          The Real-Time Kernel
@
@
@                         (c) Copyright 2009-2015; Micrium, Inc.; Weston, FL
@                    All rights reserved.  Protected by international copyright laws.
@
@                                           ARM Cortex-M4 Port
@
@ File      : OS_CPU_A.ASM
@ Version   : V3.04.05
@ By        : JJL
@             BAN
@             JBL
@
@ For       : ARMv7 Cortex-M4
@ Mode      : Thumb-2 ISA
@ Toolchain : GNU C Compiler
@********************************************************************************************************
@

@********************************************************************************************************
@                                          PUBLIC FUNCTIONS
@********************************************************************************************************

    .extern  OSRunning                                           @ External references
    .extern  OSPrioCur
    .extern  OSPrioHighRdy
    .extern  OSTCBCurPtr
    .extern  OSTCBHighRdyPtr
    .extern  OSIntExit
    .extern  OSTaskSwHook
    .extern  OS_CPU_ExceptStkBase


    .global  OSStartHighRdy                                      @ Functions declared in this file
    .global  OSCtxSw
    .global  OSIntCtxSw
    .global  OS_CPU_PendSVHandler
    
    
#if (defined(__VFP_FP__) && !defined(__SOFTFP__))
    .global  OS_CPU_FP_Reg_Push
    .global  OS_CPU_FP_Reg_Pop
#endif


@********************************************************************************************************
@                                               EQUATES
@********************************************************************************************************

.equ NVIC_INT_CTRL,     0xE000ED04                              @ Interrupt control state register.
.equ NVIC_SYSPRI14,     0xE000ED22                              @ System priority register (priority 14).
.equ NVIC_PENDSV_PRI,   0xFF                                    @ PendSV priority value (lowest).
.equ NVIC_PENDSVSET,    0x10000000                              @ Value to trigger PendSV exception.


@********************************************************************************************************
@                                     CODE GENERATION DIRECTIVES
@********************************************************************************************************


   .text
   .align 2
   .thumb
   .syntax unified
   
   
@********************************************************************************************************
@                                   FLOATING POINT REGISTERS PUSH
@                             void  OS_CPU_FP_Reg_Push (CPU_STK  *stkPtr)
@
@ Note(s) : 1) This function saves S0-S31, and FPSCR registers of the Floating Point Unit.
@
@           2) Pseudo-code is:
@              a) Get FPSCR register value;
@              b) Push value on process stack;
@              c) Push remaining regs S0-S31 on process stack;
@              d) Update OSTCBCurPtr->StkPtr;
@********************************************************************************************************

#if (defined(__VFP_FP__) && !defined(__SOFTFP__))
OS_CPU_FP_Reg_Push:
    MRS     R1, PSP                                             @ PSP is process stack pointer
    CBZ     R1, OS_CPU_FP_nosave                                @ Skip FP register save the first time

    VMRS    R1, FPSCR
    STR R1, [R0, #-4]!
    VSTMDB  R0!, {S0-S31}
    LDR     R1, =OSTCBCurPtr
    LDR     R2, [R1]
    STR     R0, [R2]
OS_CPU_FP_nosave:
    BX      LR
#endif


@********************************************************************************************************
@                                   FLOATING POINT REGISTERS POP
@                             void  OS_CPU_FP_Reg_Pop (CPU_STK  *stkPtr)
@
@ Note(s) : 1) This function restores S0-S31, and FPSCR registers of the Floating Point Unit.
@
@           2) Pseudo-code is:
@              a) Restore regs S0-S31 of new process stack;
@              b) Restore FPSCR reg value
@              c) Update OSTCBHighRdyPtr->StkPtr pointer of new proces stack;
@********************************************************************************************************

#if (defined(__VFP_FP__) && !defined(__SOFTFP__))
OS_CPU_FP_Reg_Pop:
    VLDMIA  R0!, {S0-S31}
    LDMIA   R0!, {R1}
    VMSR    FPSCR, R1
    LDR     R1, =OSTCBHighRdyPtr
    LDR     R2, [R1]
    STR     R0, [R2]
    BX      LR
#endif


@********************************************************************************************************
@                                         START MULTITASKING
@                                      void OSStartHighRdy(void)
@
@ Note(s) : 1) This function triggers a PendSV exception (essentially, causes a context switch) to cause
@              the first task to start.
@
@           2) OSStartHighRdy() MUST:
@              a) Setup PendSV exception priority to lowest;
@              b) Set initial PSP to 0, to tell context switcher this is first run;
@              c) Set the main stack to OS_CPU_ExceptStkBase
@              d) Trigger PendSV exception;
@              e) Enable interrupts (tasks will run with interrupts enabled).
@********************************************************************************************************

.thumb_func
OSStartHighRdy:
    CPSID   I                                                   @ Prevent interruption during context switch
    MOVW    R0, #:lower16:NVIC_SYSPRI14                         @ Set the PendSV exception priority
    MOVT    R0, #:upper16:NVIC_SYSPRI14

    MOVW    R1, #:lower16:NVIC_PENDSV_PRI
    MOVT    R1, #:upper16:NVIC_PENDSV_PRI
    STRB    R1, [R0]

    MOV     R0, #0
    MSR     PSP, R0
    BL      OSTaskSwHook                                        @ Call OSTaskSwHook for FPU Pop

    MOVW    R0, #:lower16:OS_CPU_ExceptStkBase                  @ Initialize the MSP to the OS_CPU_ExceptStkBase
    MOVT    R0, #:upper16:OS_CPU_ExceptStkBase
    LDR     R1, [R0]
    MSR     MSP, R1

    MOVW    R0, #:lower16:OSPrioCur                             @ OSPrioCur   = OSPrioHighRdy;
    MOVT    R0, #:upper16:OSPrioCur
    MOVW    R1, #:lower16:OSPrioHighRdy
    MOVT    R1, #:upper16:OSPrioHighRdy
    LDRB    R2, [R1]
    STRB    R2, [R0]

    MOVW    R5, #:lower16:OSTCBCurPtr
    MOVT    R5, #:upper16:OSTCBCurPtr
    MOVW    R1, #:lower16:OSTCBHighRdyPtr                       @ OSTCBCurPtr = OSTCBHighRdyPtr;
    MOVT    R1, #:upper16:OSTCBHighRdyPtr
    LDR     R2, [R1]
    STR     R2, [R5]

    LDR     R0, [R2]                                            @ R0 is new process SP; SP = OSTCBHighRdyPtr->StkPtr;
    MSR     PSP, R0                                             @ Load PSP with new process SP

    MRS     R0, CONTROL
    ORR     R0, R0, #2
    MSR     CONTROL, R0
    ISB                                                         @ Sync instruction stream

    LDMFD    SP!, {R4-R11}                                      @ Restore r4-11 from new process stack
    LDMFD    SP!, {R0-R3}                                       @ Restore r0, r3
    LDMFD    SP!, {R12, LR}                                     @ Load R12 and LR
    LDMFD    SP!, {R1, R2}                                      @ Load PC and discard xPSR
    CPSIE    I
    BX       R1


@********************************************************************************************************
@                       PERFORM A CONTEXT SWITCH (From task level) - OSCtxSw()
@
@ Note(s) : 1) OSCtxSw() is called when OS wants to perform a task context switch.  This function
@              triggers the PendSV exception which is where the real work is done.
@********************************************************************************************************

.thumb_func
OSCtxSw:
    LDR     R0, =NVIC_INT_CTRL                                  @ Trigger the PendSV exception (causes context switch)
    LDR     R1, =NVIC_PENDSVSET
    STR     R1, [R0]
    BX      LR


@********************************************************************************************************
@                   PERFORM A CONTEXT SWITCH (From interrupt level) - OSIntCtxSw()
@
@ Note(s) : 1) OSIntCtxSw() is called by OSIntExit() when it determines a context switch is needed as
@              the result of an interrupt.  This function simply triggers a PendSV exception which will
@              be handled when there are no more interrupts active and interrupts are enabled.
@********************************************************************************************************

.thumb_func
OSIntCtxSw:
    LDR     R0, =NVIC_INT_CTRL                                  @ Trigger the PendSV exception (causes context switch)
    LDR     R1, =NVIC_PENDSVSET
    STR     R1, [R0]
    BX      LR


@********************************************************************************************************
@                                       HANDLE PendSV EXCEPTION
@                                   void OS_CPU_PendSVHandler(void)
@
@ Note(s) : 1) PendSV is used to cause a context switch.  This is a recommended method for performing
@              context switches with Cortex-M4.  This is because the Cortex-M4 auto-saves half of the
@              processor context on any exception, and restores same on return from exception.  So only
@              saving of R4-R11 is required and fixing up the stack pointers.  Using the PendSV exception
@              this way means that context saving and restoring is identical whether it is initiated from
@              a thread or occurs due to an interrupt or exception.
@
@           2) Pseudo-code is:
@              a) Get the process SP, if 0 then skip (goto d) the saving part (first context switch);
@              b) Save remaining regs r4-r11 on process stack;
@              c) Save the process SP in its TCB, OSTCBCurPtr->OSTCBStkPtr = SP;
@              d) Call OSTaskSwHook();
@              e) Get current high priority, OSPrioCur = OSPrioHighRdy;
@              f) Get current ready thread TCB, OSTCBCurPtr = OSTCBHighRdyPtr;
@              g) Get new process SP from TCB, SP = OSTCBHighRdyPtr->OSTCBStkPtr;
@              h) Restore R4-R11 from new process stack;
@              i) Perform exception return which will restore remaining context.
@
@           3) On entry into PendSV handler:
@              a) The following have been saved on the process stack (by processor):
@                 xPSR, PC, LR, R12, R0-R3
@              b) Processor mode is switched to Handler mode (from Thread mode)
@              c) Stack is Main stack (switched from Process stack)
@              d) OSTCBCurPtr      points to the OS_TCB of the task to suspend
@                 OSTCBHighRdyPtr  points to the OS_TCB of the task to resume
@
@           4) Since PendSV is set to lowest priority in the system (by OSStartHighRdy() above), we
@              know that it will only be run when no other exception or interrupt is active, and
@              therefore safe to assume that context being switched out was using the process stack (PSP).
@********************************************************************************************************

.thumb_func
OS_CPU_PendSVHandler:
    CPSID   I                                                   @ Prevent interruption during context switch
    MRS     R0, PSP                                             @ PSP is process stack pointer
    STMFD   R0!, {R4-R11}                                       @ Save remaining regs r4-11 on process stack

    MOVW    R5, #:lower16:OSTCBCurPtr                           @ OSTCBCurPtr->OSTCBStkPtr = SP;
    MOVT    R5, #:upper16:OSTCBCurPtr
    LDR     R6, [R5]
    STR     R0, [R6]                                            @ R0 is SP of process being switched out

                                                                @ At this point, entire context of process has been saved
    MOV     R4, LR                                              @ Save LR exc_return value
    BL      OSTaskSwHook                                        @ OSTaskSwHook();

    MOVW    R0, #:lower16:OSPrioCur                             @ OSPrioCur   = OSPrioHighRdy;
    MOVT    R0, #:upper16:OSPrioCur
    MOVW    R1, #:lower16:OSPrioHighRdy
    MOVT    R1, #:upper16:OSPrioHighRdy
    LDRB    R2, [R1]
    STRB    R2, [R0]

    MOVW    R1, #:lower16:OSTCBHighRdyPtr                      @ OSTCBCurPtr = OSTCBHighRdyPtr;
    MOVT    R1, #:upper16:OSTCBHighRdyPtr
    LDR     R2, [R1]
    STR     R2, [R5]

    ORR     LR, R4, #0xF4                                       @ Ensure exception return uses process stack
    LDR     R0, [R2]                                            @ R0 is new process SP; SP = OSTCBHighRdyPtr->StkPtr;
    LDMFD   R0!, {R4-R11}                                       @ Restore r4-11 from new process stack
    MSR     PSP, R0                                             @ Load PSP with new process SP
    CPSIE   I
    BX      LR                                                  @ Exception return will restore remaining context

.end
