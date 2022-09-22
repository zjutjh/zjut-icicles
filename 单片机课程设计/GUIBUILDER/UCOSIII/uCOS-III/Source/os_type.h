/*
************************************************************************************************************************
*                                                      uC/OS-III
*                                                 The Real-Time Kernel
*
*                                  (c) Copyright 2009-2015; Micrium, Inc.; Weston, FL
*                           All rights reserved.  Protected by international copyright laws.
*
* File    : OS_TYPE.H
* By      : JJL
* Version : V3.04.05
*
* LICENSING TERMS:
* ---------------
*           uC/OS-III is provided in source form for FREE short-term evaluation, for educational use or 
*           for peaceful research.  If you plan or intend to use uC/OS-III in a commercial application/
*           product then, you need to contact Micrium to properly license uC/OS-III for its use in your 
*           application/product.   We provide ALL the source code for your convenience and to help you 
*           experience uC/OS-III.  The fact that the source is provided does NOT mean that you can use 
*           it commercially without paying a licensing fee.
*
*           Knowledge of the source code may NOT be used to develop a similar product.
*
*           Please help us continue to provide the embedded community with the finest software available.
*           Your honesty is greatly appreciated.
*
*           You can find our product's user manual, API reference, release notes and
*           more information at https://doc.micrium.com.
*           You can contact us at www.micrium.com.
************************************************************************************************************************
*/

#ifndef   OS_TYPE_H
#define   OS_TYPE_H

#ifdef    VSC_INCLUDE_H_FILE_NAMES
const     CPU_CHAR  *os_type__h = "$Id: $";
#endif

/*
************************************************************************************************************************
*                                                 INCLUDE HEADER FILES
************************************************************************************************************************
*/

                                                       /*       Description                                    # Bits */
                                                       /*                                               <recommended> */
                                                       /* ----------------------------------------------------------- */

typedef   CPU_INT16U      OS_CPU_USAGE;                /* CPU Usage 0..10000                                  <16>/32 */

typedef   CPU_INT32U      OS_CTR;                      /* Counter,                                                 32 */

typedef   CPU_INT32U      OS_CTX_SW_CTR;               /* Counter of context switches,                             32 */

typedef   CPU_INT32U      OS_CYCLES;                   /* CPU clock cycles,                                   <32>/64 */

typedef   CPU_INT32U      OS_FLAGS;                    /* Event flags,                                      8/16/<32> */

typedef   CPU_INT32U      OS_IDLE_CTR;                 /* Holds the number of times the idle task runs,       <32>/64 */

typedef   CPU_INT16U      OS_MEM_QTY;                  /* Number of memory blocks,                            <16>/32 */
typedef   CPU_INT16U      OS_MEM_SIZE;                 /* Size in bytes of a memory block,                    <16>/32 */

typedef   CPU_INT16U      OS_MSG_QTY;                  /* Number of OS_MSGs in the msg pool,                  <16>/32 */
typedef   CPU_INT16U      OS_MSG_SIZE;                 /* Size of messages in number of bytes,                <16>/32 */

typedef   CPU_INT08U      OS_NESTING_CTR;              /* Interrupt and scheduler nesting,                  <8>/16/32 */

typedef   CPU_INT16U      OS_OBJ_QTY;                  /* Number of kernel objects counter,                   <16>/32 */
typedef   CPU_INT32U      OS_OBJ_TYPE;                 /* Special flag to determine object type,                   32 */

typedef   CPU_INT16U      OS_OPT;                      /* Holds function options                              <16>/32 */

typedef   CPU_INT08U      OS_PRIO;                     /* Priority of a task,                               <8>/16/32 */

typedef   CPU_INT16U      OS_QTY;                      /* Quantity                                            <16>/32 */

typedef   CPU_INT32U      OS_RATE_HZ;                  /* Rate in Hertz                                            32 */

typedef   CPU_INT32U      OS_REG;                      /* Task register                                     8/16/<32> */
typedef   CPU_INT08U      OS_REG_ID;                   /* Index to task register                            <8>/16/32 */

typedef   CPU_INT32U      OS_SEM_CTR;                  /* Semaphore value                                     16/<32> */

typedef   CPU_INT08U      OS_STATE;                    /* State variable                                    <8>/16/32 */

typedef   CPU_INT08U      OS_STATUS;                   /* Status                                            <8>/16/32 */

typedef   CPU_INT32U      OS_TICK;                     /* Clock tick counter                                  <32>/64 */

#endif
