/*
************************************************************************************************************************
*                                                      uC/OS-III
*                                                 The Real-Time Kernel
*
*                                  (c) Copyright 2009-2015; Micrium, Inc.; Weston, FL
*                           All rights reserved.  Protected by international copyright laws.
*
*                                       OS CONFIGURATION (APPLICATION SPECIFICS)
*
* File    : OS_CFG_APP.C
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
* Note(s) : DO NOT CHANGE THIS FILE!
************************************************************************************************************************
*/

#define  MICRIUM_SOURCE
#include <os_cfg_app.h>
#include "os.h"

#ifdef VSC_INCLUDE_SOURCE_FILE_NAMES
const  CPU_CHAR  *os_cfg_app__c = "$Id: $";
#endif

#define  OS_CFG_IDLE_TASK_STK_LIMIT      ((OS_CFG_IDLE_TASK_STK_SIZE  * OS_CFG_TASK_STK_LIMIT_PCT_EMPTY) / 100u)
#define  OS_CFG_INT_Q_TASK_STK_LIMIT     ((OS_CFG_INT_Q_TASK_STK_SIZE * OS_CFG_TASK_STK_LIMIT_PCT_EMPTY) / 100u)
#define  OS_CFG_STAT_TASK_STK_LIMIT      ((OS_CFG_STAT_TASK_STK_SIZE  * OS_CFG_TASK_STK_LIMIT_PCT_EMPTY) / 100u)
#define  OS_CFG_TICK_TASK_STK_LIMIT      ((OS_CFG_TICK_TASK_STK_SIZE  * OS_CFG_TASK_STK_LIMIT_PCT_EMPTY) / 100u)
#define  OS_CFG_TMR_TASK_STK_LIMIT       ((OS_CFG_TMR_TASK_STK_SIZE   * OS_CFG_TASK_STK_LIMIT_PCT_EMPTY) / 100u)

/*
************************************************************************************************************************
*                                                    DATA STORAGE
************************************************************************************************************************
*/

CPU_STK        OSCfg_IdleTaskStk   [OS_CFG_IDLE_TASK_STK_SIZE];

#if (OS_CFG_ISR_POST_DEFERRED_EN > 0u)
OS_INT_Q       OSCfg_IntQ          [OS_CFG_INT_Q_SIZE];
CPU_STK        OSCfg_IntQTaskStk   [OS_CFG_INT_Q_TASK_STK_SIZE];
#endif

#if (OS_CFG_ISR_STK_SIZE > 0u)
CPU_STK        OSCfg_ISRStk        [OS_CFG_ISR_STK_SIZE];
#endif

#if (OS_MSG_EN > 0u)
OS_MSG         OSCfg_MsgPool       [OS_CFG_MSG_POOL_SIZE];
#endif

#if (OS_CFG_STAT_TASK_EN > 0u)
CPU_STK        OSCfg_StatTaskStk   [OS_CFG_STAT_TASK_STK_SIZE];
#endif

CPU_STK        OSCfg_TickTaskStk   [OS_CFG_TICK_TASK_STK_SIZE];

#if (OS_CFG_TMR_EN > 0u)
CPU_STK        OSCfg_TmrTaskStk    [OS_CFG_TMR_TASK_STK_SIZE];
#endif

/*
************************************************************************************************************************
*                                                      CONSTANTS
************************************************************************************************************************
*/

CPU_STK      * const  OSCfg_IdleTaskStkBasePtr   = (CPU_STK    *)&OSCfg_IdleTaskStk[0];
CPU_STK_SIZE   const  OSCfg_IdleTaskStkLimit     = (CPU_STK_SIZE)OS_CFG_IDLE_TASK_STK_LIMIT;
CPU_STK_SIZE   const  OSCfg_IdleTaskStkSize      = (CPU_STK_SIZE)OS_CFG_IDLE_TASK_STK_SIZE;
CPU_INT32U     const  OSCfg_IdleTaskStkSizeRAM   = (CPU_INT32U  )sizeof(OSCfg_IdleTaskStk);


#if (OS_CFG_ISR_POST_DEFERRED_EN > 0u)
OS_INT_Q     * const  OSCfg_IntQBasePtr          = (OS_INT_Q   *)&OSCfg_IntQ[0];
OS_OBJ_QTY     const  OSCfg_IntQSize             = (OS_OBJ_QTY  )OS_CFG_INT_Q_SIZE;
CPU_INT32U     const  OSCfg_IntQSizeRAM          = (CPU_INT32U  )sizeof(OSCfg_IntQ);
CPU_STK      * const  OSCfg_IntQTaskStkBasePtr   = (CPU_STK    *)&OSCfg_IntQTaskStk[0];
CPU_STK_SIZE   const  OSCfg_IntQTaskStkLimit     = (CPU_STK_SIZE)OS_CFG_INT_Q_TASK_STK_LIMIT;
CPU_STK_SIZE   const  OSCfg_IntQTaskStkSize      = (CPU_STK_SIZE)OS_CFG_INT_Q_TASK_STK_SIZE;
CPU_INT32U     const  OSCfg_IntQTaskStkSizeRAM   = (CPU_INT32U  )sizeof(OSCfg_IntQTaskStk);
#else
OS_INT_Q     * const  OSCfg_IntQBasePtr          = (OS_INT_Q   *)0;
OS_OBJ_QTY     const  OSCfg_IntQSize             = (OS_OBJ_QTY  )0;
CPU_INT32U     const  OSCfg_IntQSizeRAM          = (CPU_INT32U  )0;
CPU_STK      * const  OSCfg_IntQTaskStkBasePtr   = (CPU_STK    *)0;
CPU_STK_SIZE   const  OSCfg_IntQTaskStkLimit     = (CPU_STK_SIZE)0;
CPU_STK_SIZE   const  OSCfg_IntQTaskStkSize      = (CPU_STK_SIZE)0;
CPU_INT32U     const  OSCfg_IntQTaskStkSizeRAM   = (CPU_INT32U  )0;
#endif


#if (OS_CFG_ISR_STK_SIZE > 0u)
CPU_STK      * const  OSCfg_ISRStkBasePtr        = (CPU_STK    *)&OSCfg_ISRStk[0];
CPU_STK_SIZE   const  OSCfg_ISRStkSize           = (CPU_STK_SIZE)OS_CFG_ISR_STK_SIZE;
CPU_INT32U     const  OSCfg_ISRStkSizeRAM        = (CPU_INT32U  )sizeof(OSCfg_ISRStk);
#else
CPU_STK      * const  OSCfg_ISRStkBasePtr        = (CPU_STK    *)0;
CPU_STK_SIZE   const  OSCfg_ISRStkSize           = (CPU_STK_SIZE)0;
CPU_INT32U     const  OSCfg_ISRStkSizeRAM        = (CPU_INT32U  )0;
#endif


#if (OS_MSG_EN > 0u)
OS_MSG_SIZE    const  OSCfg_MsgPoolSize          = (OS_MSG_SIZE)OS_CFG_MSG_POOL_SIZE;
CPU_INT32U     const  OSCfg_MsgPoolSizeRAM       = (CPU_INT32U )sizeof(OSCfg_MsgPool);
OS_MSG       * const  OSCfg_MsgPoolBasePtr       = (OS_MSG    *)&OSCfg_MsgPool[0];
#else
OS_MSG_SIZE    const  OSCfg_MsgPoolSize          = (OS_MSG_SIZE)0;
CPU_INT32U     const  OSCfg_MsgPoolSizeRAM       = (CPU_INT32U )0;
OS_MSG       * const  OSCfg_MsgPoolBasePtr       = (OS_MSG    *)0;
#endif


#if (OS_CFG_STAT_TASK_EN > 0u)
OS_PRIO        const  OSCfg_StatTaskPrio         = (OS_PRIO     )OS_CFG_STAT_TASK_PRIO;
OS_RATE_HZ     const  OSCfg_StatTaskRate_Hz      = (OS_RATE_HZ  )OS_CFG_STAT_TASK_RATE_HZ;
CPU_STK      * const  OSCfg_StatTaskStkBasePtr   = (CPU_STK    *)&OSCfg_StatTaskStk[0];
CPU_STK_SIZE   const  OSCfg_StatTaskStkLimit     = (CPU_STK_SIZE)OS_CFG_STAT_TASK_STK_LIMIT;
CPU_STK_SIZE   const  OSCfg_StatTaskStkSize      = (CPU_STK_SIZE)OS_CFG_STAT_TASK_STK_SIZE;
CPU_INT32U     const  OSCfg_StatTaskStkSizeRAM   = (CPU_INT32U  )sizeof(OSCfg_StatTaskStk);
#else
OS_PRIO        const  OSCfg_StatTaskPrio         = (OS_PRIO     )0;
OS_RATE_HZ     const  OSCfg_StatTaskRate_Hz      = (OS_RATE_HZ  )0;
CPU_STK      * const  OSCfg_StatTaskStkBasePtr   = (CPU_STK    *)0;
CPU_STK_SIZE   const  OSCfg_StatTaskStkLimit     = (CPU_STK_SIZE)0;
CPU_STK_SIZE   const  OSCfg_StatTaskStkSize      = (CPU_STK_SIZE)0;
CPU_INT32U     const  OSCfg_StatTaskStkSizeRAM   = (CPU_INT32U  )0;
#endif


CPU_STK_SIZE   const  OSCfg_StkSizeMin           = (CPU_STK_SIZE)OS_CFG_STK_SIZE_MIN;


OS_RATE_HZ     const  OSCfg_TickRate_Hz          = (OS_RATE_HZ  )OS_CFG_TICK_RATE_HZ;
OS_PRIO        const  OSCfg_TickTaskPrio         = (OS_PRIO     )OS_CFG_TICK_TASK_PRIO;
CPU_STK      * const  OSCfg_TickTaskStkBasePtr   = (CPU_STK    *)&OSCfg_TickTaskStk[0];
CPU_STK_SIZE   const  OSCfg_TickTaskStkLimit     = (CPU_STK_SIZE)OS_CFG_TICK_TASK_STK_LIMIT;
CPU_STK_SIZE   const  OSCfg_TickTaskStkSize      = (CPU_STK_SIZE)OS_CFG_TICK_TASK_STK_SIZE;
CPU_INT32U     const  OSCfg_TickTaskStkSizeRAM   = (CPU_INT32U  )sizeof(OSCfg_TickTaskStk);


#if (OS_CFG_TMR_EN > 0u)
OS_PRIO        const  OSCfg_TmrTaskPrio          = (OS_PRIO     )OS_CFG_TMR_TASK_PRIO;
OS_RATE_HZ     const  OSCfg_TmrTaskRate_Hz       = (OS_RATE_HZ  )OS_CFG_TMR_TASK_RATE_HZ;
CPU_STK      * const  OSCfg_TmrTaskStkBasePtr    = (CPU_STK    *)&OSCfg_TmrTaskStk[0];
CPU_STK_SIZE   const  OSCfg_TmrTaskStkLimit      = (CPU_STK_SIZE)OS_CFG_TMR_TASK_STK_LIMIT;
CPU_STK_SIZE   const  OSCfg_TmrTaskStkSize       = (CPU_STK_SIZE)OS_CFG_TMR_TASK_STK_SIZE;
CPU_INT32U     const  OSCfg_TmrTaskStkSizeRAM    = (CPU_INT32U  )sizeof(OSCfg_TmrTaskStk);
#else
OS_PRIO        const  OSCfg_TmrTaskPrio          = (OS_PRIO     )0;
OS_RATE_HZ     const  OSCfg_TmrTaskRate_Hz       = (OS_RATE_HZ  )0;
CPU_STK      * const  OSCfg_TmrTaskStkBasePtr    = (CPU_STK    *)0;
CPU_STK_SIZE   const  OSCfg_TmrTaskStkLimit      = (CPU_STK_SIZE)0;
CPU_STK_SIZE   const  OSCfg_TmrTaskStkSize       = (CPU_STK_SIZE)0;
CPU_INT32U     const  OSCfg_TmrTaskStkSizeRAM    = (CPU_INT32U  )0;
#endif


/*
************************************************************************************************************************
*                                         TOTAL SIZE OF APPLICATION CONFIGURATION
************************************************************************************************************************
*/

CPU_INT32U     const  OSCfg_DataSizeRAM          = sizeof(OSCfg_IdleTaskStk)

#if (OS_CFG_ISR_POST_DEFERRED_EN > 0u)
                                                 + sizeof(OSCfg_IntQ)
                                                 + sizeof(OSCfg_IntQTaskStk)
#endif

#if (OS_MSG_EN > 0u)
                                                 + sizeof(OSCfg_MsgPool)
#endif

#if (OS_CFG_STAT_TASK_EN > 0u)
                                                 + sizeof(OSCfg_StatTaskStk)
#endif

#if (OS_CFG_TMR_EN > 0u)
                                                 + sizeof(OSCfg_TmrTaskStk)
#endif

#if (OS_CFG_ISR_STK_SIZE > 0u)
                                                 + sizeof(OSCfg_ISRStk)
#endif
                                                 + sizeof(OSCfg_TickTaskStk);


/*
************************************************************************************************************************
*                                             OS CONFIGURATION INITIALIZATION
*
* Description: This function is used to make sure that debug variables that are unused in the application are not
*              optimized away.  This function might not be necessary for all compilers.  In this case, you should simply
*              DELETE the code in this function while still leaving the declaration of the function itself.
*
* Arguments  : none
*
* Returns    : none
*
* Note(s)    : (1) This code doesn't do anything, it simply prevents the compiler from optimizing out the 'const'
*                  variables which are declared in this file.
*              (2) You may decide to 'compile out' the code (by using #if 0/#endif) INSIDE the function if your compiler
*                  DOES NOT optimize out the 'const' variables above.
************************************************************************************************************************
*/

void  OSCfg_Init (void)
{
    (void)&OSCfg_DataSizeRAM;

    (void)&OSCfg_IdleTaskStkBasePtr;
    (void)&OSCfg_IdleTaskStkLimit;
    (void)&OSCfg_IdleTaskStkSize;
    (void)&OSCfg_IdleTaskStkSizeRAM;

#if (OS_CFG_ISR_POST_DEFERRED_EN > 0u)
    (void)&OSCfg_IntQBasePtr;
    (void)&OSCfg_IntQSize;
    (void)&OSCfg_IntQSizeRAM;
    (void)&OSCfg_IntQTaskStkBasePtr;
    (void)&OSCfg_IntQTaskStkLimit;
    (void)&OSCfg_IntQTaskStkSize;
    (void)&OSCfg_IntQTaskStkSizeRAM;
#endif

    (void)&OSCfg_ISRStkBasePtr;
    (void)&OSCfg_ISRStkSize;
    (void)&OSCfg_ISRStkSizeRAM;

#if (OS_MSG_EN > 0u)
    (void)&OSCfg_MsgPoolSize;
    (void)&OSCfg_MsgPoolSizeRAM;
    (void)&OSCfg_MsgPoolBasePtr;
#endif

#if (OS_CFG_STAT_TASK_EN > 0u)
    (void)&OSCfg_StatTaskPrio;
    (void)&OSCfg_StatTaskRate_Hz;
    (void)&OSCfg_StatTaskStkBasePtr;
    (void)&OSCfg_StatTaskStkLimit;
    (void)&OSCfg_StatTaskStkSize;
    (void)&OSCfg_StatTaskStkSizeRAM;
#endif

    (void)&OSCfg_StkSizeMin;

    (void)&OSCfg_TickRate_Hz;
    (void)&OSCfg_TickTaskPrio;
    (void)&OSCfg_TickTaskStkBasePtr;
    (void)&OSCfg_TickTaskStkLimit;
    (void)&OSCfg_TickTaskStkSize;
    (void)&OSCfg_TickTaskStkSizeRAM;

#if (OS_CFG_TMR_EN > 0u)
    (void)&OSCfg_TmrTaskPrio;
    (void)&OSCfg_TmrTaskRate_Hz;
    (void)&OSCfg_TmrTaskStkBasePtr;
    (void)&OSCfg_TmrTaskStkLimit;
    (void)&OSCfg_TmrTaskStkSize;
    (void)&OSCfg_TmrTaskStkSizeRAM;
#endif
}
