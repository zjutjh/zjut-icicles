/*
************************************************************************************************************************
*                                                      uC/OS-III
*                                                 The Real-Time Kernel
*
*                                  (c) Copyright 2009-2015; Micrium, Inc.; Weston, FL
*                           All rights reserved.  Protected by international copyright laws.
*
*                                                   TIME MANAGEMENT
*
* File    : OS_TIME.C
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

#define  MICRIUM_SOURCE
#include "os.h"

#ifdef VSC_INCLUDE_SOURCE_FILE_NAMES
const  CPU_CHAR  *os_time__c = "$Id: $";
#endif

/*
************************************************************************************************************************
*                                                  DELAY TASK 'n' TICKS
*
* Description: This function is called to delay execution of the currently running task until the specified number of
*              system ticks expires.  This, of course, directly equates to delaying the current task for some time to
*              expire.  No delay will result if the specified delay is 0.  If the specified delay is greater than 0
*              then, a context switch will result.
*
* Arguments  : dly       is a value in 'clock ticks' that the task will either delay for or, the target match value
*                        of the tick counter (OSTickCtr).  Note that specifying 0 means the task is not to delay.
*
*                        depending on the option argument, the task will wake up when OSTickCtr reaches:
*
*                            OS_OPT_TIME_DLY      : OSTickCtr + dly
*                            OS_OPT_TIME_TIMEOUT  : OSTickCtr + dly
*                            OS_OPT_TIME_MATCH    : dly
*                            OS_OPT_TIME_PERIODIC : OSTCBCurPtr.TickCtrPrev + dly
*
*              opt       specifies whether 'dly' represents absolute or relative time; default option marked with *** :
*
*                        *** OS_OPT_TIME_DLY        specifies a relative time from the current value of OSTickCtr.
*                            OS_OPT_TIME_TIMEOUT    same as OS_OPT_TIME_DLY.
*                            OS_OPT_TIME_MATCH      indicates that 'dly' specifies the absolute value that OSTickCtr
*                                                   must reach before the task will be resumed.
*                            OS_OPT_TIME_PERIODIC   indicates that 'dly' specifies the periodic value that OSTickCtr
*                                                   must reach before the task will be resumed.
*
*              p_err     is a pointer to a variable that will contain an error code from this call.
*
*                            OS_ERR_NONE            the call was successful and the delay occurred.
*                            OS_ERR_OPT_INVALID     if you specified an invalid option for this function.
*                            OS_ERR_SCHED_LOCKED    can't delay when the scheduler is locked.
*                            OS_ERR_TIME_DLY_ISR    if you called this function from an ISR.
*                            OS_ERR_TIME_ZERO_DLY   if you specified a delay of zero.
*
* Returns    : none
************************************************************************************************************************
*/

void  OSTimeDly (OS_TICK   dly,
                 OS_OPT    opt,
                 OS_ERR   *p_err)
{
    CPU_SR_ALLOC();



#ifdef OS_SAFETY_CRITICAL
    if (p_err == (OS_ERR *)0) {
        OS_SAFETY_CRITICAL_EXCEPTION();
        return;
    }
#endif

#if OS_CFG_CALLED_FROM_ISR_CHK_EN > 0u
    if (OSIntNestingCtr > (OS_NESTING_CTR)0u) {             /* Not allowed to call from an ISR                        */
       *p_err = OS_ERR_TIME_DLY_ISR;
        return;
    }
#endif

    if (OSSchedLockNestingCtr > (OS_NESTING_CTR)0u) {       /* Can't delay when the scheduler is locked               */
       *p_err = OS_ERR_SCHED_LOCKED;
        return;
    }

    switch (opt) {
        case OS_OPT_TIME_DLY:
        case OS_OPT_TIME_TIMEOUT:
        case OS_OPT_TIME_PERIODIC:
             if (dly == (OS_TICK)0u) {                      /* 0 means no delay!                                      */
                *p_err = OS_ERR_TIME_ZERO_DLY;
                 return;
             }
             break;

        case OS_OPT_TIME_MATCH:
             break;

        default:
            *p_err = OS_ERR_OPT_INVALID;
             return;
    }

    OS_CRITICAL_ENTER();
    OS_TickListInsertDly(OSTCBCurPtr,
                         dly,
                         opt,
                         p_err);
    if (*p_err != OS_ERR_NONE) {
         OS_CRITICAL_EXIT_NO_SCHED();
         return;
    }
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
    TRACE_OS_TASK_DLY(dly);                                 /* Record the event.                                      */
#endif
    OS_RdyListRemove(OSTCBCurPtr);                          /* Remove current task from ready list                    */
    OS_CRITICAL_EXIT_NO_SCHED();
    OSSched();                                              /* Find next task to run!                                 */
   *p_err = OS_ERR_NONE;
}


/*
************************************************************************************************************************
*                                             DELAY TASK FOR SPECIFIED TIME
*
* Description: This function is called to delay execution of the currently running task until some time expires.  This
*              call allows you to specify the delay time in HOURS, MINUTES, SECONDS and MILLISECONDS instead of ticks.
*
* Arguments  : hours     specifies the number of hours that the task will be delayed (max. is 999 if the tick rate is
*                        1000 Hz or less otherwise, a higher value would overflow a 32-bit unsigned counter).
*
*              minutes   specifies the number of minutes (max. 59 if 'opt' is OS_OPT_TIME_HMSM_STRICT)
*
*              seconds   specifies the number of seconds (max. 59 if 'opt' is OS_OPT_TIME_HMSM_STRICT)
*
*              milli     specifies the number of milliseconds (max. 999 if 'opt' is OS_OPT_TIME_HMSM_STRICT)
*
*              opt       specifies time delay bit-field options logically OR'd; default options marked with *** :
*
*                        *** OS_OPT_TIME_DLY        specifies a relative time from the current value of OSTickCtr.
*                            OS_OPT_TIME_TIMEOUT    same as OS_OPT_TIME_DLY.
*                            OS_OPT_TIME_MATCH      indicates that the delay specifies the absolute value that OSTickCtr
*                                                   must reach before the task will be resumed.
*                            OS_OPT_TIME_PERIODIC   indicates that the delay specifies the periodic value that OSTickCtr
*                                                   must reach before the task will be resumed.
*
*                        *** OS_OPT_TIME_HMSM_STRICT            strictly allow only hours        (0...99)
*                                                                                   minutes      (0...59)
*                                                                                   seconds      (0...59)
*                                                                                   milliseconds (0...999)
*                            OS_OPT_TIME_HMSM_NON_STRICT        allow any value of  hours        (0...999)
*                                                                                   minutes      (0...9999)
*                                                                                   seconds      (0...65535)
*                                                                                   milliseconds (0...4294967295)
*
*              p_err     is a pointer to a variable that will receive an error code from this call.
*
*                            OS_ERR_NONE                        If the function returns from the desired delay
*                            OS_ERR_OPT_INVALID                 If you specified an invalid option for 'opt'
*                            OS_ERR_SCHED_LOCKED                Can't delay when the scheduler is locked
*                            OS_ERR_TIME_DLY_ISR                If called from an ISR
*                            OS_ERR_TIME_INVALID_HOURS          If you didn't specify a valid value for 'hours'
*                            OS_ERR_TIME_INVALID_MINUTES        If you didn't specify a valid value for 'minutes'
*                            OS_ERR_TIME_INVALID_SECONDS        If you didn't specify a valid value for 'seconds'
*                            OS_ERR_TIME_INVALID_MILLISECONDS   If you didn't specify a valid value for 'milli'
*                            OS_ERR_TIME_ZERO_DLY               If hours, minutes, seconds and milli are all 0
*
* Returns    : none
*
* Note(s)    : 1) The resolution on the milliseconds depends on the tick rate.  For example, you can't do a 10 mS delay
*                 if the ticker interrupts every 100 mS.  In this case, the delay would be set to 0.  The actual delay
*                 is rounded to the nearest tick.
*
*              2) Although this function allows you to delay a task for many, many hours, it's not recommended to put
*                 a task to sleep for that long.
************************************************************************************************************************
*/

#if OS_CFG_TIME_DLY_HMSM_EN > 0u
void  OSTimeDlyHMSM (CPU_INT16U   hours,
                     CPU_INT16U   minutes,
                     CPU_INT16U   seconds,
                     CPU_INT32U   milli,
                     OS_OPT       opt,
                     OS_ERR      *p_err)
{
#if OS_CFG_ARG_CHK_EN > 0u
    CPU_BOOLEAN  opt_invalid;
    CPU_BOOLEAN  opt_non_strict;
#endif
    OS_OPT       opt_time;
    OS_RATE_HZ   tick_rate;
    OS_TICK      ticks;
    CPU_SR_ALLOC();



#ifdef OS_SAFETY_CRITICAL
    if (p_err == (OS_ERR *)0) {
        OS_SAFETY_CRITICAL_EXCEPTION();
        return;
    }
#endif

#if OS_CFG_CALLED_FROM_ISR_CHK_EN > 0u
    if (OSIntNestingCtr > (OS_NESTING_CTR)0u) {             /* Not allowed to call from an ISR                        */
       *p_err = OS_ERR_TIME_DLY_ISR;
        return;
    }
#endif

    if (OSSchedLockNestingCtr > (OS_NESTING_CTR)0u) {       /* Can't delay when the scheduler is locked               */
       *p_err = OS_ERR_SCHED_LOCKED;
        return;
    }

    opt_time = opt & OS_OPT_TIME_MASK;                      /* Retrieve time options only.                            */
    switch (opt_time) {
        case OS_OPT_TIME_DLY:
        case OS_OPT_TIME_TIMEOUT:
        case OS_OPT_TIME_PERIODIC:
             if (milli == (CPU_INT32U)0u) {                 /* Make sure we didn't specify a 0 delay                  */
                 if (seconds == (CPU_INT16U)0u) {
                     if (minutes == (CPU_INT16U)0u) {
                         if (hours == (CPU_INT16U)0u) {
                            *p_err = OS_ERR_TIME_ZERO_DLY;
                             return;
                         }
                     }
                 }
             }
             break;

        case OS_OPT_TIME_MATCH:
             break;

        default:
            *p_err = OS_ERR_OPT_INVALID;
             return;
    }

#if OS_CFG_ARG_CHK_EN > 0u                                  /* Validate arguments to be within range                  */
    opt_invalid = DEF_BIT_IS_SET_ANY(opt, ~OS_OPT_TIME_OPTS_MASK);
    if (opt_invalid == DEF_YES) {
       *p_err = OS_ERR_OPT_INVALID;
        return;
    }

    opt_non_strict = DEF_BIT_IS_SET(opt, OS_OPT_TIME_HMSM_NON_STRICT);
    if (opt_non_strict != DEF_YES) {
         if (milli   > (CPU_INT32U)999u) {
            *p_err = OS_ERR_TIME_INVALID_MILLISECONDS;
             return;
         }
         if (seconds > (CPU_INT16U)59u) {
            *p_err = OS_ERR_TIME_INVALID_SECONDS;
             return;
         }
         if (minutes > (CPU_INT16U)59u) {
            *p_err = OS_ERR_TIME_INVALID_MINUTES;
             return;
         }
         if (hours   > (CPU_INT16U)99u) {
            *p_err = OS_ERR_TIME_INVALID_HOURS;
             return;
         }
    } else {
         if (minutes > (CPU_INT16U)9999u) {
            *p_err = OS_ERR_TIME_INVALID_MINUTES;
             return;
         }
         if (hours   > (CPU_INT16U)999u) {
            *p_err = OS_ERR_TIME_INVALID_HOURS;
             return;
         }
    }
#endif

                                                            /* Compute the total number of clock ticks required..     */
                                                            /* .. (rounded to the nearest tick)                       */
    tick_rate = OSCfg_TickRate_Hz;
    ticks     = ((OS_TICK)hours * (OS_TICK)3600u + (OS_TICK)minutes * (OS_TICK)60u + (OS_TICK)seconds) * tick_rate
              + (tick_rate * ((OS_TICK)milli + (OS_TICK)500u / tick_rate)) / (OS_TICK)1000u;

    if (ticks > (OS_TICK)0u) {
        OS_CRITICAL_ENTER();
        OS_TickListInsertDly(OSTCBCurPtr,
                             ticks,
                             opt_time,
                             p_err);
        if (*p_err != OS_ERR_NONE) {
             OS_CRITICAL_EXIT_NO_SCHED();
             return;
        }
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
        TRACE_OS_TASK_DLY(ticks);                           /* Record the event.                                      */
#endif
        OS_RdyListRemove(OSTCBCurPtr);                      /* Remove current task from ready list                    */
        OS_CRITICAL_EXIT_NO_SCHED();
        OSSched();                                          /* Find next task to run!                                 */
       *p_err = OS_ERR_NONE;
    } else {
       *p_err = OS_ERR_TIME_ZERO_DLY;
    }
}
#endif

/*
************************************************************************************************************************
*                                                RESUME A DELAYED TASK
*
* Description: This function is used resume a task that has been delayed through a call to either OSTimeDly() or
*              OSTimeDlyHMSM().  Note that you cannot call this function to resume a task that is waiting for an event
*              with timeout.
*
* Arguments  : p_tcb    is a pointer to the TCB of the task to resume.
*
*              p_err    is a pointer to a variable that will receive an error code
*
*                           OS_ERR_NONE                  Task has been resumed
*                           OS_ERR_STATE_INVALID         Task is in an invalid state
*                           OS_ERR_TIME_DLY_RESUME_ISR   If called from an ISR
*                           OS_ERR_TIME_NOT_DLY          Task is not waiting for time to expire
*                           OS_ERR_TASK_SUSPENDED        Task cannot be resumed, it was suspended by OSTaskSuspend()
*
* Note(s)    : none
************************************************************************************************************************
*/

#if OS_CFG_TIME_DLY_RESUME_EN > 0u
void  OSTimeDlyResume (OS_TCB  *p_tcb,
                       OS_ERR  *p_err)
{
    CPU_SR_ALLOC();



#ifdef OS_SAFETY_CRITICAL
    if (p_err == (OS_ERR *)0) {
        OS_SAFETY_CRITICAL_EXCEPTION();
        return;
    }
#endif

#if OS_CFG_CALLED_FROM_ISR_CHK_EN > 0u
    if (OSIntNestingCtr > (OS_NESTING_CTR)0u) {             /* Not allowed to call from an ISR                        */
       *p_err = OS_ERR_TIME_DLY_RESUME_ISR;
        return;
    }
#endif

#if OS_CFG_ARG_CHK_EN > 0u
    if (p_tcb == (OS_TCB *)0) {                             /* Not possible for the running task to be delayed!       */
       *p_err = OS_ERR_TASK_NOT_DLY;
        return;
    }
#endif

    CPU_CRITICAL_ENTER();
    if (p_tcb == OSTCBCurPtr) {                             /* Not possible for the running task to be delayed!       */
       *p_err = OS_ERR_TASK_NOT_DLY;
        CPU_CRITICAL_EXIT();
        return;
    }

    switch (p_tcb->TaskState) {
        case OS_TASK_STATE_RDY:                             /* Cannot Abort delay if task is ready                    */
             CPU_CRITICAL_EXIT();
            *p_err = OS_ERR_TASK_NOT_DLY;
             break;

        case OS_TASK_STATE_DLY:
             OS_CRITICAL_ENTER_CPU_EXIT();
             p_tcb->TaskState = OS_TASK_STATE_RDY;
             OS_TickListRemove(p_tcb);                      /* Remove task from tick list                             */
             OS_RdyListInsert(p_tcb);                       /* Add to ready list                                      */
             OS_CRITICAL_EXIT_NO_SCHED();
            *p_err = OS_ERR_NONE;
             break;

        case OS_TASK_STATE_PEND:
             CPU_CRITICAL_EXIT();
            *p_err = OS_ERR_TASK_NOT_DLY;
             break;

        case OS_TASK_STATE_PEND_TIMEOUT:
             CPU_CRITICAL_EXIT();
            *p_err = OS_ERR_TASK_NOT_DLY;
             break;

        case OS_TASK_STATE_SUSPENDED:
             CPU_CRITICAL_EXIT();
            *p_err = OS_ERR_TASK_NOT_DLY;
             break;

        case OS_TASK_STATE_DLY_SUSPENDED:
             OS_CRITICAL_ENTER_CPU_EXIT();
             p_tcb->TaskState = OS_TASK_STATE_SUSPENDED;
             OS_TickListRemove(p_tcb);                      /* Remove task from tick list                             */
             OS_CRITICAL_EXIT_NO_SCHED();
            *p_err            = OS_ERR_TASK_SUSPENDED;
             break;

        case OS_TASK_STATE_PEND_SUSPENDED:
             CPU_CRITICAL_EXIT();
            *p_err = OS_ERR_TASK_NOT_DLY;
             break;

        case OS_TASK_STATE_PEND_TIMEOUT_SUSPENDED:
             CPU_CRITICAL_EXIT();
            *p_err = OS_ERR_TASK_NOT_DLY;
             break;

        default:
             CPU_CRITICAL_EXIT();
            *p_err = OS_ERR_STATE_INVALID;
             break;
    }

    OSSched();
}
#endif

/*
************************************************************************************************************************
*                                               GET CURRENT SYSTEM TIME
*
* Description: This function is used by your application to obtain the current value of the counter which keeps track of
*              the number of clock ticks.
*
* Arguments  : p_err    is a pointer to a variable that will receive an error code
*
*                           OS_ERR_NONE           If the call was successful
*
* Returns    : The current value of OSTickCtr
************************************************************************************************************************
*/

OS_TICK  OSTimeGet (OS_ERR  *p_err)
{
    OS_TICK  ticks;
    CPU_SR_ALLOC();



#ifdef OS_SAFETY_CRITICAL
    if (p_err == (OS_ERR *)0) {
        OS_SAFETY_CRITICAL_EXCEPTION();
        return ((OS_TICK)0);
    }
#endif

    CPU_CRITICAL_ENTER();
    ticks = OSTickCtr;
    CPU_CRITICAL_EXIT();
   *p_err = OS_ERR_NONE;
    return (ticks);
}

/*
************************************************************************************************************************
*                                                   SET SYSTEM CLOCK
*
* Description: This function sets the counter which keeps track of the number of clock ticks.
*
* Arguments  : ticks    is the desired tick value
*
*              p_err    is a pointer to a variable that will receive an error code
*
*                           OS_ERR_NONE           If the call was successful
*
* Returns    : none
************************************************************************************************************************
*/

void  OSTimeSet (OS_TICK   ticks,
                 OS_ERR   *p_err)
{
    CPU_SR_ALLOC();



#ifdef OS_SAFETY_CRITICAL
    if (p_err == (OS_ERR *)0) {
        OS_SAFETY_CRITICAL_EXCEPTION();
        return;
    }
#endif

    CPU_CRITICAL_ENTER();
    OSTickCtr = ticks;
    CPU_CRITICAL_EXIT();
   *p_err     = OS_ERR_NONE;
}


/*
************************************************************************************************************************
*                                                 PROCESS SYSTEM TICK
*
* Description: This function is used to signal to uC/OS-III the occurrence of a 'system tick' (also known as a
*              'clock tick').  This function should be called by the tick ISR.
*
* Arguments  : none
*
* Returns    : none
************************************************************************************************************************
*/

void  OSTimeTick (void)
{
    OS_ERR  err;
#if OS_CFG_ISR_POST_DEFERRED_EN > 0u
    CPU_TS  ts;
#endif


    OSTimeTickHook();                                       /* Call user definable hook                               */

#if OS_CFG_ISR_POST_DEFERRED_EN > 0u

    ts = OS_TS_GET();                                       /* Get timestamp                                          */
    OS_IntQPost((OS_OBJ_TYPE) OS_OBJ_TYPE_TICK,             /* Post to ISR queue                                      */
                (void      *)&OSRdyList[OSPrioCur],
                (void      *) 0,
                (OS_MSG_SIZE) 0u,
                (OS_FLAGS   ) 0u,
                (OS_OPT     ) 0u,
                (CPU_TS     ) ts,
                (OS_ERR    *)&err);

#else

   (void)OSTaskSemPost((OS_TCB *)&OSTickTaskTCB,            /* Signal tick task                                       */
                       (OS_OPT  ) OS_OPT_POST_NONE,
                       (OS_ERR *)&err);

   (void)err;

#if OS_CFG_SCHED_ROUND_ROBIN_EN > 0u
    OS_SchedRoundRobin(&OSRdyList[OSPrioCur]);
#endif

#if OS_CFG_TMR_EN > 0u
    OSTmrUpdateCtr--;
    if (OSTmrUpdateCtr == (OS_CTR)0u) {
        OSTmrUpdateCtr = OSTmrUpdateCnt;
        OSTaskSemPost((OS_TCB *)&OSTmrTaskTCB,              /* Signal timer task                                      */
                      (OS_OPT  ) OS_OPT_POST_NONE,
                      (OS_ERR *)&err);
    }
#endif

#endif
}
