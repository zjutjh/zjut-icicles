/*
***********************************************************************************************************************
*                                                      uC/OS-III
*                                                 The Real-Time Kernel
*
*                                  (c) Copyright 2009-2015; Micrium, Inc.; Weston, FL
*                           All rights reserved.  Protected by international copyright laws.
*
*                                                   TICK MANAGEMENT
*
* File    : OS_TICK.C
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
const  CPU_CHAR  *os_tick__c = "$Id: $";
#endif

/*
************************************************************************************************************************
*                                                 FUNCTION PROTOTYPES
************************************************************************************************************************
*/

static  CPU_TS  OS_TickListUpdateDly     (void);
static  CPU_TS  OS_TickListUpdateTimeout (void);

/*
************************************************************************************************************************
*                                                      TICK TASK
*
* Description: This task is internal to uC/OS-III and is triggered by the tick interrupt.
*
* Arguments  : p_arg     is an argument passed to the task when the task is created (unused).
*
* Returns    : none
*
* Note(s)    : This function is INTERNAL to uC/OS-III and your application should not call it.
************************************************************************************************************************
*/

void  OS_TickTask (void  *p_arg)
{
    OS_ERR  err;
    CPU_TS  ts_delta;
    CPU_TS  ts_delta_dly;
    CPU_TS  ts_delta_timeout;
    CPU_SR_ALLOC();


    (void)&p_arg;                                               /* Prevent compiler warning                             */

    while (DEF_ON) {
        (void)OSTaskSemPend((OS_TICK  )0,
                            (OS_OPT   )OS_OPT_PEND_BLOCKING,
                            (CPU_TS  *)0,
                            (OS_ERR  *)&err);                   /* Wait for signal from tick interrupt                  */
        if (err == OS_ERR_NONE) {
            OS_CRITICAL_ENTER();
            OSTickCtr++;                                        /* Keep track of the number of ticks                    */
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
            TRACE_OS_TICK_INCREMENT(OSTickCtr);                 /* Record the event.                                    */
#endif
            OS_CRITICAL_EXIT();
            ts_delta_dly     = OS_TickListUpdateDly();
            ts_delta_timeout = OS_TickListUpdateTimeout();
            ts_delta         = ts_delta_dly + ts_delta_timeout; /* Compute total execution time of list updates         */
            if (OSTickTaskTimeMax < ts_delta) {
                OSTickTaskTimeMax = ts_delta;
            }
        }
    }
}

/*
************************************************************************************************************************
*                                                 INITIALIZE TICK TASK
*
* Description: This function is called by OSInit() to create the tick task.
*
* Arguments  : p_err   is a pointer to a variable that will hold the value of an error code:
*
*                          OS_ERR_TICK_STK_INVALID   if the pointer to the tick task stack is a NULL pointer
*                          OS_ERR_TICK_STK_SIZE      indicates that the specified stack size
*                          OS_ERR_PRIO_INVALID       if the priority you specified in the configuration is invalid
*                                                      (There could be only one task at the Idle Task priority)
*                                                      (Maybe the priority you specified is higher than OS_CFG_PRIO_MAX-1
*                          OS_ERR_??                 other error code returned by OSTaskCreate()
*
* Returns    : none
*
* Note(s)    : This function is INTERNAL to uC/OS-III and your application should not call it.
************************************************************************************************************************
*/

void  OS_TickTaskInit (OS_ERR  *p_err)
{
#ifdef OS_SAFETY_CRITICAL
    if (p_err == (OS_ERR *)0) {
        OS_SAFETY_CRITICAL_EXCEPTION();
        return;
    }
#endif

    OSTickCtr                    = (OS_TICK)0u;                         /* Clear the tick counter                            */

    OSTickListDly.TCB_Ptr        = (OS_TCB   *)0;
    OSTickListTimeout.TCB_Ptr    = (OS_TCB   *)0;

#if OS_CFG_DBG_EN > 0u
    OSTickListDly.NbrEntries     = (OS_OBJ_QTY)0;
    OSTickListDly.NbrUpdated     = (OS_OBJ_QTY)0;

    OSTickListTimeout.NbrEntries = (OS_OBJ_QTY)0;
    OSTickListTimeout.NbrUpdated = (OS_OBJ_QTY)0;
#endif

                                                                        /* ---------------- CREATE THE TICK TASK ----------- */
    if (OSCfg_TickTaskStkBasePtr == (CPU_STK *)0) {
       *p_err = OS_ERR_TICK_STK_INVALID;
        return;
    }

    if (OSCfg_TickTaskStkSize < OSCfg_StkSizeMin) {
       *p_err = OS_ERR_TICK_STK_SIZE_INVALID;
        return;
    }

    if (OSCfg_TickTaskPrio >= (OS_CFG_PRIO_MAX - 1u)) {                 /* Only one task at the 'Idle Task' priority         */
       *p_err = OS_ERR_TICK_PRIO_INVALID;
        return;
    }

    OSTaskCreate((OS_TCB     *)&OSTickTaskTCB,
                 (CPU_CHAR   *)((void *)"uC/OS-III Tick Task"),
                 (OS_TASK_PTR )OS_TickTask,
                 (void       *)0,
                 (OS_PRIO     )OSCfg_TickTaskPrio,
                 (CPU_STK    *)OSCfg_TickTaskStkBasePtr,
                 (CPU_STK_SIZE)OSCfg_TickTaskStkLimit,
                 (CPU_STK_SIZE)OSCfg_TickTaskStkSize,
                 (OS_MSG_QTY  )0u,
                 (OS_TICK     )0u,
                 (void       *)0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_NO_TLS),
                 (OS_ERR     *)p_err);
}

/*
************************************************************************************************************************
*                                                      INSERT
*
* Description: This task is internal to uC/OS-III and allows the insertion of a task in a tick list. 
*
* Arguments  : p_list      is a pointer to the desired list
*
*              p_tcb       is a pointer to the TCB to insert in the list
*
*              time        is the amount of time remaining (in ticks) for the task to become ready
*
* Returns    : none
*
* Note(s)    : This function is INTERNAL to uC/OS-III and your application should not call it.
************************************************************************************************************************
*/

void  OS_TickListInsert (OS_TICK_LIST  *p_list,
                         OS_TCB        *p_tcb,
                         OS_TICK        time)
{
    OS_TCB  *p_tcb1;
    OS_TCB  *p_tcb2;
    OS_TICK  remain;


    if (p_list->TCB_Ptr == (OS_TCB *)0) {                               /* Is the list empty?                                */
        p_tcb->TickRemain  = time;                                      /* Yes, Store time in TCB                            */
        p_tcb->TickNextPtr = (OS_TCB *)0;
        p_tcb->TickPrevPtr = (OS_TCB *)0;
        p_tcb->TickListPtr = (OS_TICK_LIST *)p_list;                    /*      Link to this list                            */
        p_list->TCB_Ptr    = p_tcb;                                     /*      Point to TCB of task to place in the list    */
#if OS_CFG_DBG_EN > 0u
        p_list->NbrEntries = 1u;                                        /*      List contains 1 entry                        */
#endif
    } else {
        p_tcb1 = p_list->TCB_Ptr;
        p_tcb2 = p_list->TCB_Ptr;                                       /* No,  Insert somewhere in the list in delta order  */
        remain = time;
        while (p_tcb2 != (OS_TCB *)0) {
            if (remain <= p_tcb2->TickRemain) {
                if (p_tcb2->TickPrevPtr == (OS_TCB *)0) {               /*      Insert before the first entry in the list?   */
                    p_tcb->TickRemain   = remain;                       /*      Yes, Store remaining time                    */                                          
                    p_tcb->TickPrevPtr  = (OS_TCB *)0;
                    p_tcb->TickNextPtr  = p_tcb2;    
                    p_tcb->TickListPtr  = (OS_TICK_LIST *)p_list;       /*           Link TCB to this list                   */
                    p_tcb2->TickRemain -= remain;                       /*           Reduce time of next entry in the list   */
                    p_tcb2->TickPrevPtr = p_tcb;
                    p_list->TCB_Ptr     = p_tcb;                        /*           Add TCB to the list                     */
#if OS_CFG_DBG_EN > 0u
                    p_list->NbrEntries++;                               /*           List contains an extra entry            */
#endif
                } else {                                                /*      No,  Insert somewhere further in the list    */
                    p_tcb1              = p_tcb2->TickPrevPtr;
                    p_tcb->TickRemain   = remain;                       /*           Store remaining time                    */
                    p_tcb->TickPrevPtr  = p_tcb1;
                    p_tcb->TickNextPtr  = p_tcb2;    
                    p_tcb->TickListPtr  = (OS_TICK_LIST *)p_list;       /*           TCB points to this list                 */
                    p_tcb2->TickRemain -= remain;                       /*           Reduce time of next entry in the list   */
                    p_tcb2->TickPrevPtr = p_tcb;
                    p_tcb1->TickNextPtr = p_tcb;
#if OS_CFG_DBG_EN > 0u
                    p_list->NbrEntries++;                               /*           List contains an extra entry            */
#endif
                }
                return;
            } else {
                remain -= p_tcb2->TickRemain;                           /*           Point to the next TCB in the list       */
                p_tcb1  = p_tcb2;
                p_tcb2  = p_tcb2->TickNextPtr;
            }                 
        }
        p_tcb->TickRemain   = remain;                       
        p_tcb->TickPrevPtr  = p_tcb1;
        p_tcb->TickNextPtr  = (OS_TCB *)0;    
        p_tcb->TickListPtr  = (OS_TICK_LIST *)p_list;                   /*           Link the list to the TCB                */
        p_tcb1->TickNextPtr = p_tcb;
#if OS_CFG_DBG_EN > 0u
        p_list->NbrEntries++;                                           /*           List contains an extra entry            */
#endif
    }
}

/*
************************************************************************************************************************
*                                            ADD TASK TO DELAYED TICK LIST
*
* Description: This function is called to place a task in a list of task waiting for either time to expire 
*
* Arguments  : p_tcb          is a pointer to the OS_TCB of the task to add to the tick list
*              -----
*
*              time           represents either the 'match' value of OSTickCtr or a relative time from the current
*                             value of OSTickCtr as specified by the 'opt' argument..
*
*                             relative when 'opt' is set to OS_OPT_TIME_DLY
*                             relative when 'opt' is set to OS_OPT_TIME_TIMEOUT
*                             match    when 'opt' is set to OS_OPT_TIME_MATCH
*                             periodic when 'opt' is set to OS_OPT_TIME_PERIODIC
*
*              opt            is an option specifying how to calculate time.  The valid values are:
*              ---
*                                 OS_OPT_TIME_DLY
*                                 OS_OPT_TIME_TIMEOUT
*                                 OS_OPT_TIME_PERIODIC
*                                 OS_OPT_TIME_MATCH
*
*              p_err          is a pointer to a variable that will contain an error code returned by this function.
*              -----
*                                 OS_ERR_NONE           the call was successful and the time delay was scheduled.
*                                 OS_ERR_TIME_ZERO_DLY  if delay is zero or already occurred.
*
* Returns    : None
*
* Note(s)    : 1) This function is INTERNAL to uC/OS-III and your application MUST NOT call it.
*
*              2) This function is assumed to be called with interrupts disabled.
************************************************************************************************************************
*/

void  OS_TickListInsertDly (OS_TCB   *p_tcb,
                            OS_TICK   time,
                            OS_OPT    opt,
                            OS_ERR   *p_err)
{
    OS_TICK   remain;



    if (opt == OS_OPT_TIME_MATCH) {                                     /* MATCH to absolute OSTickCtr value mode            */
        remain = time - OSTickCtr;
        if ((remain > OS_TICK_TH_RDY) ||                                /* If delay already occurred, ...                    */
            (remain == (OS_TICK)0u)) {
            p_tcb->TickRemain = (OS_TICK)0u;
           *p_err             =  OS_ERR_TIME_ZERO_DLY;                  /* ... do NOT delay.                                 */
            return;
        }

    } else if (opt == OS_OPT_TIME_PERIODIC) {                           /* PERIODIC mode.                                    */
        if ((OSTickCtr - p_tcb->TickCtrPrev) > time) {
            remain             = time;                                  /* ... first time we load .TickCtrPrev               */
            p_tcb->TickCtrPrev = OSTickCtr + time;
        } else {
            remain = time - (OSTickCtr - p_tcb->TickCtrPrev);
            if ((remain > OS_TICK_TH_RDY) ||                            /* If delay time has already passed, ...             */
                (remain == (OS_TICK)0u)) {
                p_tcb->TickCtrPrev += time + time * ((OSTickCtr - p_tcb->TickCtrPrev) / time); /* Try to recover the period  */
                p_tcb->TickRemain   = (OS_TICK)0u;
               *p_err               =  OS_ERR_TIME_ZERO_DLY;            /* ... do NOT delay.                                 */
                return;
            }
            p_tcb->TickCtrPrev += time;
        }

    } else if (time > (OS_TICK)0u) {                                    /* RELATIVE time delay mode                          */
        remain = time;

    } else {                                                            /* Zero time delay; ...                              */
        p_tcb->TickRemain = (OS_TICK)0u;
       *p_err             =  OS_ERR_TIME_ZERO_DLY;                      /* ... do NOT delay.                                 */
        return;
    }

    p_tcb->TaskState = OS_TASK_STATE_DLY;
    OS_TickListInsert(&OSTickListDly, p_tcb, remain);

   *p_err = OS_ERR_NONE;
}

/*
************************************************************************************************************************
*                                         REMOVE A TASK FROM THE TICK LIST
*
* Description: This function is called to remove a task from the tick list
*
* Arguments  : p_tcb          Is a pointer to the OS_TCB to remove.
*              -----
*
* Returns    : none
*
* Note(s)    : 1) This function is INTERNAL to uC/OS-III and your application MUST NOT call it.
*
*              2) This function is assumed to be called with interrupts disabled.
************************************************************************************************************************
*/

void  OS_TickListRemove (OS_TCB  *p_tcb)
{
    OS_TICK_LIST  *p_list;
    OS_TCB        *p_tcb1;
    OS_TCB        *p_tcb2;


    p_list = (OS_TICK_LIST *)p_tcb->TickListPtr;
    p_tcb1  = p_tcb->TickPrevPtr;
    p_tcb2  = p_tcb->TickNextPtr;
    if (p_tcb1 == (OS_TCB *)0) {
        if (p_tcb2 == (OS_TCB *)0) {                                    /* Remove ONLY entry in the list?                    */
            p_list->TCB_Ptr    = (OS_TCB        *)0;
#if OS_CFG_DBG_EN > 0u
            p_list->NbrEntries = (OS_OBJ_QTY    )0u;
#endif
            p_tcb->TickRemain   = (OS_TICK       )0u;
            p_tcb->TickListPtr  = (OS_TICK_LIST *)0;
        } else {
            p_tcb2->TickPrevPtr = (OS_TCB       *)0;
            p_tcb2->TickRemain += p_tcb->TickRemain;                    /* Add back the ticks to the delta                   */
            p_list->TCB_Ptr    = p_tcb2;
#if OS_CFG_DBG_EN > 0u
            p_list->NbrEntries--;
#endif
            p_tcb->TickNextPtr  = (OS_TCB       *)0;
            p_tcb->TickRemain   = (OS_TICK       )0u;
            p_tcb->TickListPtr  = (OS_TICK_LIST *)0;
        }
    } else {
        p_tcb1->TickNextPtr = p_tcb2;    
        if (p_tcb2 != (OS_TCB *)0) {
            p_tcb2->TickPrevPtr = p_tcb1;
            p_tcb2->TickRemain += p_tcb->TickRemain;                    /* Add back the ticks to the delta list              */
        }
        p_tcb->TickPrevPtr  = (OS_TCB       *)0;
#if OS_CFG_DBG_EN > 0u
        p_list->NbrEntries--;
#endif
        p_tcb->TickNextPtr  = (OS_TCB       *)0;
        p_tcb->TickRemain   = (OS_TICK       )0u;
        p_tcb->TickListPtr  = (OS_TICK_LIST *)0;
    }
}


/*
************************************************************************************************************************
*                                           UPDATE THE LIST OF TASKS DELAYED
*
* Description: This function updates the delta list which contains tasks that have been delayed.
*
* Arguments  : non
*
* Returns    : none
*
* Note(s)    : 1) This function is INTERNAL to uC/OS-III and your application MUST NOT call it.
************************************************************************************************************************
*/

static  CPU_TS  OS_TickListUpdateDly (void)
{
    OS_TCB       *p_tcb;
    OS_TICK_LIST *p_list;
    CPU_TS        ts_start;
    CPU_TS        ts_delta_dly;
#if OS_CFG_DBG_EN > 0u
    OS_OBJ_QTY    nbr_updated;
#endif
    CPU_SR_ALLOC();

                                                              
                                                                        
    OS_CRITICAL_ENTER();
    ts_start    = OS_TS_GET();
#if OS_CFG_DBG_EN > 0u
    nbr_updated = (OS_OBJ_QTY)0u;
#endif
    p_list      = &OSTickListDly;
    p_tcb       = p_list->TCB_Ptr;                                      
    if (p_tcb != (OS_TCB *)0) {
        p_tcb->TickRemain--;
        while (p_tcb->TickRemain == 0u) {
#if OS_CFG_DBG_EN > 0u
            nbr_updated++;											    /* Keep track of the number of TCBs updated          */
#endif
            if (p_tcb->TaskState == OS_TASK_STATE_DLY) {
                p_tcb->TaskState = OS_TASK_STATE_RDY;
                OS_RdyListInsert(p_tcb);                                /* Insert the task in the ready list                 */
            } else if (p_tcb->TaskState == OS_TASK_STATE_DLY_SUSPENDED) {
                p_tcb->TaskState = OS_TASK_STATE_SUSPENDED;
            }

            p_list->TCB_Ptr = p_tcb->TickNextPtr;
            p_tcb           = p_list->TCB_Ptr;                          /* Get 'p_tcb' again for loop                        */
            if (p_tcb == (OS_TCB *)0) {
#if OS_CFG_DBG_EN > 0u
                p_list->NbrEntries = (OS_OBJ_QTY)0u;
#endif
                break;
            } else {
#if OS_CFG_DBG_EN > 0u
                p_list->NbrEntries--;
#endif
                p_tcb->TickPrevPtr = (OS_TCB *)0;
            }
        }
    }
#if OS_CFG_DBG_EN > 0u
    p_list->NbrUpdated = nbr_updated;
#endif
    ts_delta_dly       = OS_TS_GET() - ts_start;                        /* Measure execution time of the update              */
    OS_CRITICAL_EXIT();

    return (ts_delta_dly);
}


/*
************************************************************************************************************************
*                                       UPDATE THE LIST OF TASKS PENDING WITH TIMEOUT
*
* Description: This function updales the delta list which contains tasks that are pending with a timeout.
*
* Arguments  : non
*
* Returns    : none
*
* Note(s)    : 1) This function is INTERNAL to uC/OS-III and your application MUST NOT call it.
************************************************************************************************************************
*/

static  CPU_TS  OS_TickListUpdateTimeout (void)
{
    OS_TCB       *p_tcb;
    OS_TICK_LIST *p_list;
    CPU_TS        ts_start;
    CPU_TS        ts_delta_timeout;
#if OS_CFG_DBG_EN > 0u
    OS_OBJ_QTY    nbr_updated;
#endif
#if OS_CFG_MUTEX_EN > 0u
    OS_TCB       *p_tcb_owner;
    OS_PRIO       prio_new;
#endif
    CPU_SR_ALLOC();

                                                              
                                                                        
    OS_CRITICAL_ENTER();                                                /* ======= UPDATE TASKS WAITING WITH TIMEOUT ======= */
    ts_start    = OS_TS_GET();
#if OS_CFG_DBG_EN > 0u
    nbr_updated = (OS_OBJ_QTY)0u;
#endif
    p_list      = &OSTickListTimeout;
    p_tcb       = p_list->TCB_Ptr;                                  
    if (p_tcb != (OS_TCB *)0) {
        p_tcb->TickRemain--;
        while (p_tcb->TickRemain == 0u) {
#if OS_CFG_DBG_EN > 0u
            nbr_updated++;
#endif

#if OS_CFG_MUTEX_EN > 0u
            p_tcb_owner = (OS_TCB *)0;
            if (p_tcb->PendOn == OS_TASK_PEND_ON_MUTEX) {
                p_tcb_owner = ((OS_MUTEX *)p_tcb->PendDataTblPtr->PendObjPtr)->OwnerTCBPtr;
            }
#endif

#if (OS_MSG_EN > 0u)
            p_tcb->MsgPtr  = (void      *)0;
            p_tcb->MsgSize = (OS_MSG_SIZE)0u;
#endif
            p_tcb->TS      = OS_TS_GET();
            OS_PendListRemove(p_tcb);                                   /* Remove from wait list                             */
            if (p_tcb->TaskState == OS_TASK_STATE_PEND_TIMEOUT) {
                OS_RdyListInsert(p_tcb);                                /* Insert the task in the ready list                 */
                p_tcb->TaskState  = OS_TASK_STATE_RDY;
            } else if (p_tcb->TaskState == OS_TASK_STATE_PEND_TIMEOUT_SUSPENDED) {

                p_tcb->TaskState  = OS_TASK_STATE_SUSPENDED;
            }
            p_tcb->PendStatus = OS_STATUS_PEND_TIMEOUT;                 /* Indicate pend timed out                           */
            p_tcb->PendOn     = OS_TASK_PEND_ON_NOTHING;                /* Indicate no longer pending                        */

#if OS_CFG_MUTEX_EN > 0u
            if(p_tcb_owner != (OS_TCB *)0) {
                if ((p_tcb_owner->Prio != p_tcb_owner->BasePrio) &&
                    (p_tcb_owner->Prio == p_tcb->Prio)) {               /* Has the owner inherited a priority?               */
                    prio_new = OS_MutexGrpPrioFindHighest(p_tcb_owner);
                    prio_new = prio_new > p_tcb_owner->BasePrio ? p_tcb_owner->BasePrio : prio_new;
                    if(prio_new != p_tcb_owner->Prio) {
                        OS_TaskChangePrio(p_tcb_owner, prio_new);
            #if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
                                      TRACE_OS_MUTEX_TASK_PRIO_DISINHERIT(p_tcb_owner, p_tcb_owner->Prio)
            #endif
                    }
                }
            }
#endif

            p_list->TCB_Ptr = p_tcb->TickNextPtr;
            p_tcb           = p_list->TCB_Ptr;                          /* Get 'p_tcb' again for loop                        */
            if (p_tcb == (OS_TCB *)0) {
#if OS_CFG_DBG_EN > 0u
                p_list->NbrEntries = (OS_OBJ_QTY)0u;
#endif
                break;
            } else {
#if OS_CFG_DBG_EN > 0u
                p_list->NbrEntries--;
#endif
                p_tcb->TickPrevPtr = (OS_TCB *)0;
            }
        }
    }
#if OS_CFG_DBG_EN > 0u
    p_list->NbrUpdated = nbr_updated;
#endif
    ts_delta_timeout   = OS_TS_GET() - ts_start;                        /* Measure execution time of the update              */
    OS_CRITICAL_EXIT();                                                 /* ------------------------------------------------- */

    return (ts_delta_timeout);
}
