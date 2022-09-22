/*
************************************************************************************************************************
*                                                      uC/OS-III
*                                                 The Real-Time Kernel
*
*                                  (c) Copyright 2009-2015; Micrium, Inc.; Weston, FL
*                           All rights reserved.  Protected by international copyright laws.
*
*                                                 ISR QUEUE MANAGEMENT
*
* File    : OS_INT.C
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

#define   MICRIUM_SOURCE
#include "os.h"

#ifdef VSC_INCLUDE_SOURCE_FILE_NAMES
const  CPU_CHAR  *os_int__c = "$Id: $";
#endif


#if OS_CFG_ISR_POST_DEFERRED_EN > 0u
/*
************************************************************************************************************************
*                                                   POST TO ISR QUEUE
*
* Description: This function places contents of posts into an intermediate queue to help defer processing of interrupts
*              at the task level.
*
* Arguments  : type       is the type of kernel object the post is destined to:
*
*                             OS_OBJ_TYPE_SEM
*                             OS_OBJ_TYPE_Q
*                             OS_OBJ_TYPE_FLAG
*                             OS_OBJ_TYPE_TASK_MSG
*                             OS_OBJ_TYPE_TASK_SIGNAL
*
*              p_obj      is a pointer to the kernel object to post to.  This can be a pointer to a semaphore,
*              -----      a message queue or a task control clock.
*
*              p_void     is a pointer to a message that is being posted.  This is used when posting to a message
*                         queue or directly to a task.
*
*              msg_size   is the size of the message being posted
*
*              flags      if the post is done to an event flag group then this corresponds to the flags being
*                         posted
*
*              ts         is a timestamp as to when the post was done
*
*              opt        this corresponds to post options and applies to:
*
*                             OSFlagPost()
*                             OSSemPost()
*                             OSQPost()
*                             OSTaskQPost()
*
*              p_err      is a pointer to a variable that will contain an error code returned by this function.
*
*                             OS_ERR_NONE         if the post to the ISR queue was successful
*                             OS_ERR_INT_Q_FULL   if the ISR queue is full and cannot accepts any further posts.  This
*                                                 generally indicates that you are receiving interrupts faster than you
*                                                 can process them or, that you didn't make the ISR queue large enough.
*
* Returns    : none
*
* Note(s)    : none
************************************************************************************************************************
*/

void  OS_IntQPost (OS_OBJ_TYPE   type,
                   void         *p_obj,
                   void         *p_void,
                   OS_MSG_SIZE   msg_size,
                   OS_FLAGS      flags,
                   OS_OPT        opt,
                   CPU_TS        ts,
                   OS_ERR       *p_err)
{
    CPU_SR_ALLOC();


    CPU_CRITICAL_ENTER();
    if (OSIntQNbrEntries < OSCfg_IntQSize) {                /* Make sure we haven't already filled the ISR queue      */
        OSIntQNbrEntries++;

        if (OSIntQNbrEntriesMax < OSIntQNbrEntries) {
            OSIntQNbrEntriesMax = OSIntQNbrEntries;
        }

        OSIntQInPtr->Type       = type;                     /* Save object type being posted                          */
        OSIntQInPtr->ObjPtr     = p_obj;                    /* Save pointer to object being posted                    */
        OSIntQInPtr->MsgPtr     = p_void;                   /* Save pointer to message if posting to a message queue  */
        OSIntQInPtr->MsgSize    = msg_size;                 /* Save the message size   if posting to a message queue  */
        OSIntQInPtr->Flags      = flags;                    /* Save the flags if posting to an event flag group       */
        OSIntQInPtr->Opt        = opt;                      /* Save post options                                      */
        OSIntQInPtr->TS         = ts;                       /* Save time stamp                                        */

        OSIntQInPtr             =  OSIntQInPtr->NextPtr;    /* Point to the next interrupt handler queue entry        */

        OSRdyList[0].NbrEntries = (OS_OBJ_QTY)1;            /* Make the interrupt handler task ready to run           */
        OSRdyList[0].HeadPtr    = &OSIntQTaskTCB;
        OSRdyList[0].TailPtr    = &OSIntQTaskTCB;
        OS_PrioInsert(0u);                                  /* Add task priority 0 in the priority table              */
        if (OSPrioCur != 0) {                               /* Chk if OSIntQTask is not running                       */
            OSPrioSaved         = OSPrioCur;                /* Save current priority                                  */
        }

       *p_err                   = OS_ERR_NONE;
    } else {
        OSIntQOvfCtr++;                                     /* Count the number of ISR queue overflows                */
       *p_err                   = OS_ERR_INT_Q_FULL;
    }
    CPU_CRITICAL_EXIT();
}


/*
************************************************************************************************************************
*                                               RE-POST FROM ISR QUEUE
*
* Description: This function takes contents of posts from an intermediate queue to help defer processing of interrupts
*              at the task level.
*
* Arguments  : none
*
* Returns    : none
************************************************************************************************************************
*/

void  OS_IntQRePost (void)
{
#if OS_CFG_TMR_EN > 0u
    CPU_TS  ts;
#endif
    OS_ERR  err;


    switch (OSIntQOutPtr->Type) {                           /* Re-post to task                                        */
        case OS_OBJ_TYPE_FLAG:
#if OS_CFG_FLAG_EN > 0u
             (void)OS_FlagPost((OS_FLAG_GRP *) OSIntQOutPtr->ObjPtr,
                               (OS_FLAGS     ) OSIntQOutPtr->Flags,
                               (OS_OPT       ) OSIntQOutPtr->Opt,
                               (CPU_TS       ) OSIntQOutPtr->TS,
                               (OS_ERR      *)&err);
#endif
             break;

        case OS_OBJ_TYPE_Q:
#if OS_CFG_Q_EN > 0u
             OS_QPost((OS_Q      *) OSIntQOutPtr->ObjPtr,
                      (void      *) OSIntQOutPtr->MsgPtr,
                      (OS_MSG_SIZE) OSIntQOutPtr->MsgSize,
                      (OS_OPT     ) OSIntQOutPtr->Opt,
                      (CPU_TS     ) OSIntQOutPtr->TS,
                      (OS_ERR    *)&err);
#endif
             break;

        case OS_OBJ_TYPE_SEM:
#if OS_CFG_SEM_EN > 0u
             (void)OS_SemPost((OS_SEM *) OSIntQOutPtr->ObjPtr,
                              (OS_OPT  ) OSIntQOutPtr->Opt,
                              (CPU_TS  ) OSIntQOutPtr->TS,
                              (OS_ERR *)&err);
#endif
             break;

        case OS_OBJ_TYPE_TASK_MSG:
#if OS_CFG_TASK_Q_EN > 0u
             OS_TaskQPost((OS_TCB    *) OSIntQOutPtr->ObjPtr,
                          (void      *) OSIntQOutPtr->MsgPtr,
                          (OS_MSG_SIZE) OSIntQOutPtr->MsgSize,
                          (OS_OPT     ) OSIntQOutPtr->Opt,
                          (CPU_TS     ) OSIntQOutPtr->TS,
                          (OS_ERR    *)&err);
#endif
             break;

        case OS_OBJ_TYPE_TASK_RESUME:
#if OS_CFG_TASK_SUSPEND_EN > 0u
             (void)OS_TaskResume((OS_TCB *) OSIntQOutPtr->ObjPtr,
                                 (OS_ERR *)&err);
#endif
             break;

        case OS_OBJ_TYPE_TASK_SIGNAL:
             (void)OS_TaskSemPost((OS_TCB *) OSIntQOutPtr->ObjPtr,
                                  (OS_OPT  ) OSIntQOutPtr->Opt,
                                  (CPU_TS  ) OSIntQOutPtr->TS,
                                  (OS_ERR *)&err);
             break;

        case OS_OBJ_TYPE_TASK_SUSPEND:
#if OS_CFG_TASK_SUSPEND_EN > 0u
             (void)OS_TaskSuspend((OS_TCB *) OSIntQOutPtr->ObjPtr,
                                  (OS_ERR *)&err);
#endif
             break;

        case OS_OBJ_TYPE_TICK:
#if OS_CFG_SCHED_ROUND_ROBIN_EN > 0u
             OS_SchedRoundRobin(&OSRdyList[OSPrioSaved]);
#endif

             (void)OS_TaskSemPost((OS_TCB *)&OSTickTaskTCB,                /* Signal tick task                        */
                                  (OS_OPT  ) OS_OPT_POST_NONE,
                                  (CPU_TS  ) OSIntQOutPtr->TS,
                                  (OS_ERR *)&err);
#if OS_CFG_TMR_EN > 0u
             OSTmrUpdateCtr--;
             if (OSTmrUpdateCtr == (OS_CTR)0u) {
                 OSTmrUpdateCtr = OSTmrUpdateCnt;
                 ts             = OS_TS_GET();                             /* Get timestamp                           */
                 (void)OS_TaskSemPost((OS_TCB *)&OSTmrTaskTCB,             /* Signal timer task                       */
                                      (OS_OPT  ) OS_OPT_POST_NONE,
                                      (CPU_TS  ) ts,
                                      (OS_ERR *)&err);
             }
#endif
             break;

        default:
             break;
    }
}


/*
************************************************************************************************************************
*                                               INTERRUPT QUEUE MANAGEMENT TASK
*
* Description: This task is internal to uC/OS-III and is used to process the queue of deffered interrupts.
*
* Arguments  : p_arg     is a pointer to an optional argument that is passed during task creation.  For this function
*                        the argument is not used and will be a NULL pointer.
*
* Returns    : none
************************************************************************************************************************
*/

void  OS_IntQTask (void  *p_arg)
{
    CPU_BOOLEAN  done;
    CPU_TS       ts_start;
    CPU_TS       ts_end;
    CPU_SR_ALLOC();



    (void)&p_arg;                                           /* Not using 'p_arg', prevent compiler warning            */
    while (DEF_ON) {
        done = DEF_FALSE;
        while (done == DEF_FALSE) {
            CPU_CRITICAL_ENTER();
            if (OSIntQNbrEntries == (OS_OBJ_QTY)0u) {
                OSRdyList[0].NbrEntries = (OS_OBJ_QTY)0u;   /* Remove from ready list                                 */
                OSRdyList[0].HeadPtr    = (OS_TCB   *)0;
                OSRdyList[0].TailPtr    = (OS_TCB   *)0;
                OS_PrioRemove(0u);                          /* Remove from the priority table                         */
                CPU_CRITICAL_EXIT();
                OSSched();
                done = DEF_TRUE;                            /* No more entries in the queue, we are done              */
            } else {
                CPU_CRITICAL_EXIT();
                ts_start = OS_TS_GET();
                OS_IntQRePost();
                ts_end   = OS_TS_GET() - ts_start;          /* Measure execution time of tick task                    */
                if (OSIntQTaskTimeMax < ts_end) {
                    OSIntQTaskTimeMax = ts_end;
                }
                CPU_CRITICAL_ENTER();
                OSIntQOutPtr = OSIntQOutPtr->NextPtr;       /* Point to next item in the ISR queue                    */
                OSIntQNbrEntries--;
                CPU_CRITICAL_EXIT();
            }
        }
    }
}


/*
************************************************************************************************************************
*                                                 INITIALIZE THE ISR QUEUE
*
* Description: This function is called by OSInit() to initialize the ISR queue.
*
* Arguments  : p_err    is a pointer to a variable that will contain an error code returned by this function.
*
*                           OS_ERR_INT_Q             If you didn't provide an ISR queue in OS_CFG.C
*                           OS_ERR_INT_Q_SIZE        If you didn't specify a large enough ISR queue.
*                           OS_ERR_STK_INVALID       If you specified a NULL pointer for the task of the ISR task
*                                                    handler
*                           OS_ERR_STK_SIZE_INVALID  If you didn't specify a stack size greater than the minimum
*                                                    specified by OS_CFG_STK_SIZE_MIN
*                           OS_ERR_???               An error code returned by OSTaskCreate().
*
* Returns    : none
*
* Note(s)    : none
************************************************************************************************************************
*/

void  OS_IntQTaskInit (OS_ERR  *p_err)
{
    OS_INT_Q      *p_int_q;
    OS_INT_Q      *p_int_q_next;
    OS_OBJ_QTY     i;


    OSIntQOvfCtr = (OS_QTY)0u;                              /* Clear the ISR queue overflow counter                   */

    if (OSCfg_IntQBasePtr == (OS_INT_Q *)0) {
       *p_err = OS_ERR_INT_Q;
        return;
    }

    if (OSCfg_IntQSize < (OS_OBJ_QTY)2u) {
       *p_err = OS_ERR_INT_Q_SIZE;
        return;
    }

    OSIntQTaskTimeMax = (CPU_TS)0;

    p_int_q           = OSCfg_IntQBasePtr;                  /* Initialize the circular ISR queue                      */
    p_int_q_next      = p_int_q;
    p_int_q_next++;
    for (i = 0u; i < OSCfg_IntQSize; i++) {
        p_int_q->Type    =  OS_OBJ_TYPE_NONE;
        p_int_q->ObjPtr  = (void      *)0;
        p_int_q->MsgPtr  = (void      *)0;
        p_int_q->MsgSize = (OS_MSG_SIZE)0u;
        p_int_q->Flags   = (OS_FLAGS   )0u;
        p_int_q->Opt     = (OS_OPT     )0u;
        p_int_q->NextPtr = p_int_q_next;
        p_int_q++;
        p_int_q_next++;
    }
    p_int_q--;
    p_int_q_next        = OSCfg_IntQBasePtr;
    p_int_q->NextPtr    = p_int_q_next;
    OSIntQInPtr         = p_int_q_next;
    OSIntQOutPtr        = p_int_q_next;
    OSIntQNbrEntries    = (OS_OBJ_QTY)0u;
    OSIntQNbrEntriesMax = (OS_OBJ_QTY)0u;

                                                            /* -------------- CREATE THE ISR QUEUE TASK ------------- */
    if (OSCfg_IntQTaskStkBasePtr == (CPU_STK *)0) {
       *p_err = OS_ERR_INT_Q_STK_INVALID;
        return;
    }

    if (OSCfg_IntQTaskStkSize < OSCfg_StkSizeMin) {
       *p_err = OS_ERR_INT_Q_STK_SIZE_INVALID;
        return;
    }

    OSTaskCreate((OS_TCB     *)&OSIntQTaskTCB,
                 (CPU_CHAR   *)((void *)"uC/OS-III ISR Queue Task"),
                 (OS_TASK_PTR )OS_IntQTask,
                 (void       *)0,
                 (OS_PRIO     )0u,                          /* This task is ALWAYS at priority '0' (i.e. highest)     */
                 (CPU_STK    *)OSCfg_IntQTaskStkBasePtr,
                 (CPU_STK_SIZE)OSCfg_IntQTaskStkLimit,
                 (CPU_STK_SIZE)OSCfg_IntQTaskStkSize,
                 (OS_MSG_QTY  )0u,
                 (OS_TICK     )0u,
                 (void       *)0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)p_err);
}

#endif
