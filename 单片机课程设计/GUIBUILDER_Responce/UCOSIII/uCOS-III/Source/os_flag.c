/*
************************************************************************************************************************
*                                                      uC/OS-III
*                                                 The Real-Time Kernel
*
*                                  (c) Copyright 2009-2015; Micrium, Inc.; Weston, FL
*                           All rights reserved.  Protected by international copyright laws.
*
*                                                EVENT FLAG MANAGEMENT
*
* File    : OS_FLAG.C
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
const  CPU_CHAR  *os_flag__c = "$Id: $";
#endif


#if OS_CFG_FLAG_EN > 0u

/*
************************************************************************************************************************
*                                                 CREATE AN EVENT FLAG
*
* Description: This function is called to create an event flag group.
*
* Arguments  : p_grp          is a pointer to the event flag group to create
*
*              p_name         is the name of the event flag group
*
*              flags          contains the initial value to store in the event flag group (typically 0).
*
*              p_err          is a pointer to an error code which will be returned to your application:
*
*                                 OS_ERR_NONE                    if the call was successful.
*                                 OS_ERR_CREATE_ISR              if you attempted to create an Event Flag from an ISR.
*                                 OS_ERR_ILLEGAL_CREATE_RUN_TIME if you are trying to create the Event Flag after you
*                                                                   called OSSafetyCriticalStart().
*                                 OS_ERR_NAME                    if 'p_name' is a NULL pointer
*                                 OS_ERR_OBJ_CREATED             if the event flag group has already been created
*                                 OS_ERR_OBJ_PTR_NULL            if 'p_grp' is a NULL pointer
*
* Returns    : none
************************************************************************************************************************
*/

void  OSFlagCreate (OS_FLAG_GRP  *p_grp,
                    CPU_CHAR     *p_name,
                    OS_FLAGS      flags,
                    OS_ERR       *p_err)
{
    CPU_SR_ALLOC();



#ifdef OS_SAFETY_CRITICAL
    if (p_err == (OS_ERR *)0) {
        OS_SAFETY_CRITICAL_EXCEPTION();
        return;
    }
#endif

#ifdef OS_SAFETY_CRITICAL_IEC61508
    if (OSSafetyCriticalStartFlag == DEF_TRUE) {
       *p_err = OS_ERR_ILLEGAL_CREATE_RUN_TIME;
        return;
    }
#endif

#if OS_CFG_CALLED_FROM_ISR_CHK_EN > 0u
    if (OSIntNestingCtr > (OS_NESTING_CTR)0) {              /* See if called from ISR ...                             */
       *p_err = OS_ERR_CREATE_ISR;                          /* ... can't CREATE from an ISR                           */
        return;
    }
#endif

#if OS_CFG_ARG_CHK_EN > 0u
    if (p_grp == (OS_FLAG_GRP *)0) {                        /* Validate 'p_grp'                                       */
       *p_err = OS_ERR_OBJ_PTR_NULL;
        return;
    }
#endif

    OS_CRITICAL_ENTER();
#if OS_OBJ_TYPE_REQ > 0u
    p_grp->Type    = OS_OBJ_TYPE_FLAG;                      /* Set to event flag group type                           */
#endif
#if OS_CFG_DBG_EN > 0u
    p_grp->NamePtr = p_name;
#else
    (void)&p_name;
#endif
    p_grp->Flags   = flags;                                 /* Set to desired initial value                           */
    p_grp->TS      = (CPU_TS)0;
    OS_PendListInit(&p_grp->PendList);

#if OS_CFG_DBG_EN > 0u
    OS_FlagDbgListAdd(p_grp);
#endif
    OSFlagQty++;

#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
    TRACE_OS_FLAG_CREATE(p_grp, p_name);                    /* Record the event.                                      */
#endif
    
    OS_CRITICAL_EXIT_NO_SCHED();
   *p_err = OS_ERR_NONE;
}


/*
************************************************************************************************************************
*                                             DELETE AN EVENT FLAG GROUP
*
* Description: This function deletes an event flag group and readies all tasks pending on the event flag group.
*
* Arguments  : p_grp     is a pointer to the desired event flag group.
*
*              opt       determines delete options as follows:
*
*                            OS_OPT_DEL_NO_PEND           Deletes the event flag group ONLY if no task pending
*                            OS_OPT_DEL_ALWAYS            Deletes the event flag group even if tasks are waiting.
*                                                         In this case, all the tasks pending will be readied.
*
*              p_err     is a pointer to an error code that can contain one of the following values:
*
*                            OS_ERR_NONE                  The call was successful and the event flag group was deleted
*                            OS_ERR_DEL_ISR               If you attempted to delete the event flag group from an ISR
*                            OS_ERR_OBJ_PTR_NULL          If 'p_grp' is a NULL pointer.
*                            OS_ERR_OBJ_TYPE              If you didn't pass a pointer to an event flag group
*                            OS_ERR_OPT_INVALID           An invalid option was specified
*                            OS_ERR_TASK_WAITING          One or more tasks were waiting on the event flag group.
*
* Returns    : == 0          if no tasks were waiting on the event flag group, or upon error.
*              >  0          if one or more tasks waiting on the event flag group are now readied and informed.
*
* Note(s)    : 1) This function must be used with care.  Tasks that would normally expect the presence of the event flag
*                 group MUST check the return code of OSFlagPost and OSFlagPend().
************************************************************************************************************************
*/

#if OS_CFG_FLAG_DEL_EN > 0u
OS_OBJ_QTY  OSFlagDel (OS_FLAG_GRP  *p_grp,
                       OS_OPT        opt,
                       OS_ERR       *p_err)
{
    OS_OBJ_QTY        cnt;
    OS_OBJ_QTY        nbr_tasks;
    OS_PEND_DATA     *p_pend_data;
    OS_PEND_LIST     *p_pend_list;
    OS_TCB           *p_tcb;
    CPU_TS            ts;
    CPU_SR_ALLOC();



#ifdef OS_SAFETY_CRITICAL
    if (p_err == (OS_ERR *)0) {
        OS_SAFETY_CRITICAL_EXCEPTION();
        return ((OS_OBJ_QTY)0);
    }
#endif

#if OS_CFG_CALLED_FROM_ISR_CHK_EN > 0u
    if (OSIntNestingCtr > (OS_NESTING_CTR)0) {              /* See if called from ISR ...                             */
       *p_err = OS_ERR_DEL_ISR;                             /* ... can't DELETE from an ISR                           */
        return ((OS_OBJ_QTY)0);
    }
#endif

#if OS_CFG_ARG_CHK_EN > 0u
    if (p_grp == (OS_FLAG_GRP *)0) {                        /* Validate 'p_grp'                                       */
       *p_err  = OS_ERR_OBJ_PTR_NULL;
        return ((OS_OBJ_QTY)0);
    }
    switch (opt) {                                          /* Validate 'opt'                                         */
        case OS_OPT_DEL_NO_PEND:
        case OS_OPT_DEL_ALWAYS:
             break;

        default:
            *p_err = OS_ERR_OPT_INVALID;
             return ((OS_OBJ_QTY)0);
    }
#endif

#if OS_CFG_OBJ_TYPE_CHK_EN > 0u
    if (p_grp->Type != OS_OBJ_TYPE_FLAG) {                  /* Validate event group object                            */
       *p_err = OS_ERR_OBJ_TYPE;
        return ((OS_OBJ_QTY)0);
    }
#endif
    OS_CRITICAL_ENTER();
    p_pend_list = &p_grp->PendList;
    cnt         = p_pend_list->NbrEntries;
    nbr_tasks   = cnt;
    switch (opt) {
        case OS_OPT_DEL_NO_PEND:                            /* Delete group if no task waiting                        */
             if (nbr_tasks == (OS_OBJ_QTY)0) {
#if OS_CFG_DBG_EN > 0u
                 OS_FlagDbgListRemove(p_grp);
#endif
                 OSFlagQty--;
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
                 TRACE_OS_FLAG_DEL(p_grp);                  /* Record the event.                                      */
#endif
                 OS_FlagClr(p_grp);

                 OS_CRITICAL_EXIT();

                *p_err = OS_ERR_NONE;
             } else {
                 OS_CRITICAL_EXIT();
                *p_err = OS_ERR_TASK_WAITING;
             }
             break;

        case OS_OPT_DEL_ALWAYS:                             /* Always delete the event flag group                     */
             ts = OS_TS_GET();                              /* Get local time stamp so all tasks get the same time    */
             while (cnt > 0u) {                             /* Remove all tasks from the pend list                    */
                 p_pend_data = p_pend_list->HeadPtr;
                 p_tcb       = p_pend_data->TCBPtr;
                 OS_PendObjDel((OS_PEND_OBJ *)((void *)p_grp),
                               p_tcb,
                               ts);
                 cnt--;
             }
#if OS_CFG_DBG_EN > 0u
             OS_FlagDbgListRemove(p_grp);
#endif
             OSFlagQty--;
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
             TRACE_OS_FLAG_DEL(p_grp);                      /* Record the event.                                      */
#endif
             OS_FlagClr(p_grp);
             OS_CRITICAL_EXIT_NO_SCHED();

             OSSched();                                     /* Find highest priority task ready to run                */
            *p_err = OS_ERR_NONE;
             break;

        default:
             OS_CRITICAL_EXIT();
            *p_err = OS_ERR_OPT_INVALID;
             break;
    }
    return (nbr_tasks);
}
#endif

/*
************************************************************************************************************************
*                                             WAIT ON AN EVENT FLAG GROUP
*
* Description: This function is called to wait for a combination of bits to be set in an event flag group.  Your
*              application can wait for ANY bit to be set or ALL bits to be set.
*
* Arguments  : p_grp         is a pointer to the desired event flag group.
*
*              flags         Is a bit pattern indicating which bit(s) (i.e. flags) you wish to wait for.
*                            The bits you want are specified by setting the corresponding bits in 'flags'.
*                            e.g. if your application wants to wait for bits 0 and 1 then 'flags' would contain 0x03.
*
*              timeout       is an optional timeout (in clock ticks) that your task will wait for the
*                            desired bit combination.  If you specify 0, however, your task will wait
*                            forever at the specified event flag group or, until a message arrives.
*
*              opt           specifies whether you want ALL bits to be set or ANY of the bits to be set.
*                            You can specify the 'ONE' of the following arguments:
*
*                                OS_OPT_PEND_FLAG_CLR_ALL   You will wait for ALL bits in 'flags' to be clear (0)
*                                OS_OPT_PEND_FLAG_CLR_ANY   You will wait for ANY bit  in 'flags' to be clear (0)
*                                OS_OPT_PEND_FLAG_SET_ALL   You will wait for ALL bits in 'flags' to be set   (1)
*                                OS_OPT_PEND_FLAG_SET_ANY   You will wait for ANY bit  in 'flags' to be set   (1)
*
*                            You can 'ADD' OS_OPT_PEND_FLAG_CONSUME if you want the event flag to be 'consumed' by
*                                      the call.  Example, to wait for any flag in a group AND then clear
*                                      the flags that are present, set 'wait_opt' to:
*
*                                      OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME
*
*                            You can also 'ADD' the type of pend with 'ONE' of the two option:
*
*                                OS_OPT_PEND_NON_BLOCKING   Task will NOT block if flags are not available
*                                OS_OPT_PEND_BLOCKING       Task will     block if flags are not available
*
*              p_ts          is a pointer to a variable that will receive the timestamp of when the event flag group was
*                            posted, aborted or the event flag group deleted.  If you pass a NULL pointer (i.e. (CPU_TS *)0)
*                            then you will not get the timestamp.  In other words, passing a NULL pointer is valid and
*                            indicates that you don't need the timestamp.
*
*              p_err         is a pointer to an error code and can be:
*
*                                OS_ERR_NONE                The desired bits have been set within the specified 'timeout'
*                                OS_ERR_OBJ_PTR_NULL        If 'p_grp' is a NULL pointer.
*                                OS_ERR_OBJ_TYPE            You are not pointing to an event flag group
*                                OS_ERR_OPT_INVALID         You didn't specify a proper 'opt' argument.
*                                OS_ERR_PEND_ABORT          The wait on the flag was aborted.
*                                OS_ERR_PEND_ISR            If you tried to PEND from an ISR
*                                OS_ERR_PEND_WOULD_BLOCK    If you specified non-blocking but the flags were not
*                                                           available.
*                                OS_ERR_SCHED_LOCKED        If you called this function when the scheduler is locked
*                                OS_ERR_TIMEOUT             The bit(s) have not been set in the specified 'timeout'.
*
* Returns    : The flags in the event flag group that made the task ready or, 0 if a timeout or an error
*              occurred.
************************************************************************************************************************
*/

OS_FLAGS  OSFlagPend (OS_FLAG_GRP  *p_grp,
                      OS_FLAGS      flags,
                      OS_TICK       timeout,
                      OS_OPT        opt,
                      CPU_TS       *p_ts,
                      OS_ERR       *p_err)
{
    CPU_BOOLEAN   consume;
    OS_FLAGS      flags_rdy;
    OS_OPT        mode;
    OS_PEND_DATA  pend_data;
    CPU_SR_ALLOC();



#ifdef OS_SAFETY_CRITICAL
    if (p_err == (OS_ERR *)0) {
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
        TRACE_OS_FLAG_PEND_FAILED(p_grp);                   /* Record the event.                                      */
#endif
        OS_SAFETY_CRITICAL_EXCEPTION();
        return ((OS_FLAGS)0);
    }
#endif

#if OS_CFG_CALLED_FROM_ISR_CHK_EN > 0u
    if (OSIntNestingCtr > (OS_NESTING_CTR)0) {              /* See if called from ISR ...                             */
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
        TRACE_OS_FLAG_PEND_FAILED(p_grp);                   /* Record the event.                                      */
#endif
       *p_err = OS_ERR_PEND_ISR;                            /* ... can't PEND from an ISR                             */
        return ((OS_FLAGS)0);
    }
#endif

#if OS_CFG_ARG_CHK_EN > 0u
    if (p_grp == (OS_FLAG_GRP *)0) {                        /* Validate 'p_grp'                                       */
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
        TRACE_OS_FLAG_PEND_FAILED(p_grp);                   /* Record the event.                                      */
#endif
       *p_err = OS_ERR_OBJ_PTR_NULL;
        return ((OS_FLAGS)0);
    }
    switch (opt) {                                          /* Validate 'opt'                                         */
        case OS_OPT_PEND_FLAG_CLR_ALL:
        case OS_OPT_PEND_FLAG_CLR_ANY:
        case OS_OPT_PEND_FLAG_SET_ALL:
        case OS_OPT_PEND_FLAG_SET_ANY:
        case OS_OPT_PEND_FLAG_CLR_ALL | OS_OPT_PEND_FLAG_CONSUME:
        case OS_OPT_PEND_FLAG_CLR_ANY | OS_OPT_PEND_FLAG_CONSUME:
        case OS_OPT_PEND_FLAG_SET_ALL | OS_OPT_PEND_FLAG_CONSUME:
        case OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME:
        case OS_OPT_PEND_FLAG_CLR_ALL | OS_OPT_PEND_NON_BLOCKING:
        case OS_OPT_PEND_FLAG_CLR_ANY | OS_OPT_PEND_NON_BLOCKING:
        case OS_OPT_PEND_FLAG_SET_ALL | OS_OPT_PEND_NON_BLOCKING:
        case OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_NON_BLOCKING:
        case OS_OPT_PEND_FLAG_CLR_ALL | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_NON_BLOCKING:
        case OS_OPT_PEND_FLAG_CLR_ANY | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_NON_BLOCKING:
        case OS_OPT_PEND_FLAG_SET_ALL | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_NON_BLOCKING:
        case OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_NON_BLOCKING:
             break;

        default:
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
             TRACE_OS_FLAG_PEND_FAILED(p_grp);              /* Record the event.                                      */
#endif
            *p_err = OS_ERR_OPT_INVALID;
             return ((OS_OBJ_QTY)0);
    }
#endif

#if OS_CFG_OBJ_TYPE_CHK_EN > 0u
    if (p_grp->Type != OS_OBJ_TYPE_FLAG) {                  /* Validate that we are pointing at an event flag         */
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
        TRACE_OS_FLAG_PEND_FAILED(p_grp);                   /* Record the event.                                      */
#endif
       *p_err = OS_ERR_OBJ_TYPE;
        return ((OS_FLAGS)0);
    }
#endif

    if ((opt & OS_OPT_PEND_FLAG_CONSUME) != (OS_OPT)0) {    /* See if we need to consume the flags                    */
        consume = DEF_TRUE;
    } else {
        consume = DEF_FALSE;
    }

    if (p_ts != (CPU_TS *)0) {
       *p_ts = (CPU_TS)0;                                   /* Initialize the returned timestamp                      */
    }

    mode = opt & OS_OPT_PEND_FLAG_MASK;
    CPU_CRITICAL_ENTER();
    switch (mode) {
        case OS_OPT_PEND_FLAG_SET_ALL:                      /* See if all required flags are set                      */
             flags_rdy = (OS_FLAGS)(p_grp->Flags & flags);  /* Extract only the bits we want                          */
             if (flags_rdy == flags) {                      /* Must match ALL the bits that we want                   */
                 if (consume == DEF_TRUE) {                 /* See if we need to consume the flags                    */
                     p_grp->Flags &= ~flags_rdy;            /* Clear ONLY the flags that we wanted                    */
                 }
                 OSTCBCurPtr->FlagsRdy = flags_rdy;         /* Save flags that were ready                             */
                 if (p_ts != (CPU_TS *)0) {
                    *p_ts  = p_grp->TS;
                 }
                 CPU_CRITICAL_EXIT();                       /* Yes, condition met, return to caller                   */
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
                 TRACE_OS_FLAG_PEND(p_grp);                 /* Record the event.                                      */
#endif
                *p_err = OS_ERR_NONE;
                 return (flags_rdy);
             } else {                                       /* Block task until events occur or timeout               */
                 if ((opt & OS_OPT_PEND_NON_BLOCKING) != (OS_OPT)0) {
                     CPU_CRITICAL_EXIT();
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
                     TRACE_OS_FLAG_PEND_FAILED(p_grp);      /* Record the event.                                      */
#endif
                    *p_err = OS_ERR_PEND_WOULD_BLOCK;       /* Specified non-blocking so task would block             */
                     return ((OS_FLAGS)0);
                 } else {                                   /* Specified blocking so check is scheduler is locked     */
                     if (OSSchedLockNestingCtr > (OS_NESTING_CTR)0) { /* See if called with scheduler locked ...      */
                         CPU_CRITICAL_EXIT();
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
                         TRACE_OS_FLAG_PEND_FAILED(p_grp);  /* Record the event.                                      */
#endif
                        *p_err = OS_ERR_SCHED_LOCKED;       /* ... can't PEND when locked                             */
                         return ((OS_FLAGS)0);
                     }
                 }
                                                            /* Lock the scheduler/re-enable interrupts                */
                 OS_CRITICAL_ENTER_CPU_EXIT();              
                 OS_FlagBlock(&pend_data,
                              p_grp,
                              flags,
                              opt,
                              timeout);
                 OS_CRITICAL_EXIT_NO_SCHED();
             }
             break;

        case OS_OPT_PEND_FLAG_SET_ANY:
             flags_rdy = (OS_FLAGS)(p_grp->Flags & flags);  /* Extract only the bits we want                          */
             if (flags_rdy != (OS_FLAGS)0) {                /* See if any flag set                                    */
                 if (consume == DEF_TRUE) {                 /* See if we need to consume the flags                    */
                     p_grp->Flags &= ~flags_rdy;            /* Clear ONLY the flags that we got                       */
                 }
                 OSTCBCurPtr->FlagsRdy = flags_rdy;         /* Save flags that were ready                             */
                 if (p_ts != (CPU_TS *)0) {
                    *p_ts  = p_grp->TS;
                 }
                 CPU_CRITICAL_EXIT();                       /* Yes, condition met, return to caller                   */
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
                 TRACE_OS_FLAG_PEND(p_grp);                 /* Record the event.                                      */
#endif
                *p_err = OS_ERR_NONE;
                 return (flags_rdy);
             } else {                                       /* Block task until events occur or timeout               */
                 if ((opt & OS_OPT_PEND_NON_BLOCKING) != (OS_OPT)0) {
                     CPU_CRITICAL_EXIT();
                    *p_err = OS_ERR_PEND_WOULD_BLOCK;       /* Specified non-blocking so task would block             */
                     return ((OS_FLAGS)0);
                 } else {                                   /* Specified blocking so check is scheduler is locked     */
                     if (OSSchedLockNestingCtr > (OS_NESTING_CTR)0) { /* See if called with scheduler locked ...      */
                         CPU_CRITICAL_EXIT();
                        *p_err = OS_ERR_SCHED_LOCKED;                 /* ... can't PEND when locked                   */
                         return ((OS_FLAGS)0);
                     }
                 }
                                                            /* Lock the scheduler/re-enable interrupts                */
                 OS_CRITICAL_ENTER_CPU_EXIT();              
                 OS_FlagBlock(&pend_data,
                              p_grp,
                              flags,
                              opt,
                              timeout);
                 OS_CRITICAL_EXIT_NO_SCHED();
             }
             break;

#if OS_CFG_FLAG_MODE_CLR_EN > 0u
        case OS_OPT_PEND_FLAG_CLR_ALL:                      /* See if all required flags are cleared                  */
             flags_rdy = (OS_FLAGS)(~p_grp->Flags & flags); /* Extract only the bits we want                          */
             if (flags_rdy == flags) {                      /* Must match ALL the bits that we want                   */
                 if (consume == DEF_TRUE) {                 /* See if we need to consume the flags                    */
                     p_grp->Flags |= flags_rdy;             /* Set ONLY the flags that we wanted                      */
                 }
                 OSTCBCurPtr->FlagsRdy = flags_rdy;         /* Save flags that were ready                             */
                 if (p_ts != (CPU_TS *)0) {
                    *p_ts  = p_grp->TS;
                 }
                 CPU_CRITICAL_EXIT();                       /* Yes, condition met, return to caller                   */
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
                 TRACE_OS_FLAG_PEND(p_grp);                 /* Record the event.                                      */
#endif                 
                *p_err = OS_ERR_NONE;
                 return (flags_rdy);
             } else {                                       /* Block task until events occur or timeout               */
                 if ((opt & OS_OPT_PEND_NON_BLOCKING) != (OS_OPT)0) {
                     CPU_CRITICAL_EXIT();
                    *p_err = OS_ERR_PEND_WOULD_BLOCK;       /* Specified non-blocking so task would block             */
                     return ((OS_FLAGS)0);
                 } else {                                   /* Specified blocking so check is scheduler is locked     */
                     if (OSSchedLockNestingCtr > (OS_NESTING_CTR)0) { /* See if called with scheduler locked ...      */
                         CPU_CRITICAL_EXIT();
                        *p_err = OS_ERR_SCHED_LOCKED;                 /* ... can't PEND when locked                   */
                         return ((OS_FLAGS)0);
                     }
                 }
                                                            
                 OS_CRITICAL_ENTER_CPU_EXIT();              /* Lock the scheduler/re-enable interrupts                */
                 OS_FlagBlock(&pend_data,
                              p_grp,
                              flags,
                              opt,
                              timeout);
                 OS_CRITICAL_EXIT_NO_SCHED();
             }
             break;

        case OS_OPT_PEND_FLAG_CLR_ANY:
             flags_rdy = (OS_FLAGS)(~p_grp->Flags & flags); /* Extract only the bits we want                          */
             if (flags_rdy != (OS_FLAGS)0) {                /* See if any flag cleared                                */
                 if (consume == DEF_TRUE) {                 /* See if we need to consume the flags                    */
                     p_grp->Flags |= flags_rdy;             /* Set ONLY the flags that we got                         */
                 }
                 OSTCBCurPtr->FlagsRdy = flags_rdy;         /* Save flags that were ready                             */
                 if (p_ts != (CPU_TS *)0) {
                    *p_ts  = p_grp->TS;
                 }
                 CPU_CRITICAL_EXIT();                       /* Yes, condition met, return to caller                   */
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
                 TRACE_OS_FLAG_PEND(p_grp);                 /* Record the event.                                      */
#endif
                *p_err = OS_ERR_NONE;
                 return (flags_rdy);
             } else {                                       /* Block task until events occur or timeout               */
                 if ((opt & OS_OPT_PEND_NON_BLOCKING) != (OS_OPT)0) {
                     CPU_CRITICAL_EXIT();
                    *p_err = OS_ERR_PEND_WOULD_BLOCK;       /* Specified non-blocking so task would block             */
                     return ((OS_FLAGS)0);
                 } else {                                   /* Specified blocking so check is scheduler is locked     */
                     if (OSSchedLockNestingCtr > (OS_NESTING_CTR)0) { /* See if called with scheduler locked ...      */
                         CPU_CRITICAL_EXIT();
                        *p_err = OS_ERR_SCHED_LOCKED;                 /* ... can't PEND when locked                   */
                         return ((OS_FLAGS)0);
                     }
                 }
                                                            
                 OS_CRITICAL_ENTER_CPU_EXIT();              /* Lock the scheduler/re-enable interrupts                */
                 OS_FlagBlock(&pend_data,              
                              p_grp,
                              flags,
                              opt,
                              timeout);
                 OS_CRITICAL_EXIT_NO_SCHED();
             }
             break;
#endif

        default:
             CPU_CRITICAL_EXIT();
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
             TRACE_OS_FLAG_PEND_FAILED(p_grp);              /* Record the event.                                      */
#endif
            *p_err = OS_ERR_OPT_INVALID;
             return ((OS_FLAGS)0);
    }
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
    TRACE_OS_FLAG_PEND_BLOCK(p_grp);                        /* Record the event.                                      */
#endif
    OSSched();                                              /* Find next HPT ready to run                             */

    CPU_CRITICAL_ENTER();
    switch (OSTCBCurPtr->PendStatus) {
        case OS_STATUS_PEND_OK:                             /* We got the event flags                                 */
             if (p_ts != (CPU_TS *)0) {
                *p_ts  = OSTCBCurPtr->TS;
             }
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
             TRACE_OS_FLAG_PEND(p_grp);                     /* Record the event.                                      */
#endif
            *p_err = OS_ERR_NONE;
             break;

        case OS_STATUS_PEND_ABORT:                          /* Indicate that we aborted                               */
             if (p_ts != (CPU_TS *)0) {
                *p_ts  = OSTCBCurPtr->TS;
             }
             CPU_CRITICAL_EXIT();
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
             TRACE_OS_FLAG_PEND_FAILED(p_grp);              /* Record the event.                                      */
#endif
            *p_err = OS_ERR_PEND_ABORT;
             break;

        case OS_STATUS_PEND_TIMEOUT:                        /* Indicate that we didn't get semaphore within timeout   */
             if (p_ts != (CPU_TS *)0) {
                *p_ts  = (CPU_TS  )0;
             }
             CPU_CRITICAL_EXIT();
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
             TRACE_OS_FLAG_PEND_FAILED(p_grp);              /* Record the event.                                      */
#endif
            *p_err = OS_ERR_TIMEOUT;
             break;

        case OS_STATUS_PEND_DEL:                            /* Indicate that object pended on has been deleted        */
             if (p_ts != (CPU_TS *)0) {
                *p_ts  = OSTCBCurPtr->TS;
             }
             CPU_CRITICAL_EXIT();
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
             TRACE_OS_FLAG_PEND_FAILED(p_grp);              /* Record the event.                                      */
#endif
            *p_err = OS_ERR_OBJ_DEL;
             break;

        default:
             CPU_CRITICAL_EXIT();
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
             TRACE_OS_FLAG_PEND_FAILED(p_grp);              /* Record the event.                                      */
#endif
            *p_err = OS_ERR_STATUS_INVALID;
             break;
    }
    if (*p_err != OS_ERR_NONE) {
        return ((OS_FLAGS)0);
    }

    flags_rdy = OSTCBCurPtr->FlagsRdy;
    if (consume == DEF_TRUE) {                              /* See if we need to consume the flags                    */
        switch (mode) {
            case OS_OPT_PEND_FLAG_SET_ALL:
            case OS_OPT_PEND_FLAG_SET_ANY:                  /* Clear ONLY the flags we got                            */
                 p_grp->Flags &= ~flags_rdy;
                 break;

#if OS_CFG_FLAG_MODE_CLR_EN > 0u
            case OS_OPT_PEND_FLAG_CLR_ALL:
            case OS_OPT_PEND_FLAG_CLR_ANY:                  /* Set   ONLY the flags we got                            */
                 p_grp->Flags |=  flags_rdy;
                 break;
#endif
            default:
                 CPU_CRITICAL_EXIT();
                *p_err = OS_ERR_OPT_INVALID;
                 return ((OS_FLAGS)0);
        }
    }
    CPU_CRITICAL_EXIT();
   *p_err = OS_ERR_NONE;                                    /* Event(s) must have occurred                            */
    return (flags_rdy);
}


/*
************************************************************************************************************************
*                                          ABORT WAITING ON AN EVENT FLAG GROUP
*
* Description: This function aborts & readies any tasks currently waiting on an event flag group.  This function should
*              be used to fault-abort the wait on the event flag group, rather than to normally post to the event flag
*              group OSFlagPost().
*
* Arguments  : p_grp     is a pointer to the event flag group
*
*              opt       determines the type of ABORT performed:
*
*                            OS_OPT_PEND_ABORT_1          ABORT wait for a single task (HPT) waiting on the event flag
*                            OS_OPT_PEND_ABORT_ALL        ABORT wait for ALL tasks that are  waiting on the event flag
*                            OS_OPT_POST_NO_SCHED         Do not call the scheduler
*
*              p_err     is a pointer to a variable that will contain an error code returned by this function.
*
*                            OS_ERR_NONE                  At least one task waiting on the event flag group and was
*                                                         readied and informed of the aborted wait; check return value
*                                                         for the number of tasks whose wait on the event flag group
*                                                         was aborted.
*                            OS_ERR_OBJ_PTR_NULL          If 'p_grp' is a NULL pointer.
*                            OS_ERR_OBJ_TYPE              If 'p_grp' is not pointing at an event flag group
*                            OS_ERR_OPT_INVALID           If you specified an invalid option
*                            OS_ERR_PEND_ABORT_ISR        If you called this function from an ISR
*                            OS_ERR_PEND_ABORT_NONE       No task were pending
*
* Returns    : == 0          if no tasks were waiting on the event flag group, or upon error.
*              >  0          if one or more tasks waiting on the event flag group are now readied and informed.
************************************************************************************************************************
*/

#if OS_CFG_FLAG_PEND_ABORT_EN > 0u
OS_OBJ_QTY  OSFlagPendAbort (OS_FLAG_GRP  *p_grp,
                             OS_OPT        opt,
                             OS_ERR       *p_err)
{
    OS_PEND_LIST  *p_pend_list;
    OS_TCB        *p_tcb;
    CPU_TS         ts;
    OS_OBJ_QTY     nbr_tasks;
    CPU_SR_ALLOC();



#ifdef OS_SAFETY_CRITICAL
    if (p_err == (OS_ERR *)0) {
        OS_SAFETY_CRITICAL_EXCEPTION();
        return ((OS_OBJ_QTY)0u);
    }
#endif

#if OS_CFG_CALLED_FROM_ISR_CHK_EN > 0u
    if (OSIntNestingCtr > (OS_NESTING_CTR)0u) {             /* Not allowed to Pend Abort from an ISR                  */
       *p_err = OS_ERR_PEND_ABORT_ISR;
        return ((OS_OBJ_QTY)0u);
    }
#endif

#if OS_CFG_ARG_CHK_EN > 0u
    if (p_grp == (OS_FLAG_GRP *)0) {                        /* Validate 'p_grp'                                       */
       *p_err  =  OS_ERR_OBJ_PTR_NULL;
        return ((OS_OBJ_QTY)0u);
    }
    switch (opt) {                                          /* Validate 'opt'                                         */
        case OS_OPT_PEND_ABORT_1:
        case OS_OPT_PEND_ABORT_ALL:
        case OS_OPT_PEND_ABORT_1   | OS_OPT_POST_NO_SCHED:
        case OS_OPT_PEND_ABORT_ALL | OS_OPT_POST_NO_SCHED:
             break;

        default:
            *p_err = OS_ERR_OPT_INVALID;
             return ((OS_OBJ_QTY)0u);
    }
#endif

#if OS_CFG_OBJ_TYPE_CHK_EN > 0u
    if (p_grp->Type != OS_OBJ_TYPE_FLAG) {                  /* Make sure event flag group was created                 */
       *p_err = OS_ERR_OBJ_TYPE;
        return ((OS_OBJ_QTY)0u);
    }
#endif

    CPU_CRITICAL_ENTER();
    p_pend_list = &p_grp->PendList;
    if (p_pend_list->NbrEntries == (OS_OBJ_QTY)0u) {        /* Any task waiting on flag group?                        */
        CPU_CRITICAL_EXIT();                                /* No                                                     */
       *p_err = OS_ERR_PEND_ABORT_NONE;
        return ((OS_OBJ_QTY)0u);
    }

    OS_CRITICAL_ENTER_CPU_EXIT();
    nbr_tasks = 0u;
    ts        = OS_TS_GET();                                /* Get local time stamp so all tasks get the same time    */
    while (p_pend_list->NbrEntries > (OS_OBJ_QTY)0u) {
        p_tcb = p_pend_list->HeadPtr->TCBPtr;
        OS_PendAbort((OS_PEND_OBJ *)((void *)p_grp),
                     p_tcb,
                     ts);
        nbr_tasks++;
        if (opt != OS_OPT_PEND_ABORT_ALL) {                 /* Pend abort all tasks waiting?                          */
            break;                                          /* No                                                     */
        }
    }
    OS_CRITICAL_EXIT_NO_SCHED();

    if ((opt & OS_OPT_POST_NO_SCHED) == (OS_OPT)0u) {
        OSSched();                                          /* Run the scheduler                                      */
    }

   *p_err = OS_ERR_NONE;
    return (nbr_tasks);
}
#endif


/*
************************************************************************************************************************
*                                       GET FLAGS WHO CAUSED TASK TO BECOME READY
*
* Description: This function is called to obtain the flags that caused the task to become ready to run.
*              In other words, this function allows you to tell "Who done it!".
*
* Arguments  : p_err     is a pointer to an error code
*
*                            OS_ERR_NONE       if the call was successful
*                            OS_ERR_PEND_ISR   if called from an ISR
*
* Returns    : The flags that caused the task to be ready.
************************************************************************************************************************
*/

OS_FLAGS  OSFlagPendGetFlagsRdy (OS_ERR  *p_err)
{
    OS_FLAGS   flags;
    CPU_SR_ALLOC();



#ifdef OS_SAFETY_CRITICAL
    if (p_err == (OS_ERR *)0) {
        OS_SAFETY_CRITICAL_EXCEPTION();
        return ((OS_FLAGS)0);
    }
#endif

#if OS_CFG_CALLED_FROM_ISR_CHK_EN > 0u
    if (OSIntNestingCtr > (OS_NESTING_CTR)0) {              /* See if called from ISR ...                             */
       *p_err = OS_ERR_PEND_ISR;                            /* ... can't get from an ISR                              */
        return ((OS_FLAGS)0);
    }
#endif

    CPU_CRITICAL_ENTER();
    flags = OSTCBCurPtr->FlagsRdy;
    CPU_CRITICAL_EXIT();
   *p_err = OS_ERR_NONE;
    return (flags);
}


/*
************************************************************************************************************************
*                                                POST EVENT FLAG BIT(S)
*
* Description: This function is called to set or clear some bits in an event flag group.  The bits to set or clear are
*              specified by a 'bit mask'.
*
* Arguments  : p_grp         is a pointer to the desired event flag group.
*
*              flags         If 'opt' (see below) is OS_OPT_POST_FLAG_SET, each bit that is set in 'flags' will
*                            set the corresponding bit in the event flag group.  e.g. to set bits 0, 4
*                            and 5 you would set 'flags' to:
*
*                                0x31     (note, bit 0 is least significant bit)
*
*                            If 'opt' (see below) is OS_OPT_POST_FLAG_CLR, each bit that is set in 'flags' will
*                            CLEAR the corresponding bit in the event flag group.  e.g. to clear bits 0,
*                            4 and 5 you would specify 'flags' as:
*
*                                0x31     (note, bit 0 is least significant bit)
*
*              opt           indicates whether the flags will be:
*
*                                OS_OPT_POST_FLAG_SET       set
*                                OS_OPT_POST_FLAG_CLR       cleared
*
*                            you can also 'add' OS_OPT_POST_NO_SCHED to prevent the scheduler from being called.
*
*              p_err         is a pointer to an error code and can be:
*
*                                OS_ERR_NONE                The call was successful
*                                OS_ERR_OBJ_PTR_NULL        You passed a NULL pointer
*                                OS_ERR_OBJ_TYPE            You are not pointing to an event flag group
*                                OS_ERR_OPT_INVALID         You specified an invalid option
*
* Returns    : the new value of the event flags bits that are still set.
*
* Note(s)    : 1) The execution time of this function depends on the number of tasks waiting on the event flag group.
************************************************************************************************************************
*/

OS_FLAGS  OSFlagPost (OS_FLAG_GRP  *p_grp,
                      OS_FLAGS      flags,
                      OS_OPT        opt,
                      OS_ERR       *p_err)
{
    OS_FLAGS  flags_cur;
    CPU_TS    ts;



#ifdef OS_SAFETY_CRITICAL
    if (p_err == (OS_ERR *)0) {
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
        TRACE_OS_FLAG_POST_FAILED(p_grp);                   /* Record the event.                                      */
#endif
        OS_SAFETY_CRITICAL_EXCEPTION();
        return ((OS_FLAGS)0);
    }
#endif

#if OS_CFG_ARG_CHK_EN > 0u
    if (p_grp == (OS_FLAG_GRP *)0) {                        /* Validate 'p_grp'                                       */
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
        TRACE_OS_FLAG_POST_FAILED(p_grp);                   /* Record the event.                                      */
#endif
       *p_err  = OS_ERR_OBJ_PTR_NULL;
        return ((OS_FLAGS)0);
    }
    switch (opt) {                                          /* Validate 'opt'                                         */
        case OS_OPT_POST_FLAG_SET:
        case OS_OPT_POST_FLAG_CLR:
        case OS_OPT_POST_FLAG_SET | OS_OPT_POST_NO_SCHED:
        case OS_OPT_POST_FLAG_CLR | OS_OPT_POST_NO_SCHED:
             break;

        default:
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
             TRACE_OS_FLAG_POST_FAILED(p_grp);              /* Record the event.                                      */
#endif
            *p_err = OS_ERR_OPT_INVALID;
             return ((OS_FLAGS)0);
    }
#endif

#if OS_CFG_OBJ_TYPE_CHK_EN > 0u
    if (p_grp->Type != OS_OBJ_TYPE_FLAG) {                  /* Make sure we are pointing to an event flag grp         */
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
        TRACE_OS_FLAG_POST_FAILED(p_grp);                   /* Record the event.                                      */
#endif
       *p_err = OS_ERR_OBJ_TYPE;
        return ((OS_FLAGS)0);
    }
#endif

    ts = OS_TS_GET();                                       /* Get timestamp                                          */
#if OS_CFG_ISR_POST_DEFERRED_EN > 0u
    if (OSIntNestingCtr > (OS_NESTING_CTR)0) {              /* See if called from an ISR                              */
        OS_IntQPost((OS_OBJ_TYPE)OS_OBJ_TYPE_FLAG,          /* Post to ISR queue                                      */
                    (void      *)p_grp,
                    (void      *)0,
                    (OS_MSG_SIZE)0,
                    (OS_FLAGS   )flags,
                    (OS_OPT     )opt,
                    (CPU_TS     )ts,
                    (OS_ERR    *)p_err);
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
        TRACE_OS_FLAG_POST(p_grp);                          /* Record the event.                                      */
#endif
        return ((OS_FLAGS)0);
    }
#endif

    flags_cur = OS_FlagPost(p_grp,
                            flags,
                            opt,
                            ts,
                            p_err);

#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
    TRACE_OS_FLAG_POST(p_grp);                              /* Record the event.                                      */
#endif    

    return (flags_cur);
}


/*
************************************************************************************************************************
*                         SUSPEND TASK UNTIL EVENT FLAG(s) RECEIVED OR TIMEOUT OCCURS
*
* Description: This function is internal to uC/OS-III and is used to put a task to sleep until the desired
*              event flag bit(s) are set.
*
* Arguments  : p_pend_data    is a pointer to an object used to link the task being blocked to the list of task(s)
*              -----------    pending on the desired event flag group.
*
*              p_grp         is a pointer to the desired event flag group.
*              -----
*
*              flags         Is a bit pattern indicating which bit(s) (i.e. flags) you wish to check.
*                            The bits you want are specified by setting the corresponding bits in
*                            'flags'.  e.g. if your application wants to wait for bits 0 and 1 then
*                            'flags' would contain 0x03.
*
*              opt           specifies whether you want ALL bits to be set/cleared or ANY of the bits
*                            to be set/cleared.
*                            You can specify the following argument:
*
*                                OS_OPT_PEND_FLAG_CLR_ALL   You will check ALL bits in 'mask' to be clear (0)
*                                OS_OPT_PEND_FLAG_CLR_ANY   You will check ANY bit  in 'mask' to be clear (0)
*                                OS_OPT_PEND_FLAG_SET_ALL   You will check ALL bits in 'mask' to be set   (1)
*                                OS_OPT_PEND_FLAG_SET_ANY   You will check ANY bit  in 'mask' to be set   (1)
*
*              timeout       is the desired amount of time that the task will wait for the event flag
*                            bit(s) to be set.
*
* Returns    : none
*
* Note(s)    : This function is INTERNAL to uC/OS-III and your application should not call it.
************************************************************************************************************************
*/

void  OS_FlagBlock (OS_PEND_DATA  *p_pend_data,
                    OS_FLAG_GRP   *p_grp,
                    OS_FLAGS       flags,
                    OS_OPT         opt,
                    OS_TICK        timeout)
{
    OSTCBCurPtr->FlagsPend = flags;                         /* Save the flags that we need to wait for                */
    OSTCBCurPtr->FlagsOpt  = opt;                           /* Save the type of wait we are doing                     */
    OSTCBCurPtr->FlagsRdy  = (OS_FLAGS)0;

    OS_Pend(p_pend_data,
            (OS_PEND_OBJ *)((void *)p_grp),
             OS_TASK_PEND_ON_FLAG,
             timeout);
}


/*
************************************************************************************************************************
*                                      CLEAR THE CONTENTS OF AN EVENT FLAG GROUP
*
* Description: This function is called by OSFlagDel() to clear the contents of an event flag group
*

* Argument(s): p_grp     is a pointer to the event flag group to clear
*              -----
*
* Returns    : none
*
* Note(s)    : This function is INTERNAL to uC/OS-III and your application should not call it.
************************************************************************************************************************
*/

void  OS_FlagClr (OS_FLAG_GRP  *p_grp)
{
    OS_PEND_LIST  *p_pend_list;


#if OS_OBJ_TYPE_REQ > 0u
    p_grp->Type             = OS_OBJ_TYPE_NONE;
#endif
#if OS_CFG_DBG_EN > 0u
    p_grp->NamePtr          = (CPU_CHAR *)((void *)"?FLAG");    /* Unknown name                                       */
#endif
    p_grp->Flags            = (OS_FLAGS )0;
    p_pend_list             = &p_grp->PendList;
    OS_PendListInit(p_pend_list);
}


/*
************************************************************************************************************************
*                                          INITIALIZE THE EVENT FLAG MODULE
*
* Description: This function is called by uC/OS-III to initialize the event flag module.  Your application MUST NOT call
*              this function.  In other words, this function is internal to uC/OS-III.
*
* Arguments  : p_err     is a pointer to an error code that can contain one of the following values:
*
*                            OS_ERR_NONE   The call was successful.
*
* Returns    : none
*
* Note(s)    : This function is INTERNAL to uC/OS-III and your application should not call it.
************************************************************************************************************************
*/

void  OS_FlagInit (OS_ERR  *p_err)
{
#ifdef OS_SAFETY_CRITICAL
    if (p_err == (OS_ERR *)0) {
        OS_SAFETY_CRITICAL_EXCEPTION();
        return;
    }
#endif

#if OS_CFG_DBG_EN > 0u
    OSFlagDbgListPtr = (OS_FLAG_GRP *)0;
#endif

    OSFlagQty        = (OS_OBJ_QTY   )0;
   *p_err            = OS_ERR_NONE;
}


/*
************************************************************************************************************************
*                                    ADD/REMOVE EVENT FLAG GROUP TO/FROM DEBUG LIST
*
* Description: These functions are called by uC/OS-III to add or remove an event flag group from the event flag debug
*              list.
*
* Arguments  : p_grp     is a pointer to the event flag group to add/remove
*
* Returns    : none
*
* Note(s)    : These functions are INTERNAL to uC/OS-III and your application should not call it.
************************************************************************************************************************
*/

#if OS_CFG_DBG_EN > 0u
void  OS_FlagDbgListAdd (OS_FLAG_GRP  *p_grp)
{
    p_grp->DbgNamePtr                = (CPU_CHAR    *)((void *)" ");
    p_grp->DbgPrevPtr                = (OS_FLAG_GRP *)0;
    if (OSFlagDbgListPtr == (OS_FLAG_GRP *)0) {
        p_grp->DbgNextPtr            = (OS_FLAG_GRP *)0;
    } else {
        p_grp->DbgNextPtr            =  OSFlagDbgListPtr;
        OSFlagDbgListPtr->DbgPrevPtr =  p_grp;
    }
    OSFlagDbgListPtr                 =  p_grp;
}



void  OS_FlagDbgListRemove (OS_FLAG_GRP  *p_grp)
{
    OS_FLAG_GRP  *p_grp_next;
    OS_FLAG_GRP  *p_grp_prev;


    p_grp_prev = p_grp->DbgPrevPtr;
    p_grp_next = p_grp->DbgNextPtr;

    if (p_grp_prev == (OS_FLAG_GRP *)0) {
        OSFlagDbgListPtr = p_grp_next;
        if (p_grp_next != (OS_FLAG_GRP *)0) {
            p_grp_next->DbgPrevPtr = (OS_FLAG_GRP *)0;
        }
        p_grp->DbgNextPtr = (OS_FLAG_GRP *)0;

    } else if (p_grp_next == (OS_FLAG_GRP *)0) {
        p_grp_prev->DbgNextPtr = (OS_FLAG_GRP *)0;
        p_grp->DbgPrevPtr      = (OS_FLAG_GRP *)0;

    } else {
        p_grp_prev->DbgNextPtr =  p_grp_next;
        p_grp_next->DbgPrevPtr =  p_grp_prev;
        p_grp->DbgNextPtr      = (OS_FLAG_GRP *)0;
        p_grp->DbgPrevPtr      = (OS_FLAG_GRP *)0;
    }
}
#endif


/*
************************************************************************************************************************
*                                                POST EVENT FLAG BIT(S)
*
* Description: This function is called to set or clear some bits in an event flag group.  The bits to set or clear are
*              specified by a 'bit mask'.
*
* Arguments  : p_grp         is a pointer to the desired event flag group.
*
*              flags         If 'opt' (see below) is OS_OPT_POST_FLAG_SET, each bit that is set in 'flags' will
*                            set the corresponding bit in the event flag group.  e.g. to set bits 0, 4
*                            and 5 you would set 'flags' to:
*
*                                0x31     (note, bit 0 is least significant bit)
*
*                            If 'opt' (see below) is OS_OPT_POST_FLAG_CLR, each bit that is set in 'flags' will
*                            CLEAR the corresponding bit in the event flag group.  e.g. to clear bits 0,
*                            4 and 5 you would specify 'flags' as:
*
*                                0x31     (note, bit 0 is least significant bit)
*
*              opt           indicates whether the flags will be:
*
*                                OS_OPT_POST_FLAG_SET       set
*                                OS_OPT_POST_FLAG_CLR       cleared
*
*                            you can also 'add' OS_OPT_POST_NO_SCHED to prevent the scheduler from being called.
*
*              ts            is the timestamp of the post
*
*              p_err         is a pointer to an error code and can be:
*
*                                OS_ERR_NONE                The call was successful
*                                OS_ERR_OBJ_PTR_NULL        You passed a NULL pointer
*                                OS_ERR_OBJ_TYPE            You are not pointing to an event flag group
*                                OS_ERR_OPT_INVALID         You specified an invalid option
*
* Returns    : the new value of the event flags bits that are still set.
*
* Note(s)    : 1) The execution time of this function depends on the number of tasks waiting on the event flag group.
************************************************************************************************************************
*/

OS_FLAGS  OS_FlagPost (OS_FLAG_GRP  *p_grp,
                       OS_FLAGS      flags,
                       OS_OPT        opt,
                       CPU_TS        ts,
                       OS_ERR       *p_err)
{
    OS_FLAGS        flags_cur;
    OS_FLAGS        flags_rdy;
    OS_OPT          mode;
    OS_PEND_DATA   *p_pend_data;
    OS_PEND_DATA   *p_pend_data_next;
    OS_PEND_LIST   *p_pend_list;
    OS_TCB         *p_tcb;
    CPU_SR_ALLOC();



    CPU_CRITICAL_ENTER();
    switch (opt) {
        case OS_OPT_POST_FLAG_SET:
        case OS_OPT_POST_FLAG_SET | OS_OPT_POST_NO_SCHED:
             p_grp->Flags |=  flags;                            /* Set   the flags specified in the group             */
             break;

        case OS_OPT_POST_FLAG_CLR:
        case OS_OPT_POST_FLAG_CLR | OS_OPT_POST_NO_SCHED:
             p_grp->Flags &= ~flags;                            /* Clear the flags specified in the group             */
             break;

        default:
             CPU_CRITICAL_EXIT();                               /* INVALID option                                     */
            *p_err = OS_ERR_OPT_INVALID;
             return ((OS_FLAGS)0);
    }
    p_grp->TS   = ts;
    p_pend_list = &p_grp->PendList;
    if (p_pend_list->NbrEntries == 0u) {                        /* Any task waiting on event flag group?              */
        CPU_CRITICAL_EXIT();                                    /* No                                                 */
       *p_err = OS_ERR_NONE;
        return (p_grp->Flags);
    }

    OS_CRITICAL_ENTER_CPU_EXIT();
    p_pend_data = p_pend_list->HeadPtr;
    p_tcb       = p_pend_data->TCBPtr;
    while (p_tcb != (OS_TCB *)0) {                              /* Go through all tasks waiting on event flag(s)      */
        p_pend_data_next = p_pend_data->NextPtr;
        mode             = p_tcb->FlagsOpt & OS_OPT_PEND_FLAG_MASK;
        switch (mode) {
            case OS_OPT_PEND_FLAG_SET_ALL:                      /* See if all req. flags are set for current node     */
                 flags_rdy = (OS_FLAGS)(p_grp->Flags & p_tcb->FlagsPend);
                 if (flags_rdy == p_tcb->FlagsPend) {
                     OS_FlagTaskRdy(p_tcb,                      /* Make task RTR, event(s) Rx'd                       */
                                    flags_rdy,
                                    ts);
                 }
                 break;

            case OS_OPT_PEND_FLAG_SET_ANY:                      /* See if any flag set                                */
                 flags_rdy = (OS_FLAGS)(p_grp->Flags & p_tcb->FlagsPend);
                 if (flags_rdy != (OS_FLAGS)0) {
                     OS_FlagTaskRdy(p_tcb,                      /* Make task RTR, event(s) Rx'd                       */
                                    flags_rdy,
                                    ts);
                 }
                 break;

#if OS_CFG_FLAG_MODE_CLR_EN > 0u
            case OS_OPT_PEND_FLAG_CLR_ALL:                      /* See if all req. flags are set for current node     */
                 flags_rdy = (OS_FLAGS)(~p_grp->Flags & p_tcb->FlagsPend);
                 if (flags_rdy == p_tcb->FlagsPend) {
                     OS_FlagTaskRdy(p_tcb,                      /* Make task RTR, event(s) Rx'd                       */
                                    flags_rdy,
                                    ts);
                 }
                 break;

            case OS_OPT_PEND_FLAG_CLR_ANY:                      /* See if any flag set                                */
                 flags_rdy = (OS_FLAGS)(~p_grp->Flags & p_tcb->FlagsPend);
                 if (flags_rdy != (OS_FLAGS)0) {
                     OS_FlagTaskRdy(p_tcb,                      /* Make task RTR, event(s) Rx'd                       */
                                    flags_rdy,
                                    ts);
                 }
                 break;
#endif
            default:
                 OS_CRITICAL_EXIT();
                *p_err = OS_ERR_FLAG_PEND_OPT;
                 return ((OS_FLAGS)0);
        }
        p_pend_data = p_pend_data_next;                         /* Point to next task waiting for event flag(s)       */
        if (p_pend_data != (OS_PEND_DATA *)0) {
            p_tcb = p_pend_data->TCBPtr;
        } else {
            p_tcb = (OS_TCB *)0;
        }
    }
    OS_CRITICAL_EXIT_NO_SCHED();

    if ((opt & OS_OPT_POST_NO_SCHED) == (OS_OPT)0) {
        OSSched();
    }

    CPU_CRITICAL_ENTER();
    flags_cur = p_grp->Flags;
    CPU_CRITICAL_EXIT();
   *p_err     = OS_ERR_NONE;
    return (flags_cur);
}


/*
************************************************************************************************************************
*                                        MAKE TASK READY-TO-RUN, EVENT(s) OCCURRED
*
* Description: This function is internal to uC/OS-III and is used to make a task ready-to-run because the desired event
*              flag bits have been set.
*
* Arguments  : p_tcb         is a pointer to the OS_TCB of the task to remove
*              -----
*
*              flags_rdy     contains the bit pattern of the event flags that cause the task to become ready-to-run.
*
*              ts            is a timestamp associated with the post
*
* Returns    : none
*
* Note(s)    : This function is INTERNAL to uC/OS-III and your application should not call it.
************************************************************************************************************************
*/

void   OS_FlagTaskRdy (OS_TCB    *p_tcb,
                       OS_FLAGS   flags_rdy,
                       CPU_TS     ts)
{
    p_tcb->FlagsRdy   = flags_rdy;
    p_tcb->PendStatus = OS_STATUS_PEND_OK;                  /* Clear pend status                                      */
    p_tcb->PendOn     = OS_TASK_PEND_ON_NOTHING;            /* Indicate no longer pending                             */
    p_tcb->TS         = ts;
    switch (p_tcb->TaskState) {
        case OS_TASK_STATE_RDY:
        case OS_TASK_STATE_DLY:
        case OS_TASK_STATE_DLY_SUSPENDED:
        case OS_TASK_STATE_SUSPENDED:
             break;

        case OS_TASK_STATE_PEND:
        case OS_TASK_STATE_PEND_TIMEOUT:
             if (p_tcb->TaskState == OS_TASK_STATE_PEND_TIMEOUT) {
                 OS_TickListRemove(p_tcb);                  /* Remove from tick list                                  */
             }
             OS_RdyListInsert(p_tcb);                       /* Insert the task in the ready list                      */
             p_tcb->TaskState = OS_TASK_STATE_RDY;
             break;

        case OS_TASK_STATE_PEND_SUSPENDED:
        case OS_TASK_STATE_PEND_TIMEOUT_SUSPENDED:
             p_tcb->TaskState = OS_TASK_STATE_SUSPENDED;
             break;

        default:
             break;
    }
    OS_PendListRemove(p_tcb);
}
#endif
