/*
************************************************************************************************************************
*                                                      uC/OS-III
*                                                 The Real-Time Kernel
*
*                                  (c) Copyright 2009-2015; Micrium, Inc.; Weston, FL
*                           All rights reserved.  Protected by international copyright laws.
*
*                                                   MUTEX MANAGEMENT
*
* File    : OS_MUTEX.C
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
const  CPU_CHAR  *os_mutex__c = "$Id: $";
#endif


#if OS_CFG_MUTEX_EN > 0u
/*
************************************************************************************************************************
*                                                   CREATE A MUTEX
*
* Description: This function creates a mutex.
*
* Arguments  : p_mutex       is a pointer to the mutex to initialize.  Your application is responsible for allocating
*                            storage for the mutex.
*
*              p_name        is a pointer to the name you would like to give the mutex.
*
*              p_err         is a pointer to a variable that will contain an error code returned by this function.
*
*                                OS_ERR_NONE                    if the call was successful
*                                OS_ERR_CREATE_ISR              if you called this function from an ISR
*                                OS_ERR_ILLEGAL_CREATE_RUN_TIME if you are trying to create the Mutex after you called
*                                                                 OSSafetyCriticalStart().
*                                OS_ERR_NAME                    if 'p_name'  is a NULL pointer
*                                OS_ERR_OBJ_CREATED             if the mutex has already been created
*                                OS_ERR_OBJ_PTR_NULL            if 'p_mutex' is a NULL pointer
*
* Returns    : none
************************************************************************************************************************
*/

void  OSMutexCreate (OS_MUTEX  *p_mutex,
                     CPU_CHAR  *p_name,
                     OS_ERR    *p_err)
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
    if (OSIntNestingCtr > (OS_NESTING_CTR)0) {              /* Not allowed to be called from an ISR                   */
       *p_err = OS_ERR_CREATE_ISR;
        return;
    }
#endif

#if OS_CFG_ARG_CHK_EN > 0u
    if (p_mutex == (OS_MUTEX *)0) {                         /* Validate 'p_mutex'                                     */
       *p_err = OS_ERR_OBJ_PTR_NULL;
        return;
    }
#endif

    OS_CRITICAL_ENTER();
#if OS_OBJ_TYPE_REQ > 0u
    p_mutex->Type              =  OS_OBJ_TYPE_MUTEX;        /* Mark the data structure as a mutex                     */
#endif
#if OS_CFG_DBG_EN > 0u
    p_mutex->NamePtr           =  p_name;
#else
    (void)&p_name;
#endif
    p_mutex->MutexGrpNextPtr   = (OS_MUTEX     *)0;
    p_mutex->OwnerTCBPtr       = (OS_TCB       *)0;
    p_mutex->OwnerNestingCtr   = (OS_NESTING_CTR)0;         /* Mutex is available                                     */
    p_mutex->TS                = (CPU_TS        )0;
    OS_PendListInit(&p_mutex->PendList);                    /* Initialize the waiting list                            */

#if OS_CFG_DBG_EN > 0u
    OS_MutexDbgListAdd(p_mutex);
#endif

    OSMutexQty++;

#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
    TRACE_OS_MUTEX_CREATE(p_mutex, p_name);                 /* Record the event.                                      */
#endif

    OS_CRITICAL_EXIT_NO_SCHED();
   *p_err = OS_ERR_NONE;
}


/*
************************************************************************************************************************
*                                                   DELETE A MUTEX
*
* Description: This function deletes a mutex and readies all tasks pending on the mutex.
*
* Arguments  : p_mutex       is a pointer to the mutex to delete
*
*              opt           determines delete options as follows:
*
*                                OS_OPT_DEL_NO_PEND          Delete mutex ONLY if no task pending
*                                OS_OPT_DEL_ALWAYS           Deletes the mutex even if tasks are waiting.
*                                                            In this case, all the tasks pending will be readied.
*
*              p_err         is a pointer to a variable that will contain an error code returned by this function.
*
*                                OS_ERR_NONE                 The call was successful and the mutex was deleted
*                                OS_ERR_DEL_ISR              If you attempted to delete the mutex from an ISR
*                                OS_ERR_OBJ_PTR_NULL         If 'p_mutex' is a NULL pointer.
*                                OS_ERR_OBJ_TYPE             If 'p_mutex' is not pointing to a mutex
*                                OS_ERR_OPT_INVALID          An invalid option was specified
*                                OS_ERR_STATE_INVALID        Task is in an invalid state
*                                OS_ERR_TASK_WAITING         One or more tasks were waiting on the mutex
*
* Returns    : == 0          if no tasks were waiting on the mutex, or upon error.
*              >  0          if one or more tasks waiting on the mutex are now readied and informed.
*
* Note(s)    : 1) This function must be used with care.  Tasks that would normally expect the presence of the mutex MUST
*                 check the return code of OSMutexPend().
*
*              2) Because ALL tasks pending on the mutex will be readied, you MUST be careful in applications where the
*                 mutex is used for mutual exclusion because the resource(s) will no longer be guarded by the mutex.
************************************************************************************************************************
*/

#if OS_CFG_MUTEX_DEL_EN > 0u
OS_OBJ_QTY  OSMutexDel (OS_MUTEX  *p_mutex,
                        OS_OPT     opt,
                        OS_ERR    *p_err)
{
    OS_OBJ_QTY     cnt;
    OS_OBJ_QTY     nbr_tasks;
    OS_PEND_DATA  *p_pend_data;
    OS_PEND_LIST  *p_pend_list;
    OS_TCB        *p_tcb;
    OS_TCB        *p_tcb_owner;
    CPU_TS         ts;
#if OS_CFG_MUTEX_EN > 0u
    OS_PRIO        prio_new;
#endif
    CPU_SR_ALLOC();



#ifdef OS_SAFETY_CRITICAL
    if (p_err == (OS_ERR *)0) {
        OS_SAFETY_CRITICAL_EXCEPTION();
        return ((OS_OBJ_QTY)0);
    }
#endif

#if OS_CFG_CALLED_FROM_ISR_CHK_EN > 0u
    if (OSIntNestingCtr > (OS_NESTING_CTR)0) {                  /* Not allowed to delete a mutex from an ISR            */
       *p_err = OS_ERR_DEL_ISR;
        return ((OS_OBJ_QTY)0);
    }
#endif

#if OS_CFG_ARG_CHK_EN > 0u
    if (p_mutex == (OS_MUTEX *)0) {                             /* Validate 'p_mutex'                                   */
       *p_err = OS_ERR_OBJ_PTR_NULL;
        return ((OS_OBJ_QTY)0);
    }
    switch (opt) {                                              /* Validate 'opt'                                       */
        case OS_OPT_DEL_NO_PEND:
        case OS_OPT_DEL_ALWAYS:
             break;

        default:
            *p_err =  OS_ERR_OPT_INVALID;
             return ((OS_OBJ_QTY)0);
    }
#endif

#if OS_CFG_OBJ_TYPE_CHK_EN > 0u
    if (p_mutex->Type != OS_OBJ_TYPE_MUTEX) {                   /* Make sure mutex was created                          */
       *p_err = OS_ERR_OBJ_TYPE;
        return ((OS_OBJ_QTY)0);
    }
#endif

    OS_CRITICAL_ENTER();
    p_pend_list = &p_mutex->PendList;
    cnt         = p_pend_list->NbrEntries;
    nbr_tasks   = cnt;
    switch (opt) {
        case OS_OPT_DEL_NO_PEND:                                /* Delete mutex only if no task waiting                 */
             if (nbr_tasks == (OS_OBJ_QTY)0) {
#if OS_CFG_DBG_EN > 0u
                 OS_MutexDbgListRemove(p_mutex);
#endif
                 OSMutexQty--;
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
                 TRACE_OS_MUTEX_DEL(p_mutex);                   /* Record the event.                                    */
#endif
                 if (p_mutex->OwnerTCBPtr != (OS_TCB *)0) {     /* Does the mutex belong to a task?                     */
                     OS_MutexGrpRemove(p_mutex->OwnerTCBPtr, p_mutex); /* yes, remove it from the task group.           */
                 }
                 OS_MutexClr(p_mutex);
                 OS_CRITICAL_EXIT();
                *p_err = OS_ERR_NONE;
             } else {
                 OS_CRITICAL_EXIT();
                *p_err = OS_ERR_TASK_WAITING;
             }
             break;

        case OS_OPT_DEL_ALWAYS:                                 /* Always delete the mutex                              */
             ts = OS_TS_GET();                                  /* Get timestamp                                        */
             while (cnt > 0u) {                                 /* Remove all tasks from the pend list                  */
                 p_pend_data = p_pend_list->HeadPtr;
                 p_tcb       = p_pend_data->TCBPtr;
                 OS_PendObjDel((OS_PEND_OBJ *)((void *)p_mutex),
                               p_tcb,
                               ts);
                 cnt--;
             }
#if OS_CFG_DBG_EN > 0u
             OS_MutexDbgListRemove(p_mutex);
#endif
             OSMutexQty--;
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
             TRACE_OS_MUTEX_DEL(p_mutex);                       /* Record the event.                                    */
#endif

             p_tcb_owner = p_mutex->OwnerTCBPtr;
             if (p_tcb_owner != (OS_TCB *)0) {                  /* Does the mutex belong to a task?                     */
                 OS_MutexGrpRemove(p_tcb_owner, p_mutex);       /* yes, remove it from the task group.                  */
             }


             if ((p_tcb_owner       != (OS_TCB *)0) &&          /* Did we had to change the prio of owner?              */
                 (p_tcb_owner->Prio !=  p_tcb_owner->BasePrio)) {
                 prio_new = OS_MutexGrpPrioFindHighest(p_tcb_owner);
                 prio_new = prio_new > p_tcb_owner->BasePrio ? p_tcb_owner->BasePrio : prio_new;
                 OS_TaskChangePrio(p_tcb_owner, prio_new);
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
                          TRACE_OS_MUTEX_TASK_PRIO_DISINHERIT(p_tcb_owner, p_tcb_owner->Prio)
#endif
             }

             OS_MutexClr(p_mutex);
             OS_CRITICAL_EXIT_NO_SCHED();
             OSSched();                                         /* Find highest priority task ready to run              */
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
*                                                    PEND ON MUTEX
*
* Description: This function waits for a mutex.
*
* Arguments  : p_mutex       is a pointer to the mutex
*
*              timeout       is an optional timeout period (in clock ticks).  If non-zero, your task will wait for the
*                            resource up to the amount of time (in 'ticks') specified by this argument.  If you specify
*                            0, however, your task will wait forever at the specified mutex or, until the resource
*                            becomes available.
*
*              opt           determines whether the user wants to block if the mutex is not available or not:
*
*                                OS_OPT_PEND_BLOCKING
*                                OS_OPT_PEND_NON_BLOCKING
*
*              p_ts          is a pointer to a variable that will receive the timestamp of when the mutex was posted or
*                            pend aborted or the mutex deleted.  If you pass a NULL pointer (i.e. (CPU_TS *)0) then you
*                            will not get the timestamp.  In other words, passing a NULL pointer is valid and indicates
*                            that you don't need the timestamp.
*
*              p_err         is a pointer to a variable that will contain an error code returned by this function.
*
*                                OS_ERR_NONE               The call was successful and your task owns the resource
*                                OS_ERR_MUTEX_OWNER        If calling task already owns the mutex
*                                OS_ERR_OBJ_DEL            If 'p_mutex' was deleted
*                                OS_ERR_OBJ_PTR_NULL       If 'p_mutex' is a NULL pointer.
*                                OS_ERR_OBJ_TYPE           If 'p_mutex' is not pointing at a mutex
*                                OS_ERR_OPT_INVALID        If you didn't specify a valid option
*                                OS_ERR_PEND_ABORT         If the pend was aborted by another task
*                                OS_ERR_PEND_ISR           If you called this function from an ISR and the result
*                                                          would lead to a suspension.
*                                OS_ERR_PEND_WOULD_BLOCK   If you specified non-blocking but the mutex was not
*                                                          available.
*                                OS_ERR_SCHED_LOCKED       If you called this function when the scheduler is locked
*                                OS_ERR_STATE_INVALID      If the task is in an invalid state
*                                OS_ERR_STATUS_INVALID     If the pend status has an invalid value
*                                OS_ERR_TIMEOUT            The mutex was not received within the specified timeout.
*
* Returns    : none
************************************************************************************************************************
*/

void  OSMutexPend (OS_MUTEX  *p_mutex,
                   OS_TICK    timeout,
                   OS_OPT     opt,
                   CPU_TS    *p_ts,
                   OS_ERR    *p_err)
{
    OS_PEND_DATA  pend_data;
    OS_TCB       *p_tcb;
    CPU_SR_ALLOC();



#ifdef OS_SAFETY_CRITICAL
    if (p_err == (OS_ERR *)0) {
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
        TRACE_OS_MUTEX_PEND_FAILED(p_mutex);                /* Record the event.                                      */
#endif
        OS_SAFETY_CRITICAL_EXCEPTION();
        return;
    }
#endif

#if OS_CFG_CALLED_FROM_ISR_CHK_EN > 0u
    if (OSIntNestingCtr > (OS_NESTING_CTR)0) {              /* Not allowed to call from an ISR                        */
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
        TRACE_OS_MUTEX_PEND_FAILED(p_mutex);                /* Record the event.                                      */
#endif
       *p_err = OS_ERR_PEND_ISR;
        return;
    }
#endif

#if OS_CFG_ARG_CHK_EN > 0u
    if (p_mutex == (OS_MUTEX *)0) {                         /* Validate arguments                                     */
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
        TRACE_OS_MUTEX_PEND_FAILED(p_mutex);                /* Record the event.                                      */
#endif
       *p_err = OS_ERR_OBJ_PTR_NULL;
        return;
    }
    switch (opt) {                                          /* Validate 'opt'                                         */
        case OS_OPT_PEND_BLOCKING:
        case OS_OPT_PEND_NON_BLOCKING:
             break;

        default:
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
             TRACE_OS_MUTEX_PEND_FAILED(p_mutex);           /* Record the event.                                      */
#endif
            *p_err = OS_ERR_OPT_INVALID;
             return;
    }
#endif

#if OS_CFG_OBJ_TYPE_CHK_EN > 0u
    if (p_mutex->Type != OS_OBJ_TYPE_MUTEX) {               /* Make sure mutex was created                            */
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
        TRACE_OS_MUTEX_PEND_FAILED(p_mutex);                /* Record the event.                                      */
#endif
       *p_err = OS_ERR_OBJ_TYPE;
        return;
    }
#endif

    if (p_ts != (CPU_TS *)0) {
       *p_ts  = (CPU_TS  )0;                                /* Initialize the returned timestamp                      */
    }

    CPU_CRITICAL_ENTER();
    if (p_mutex->OwnerNestingCtr == (OS_NESTING_CTR)0) {    /* Resource available?                                    */
        p_mutex->OwnerTCBPtr       =  OSTCBCurPtr;          /* Yes, caller may proceed                                */
        p_mutex->OwnerNestingCtr   = (OS_NESTING_CTR)1;
        if (p_ts != (CPU_TS *)0) {
           *p_ts  = p_mutex->TS;
        }
        OS_MutexGrpAdd(OSTCBCurPtr, p_mutex);               /* Add mutex to owner's group                             */
        CPU_CRITICAL_EXIT();
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
        TRACE_OS_MUTEX_PEND(p_mutex);                       /* Record the event.                                      */
#endif
       *p_err = OS_ERR_NONE;
        return;
    }

    if (OSTCBCurPtr == p_mutex->OwnerTCBPtr) {              /* See if current task is already the owner of the mutex  */
        if (p_mutex->OwnerNestingCtr == (OS_NESTING_CTR)-1) {
            CPU_CRITICAL_EXIT();
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
            TRACE_OS_MUTEX_PEND_FAILED(p_mutex);            /* Record the event.                                      */
#endif
           *p_err = OS_ERR_MUTEX_OVF;
            return;
        }
        p_mutex->OwnerNestingCtr++;
        if (p_ts != (CPU_TS *)0) {
           *p_ts  = p_mutex->TS;
        }
        CPU_CRITICAL_EXIT();
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
        TRACE_OS_MUTEX_PEND_FAILED(p_mutex);                /* Record the event.                                      */
#endif
       *p_err = OS_ERR_MUTEX_OWNER;                         /* Indicate that current task already owns the mutex      */
        return;
    }

    if ((opt & OS_OPT_PEND_NON_BLOCKING) != (OS_OPT)0) {    /* Caller wants to block if not available?                */
        CPU_CRITICAL_EXIT();
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
        TRACE_OS_MUTEX_PEND_FAILED(p_mutex);                /* Record the event.                                      */
#endif
       *p_err = OS_ERR_PEND_WOULD_BLOCK;                    /* No                                                     */
        return;
    } else {
        if (OSSchedLockNestingCtr > (OS_NESTING_CTR)0) {    /* Can't pend when the scheduler is locked                */
            CPU_CRITICAL_EXIT();
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
            TRACE_OS_MUTEX_PEND_FAILED(p_mutex);            /* Record the event.                                      */
#endif
           *p_err = OS_ERR_SCHED_LOCKED;
            return;
        }
    }
                                                            /* Lock the scheduler/re-enable interrupts                */
    OS_CRITICAL_ENTER_CPU_EXIT();
    p_tcb = p_mutex->OwnerTCBPtr;                           /* Point to the TCB of the Mutex owner                    */
    if (p_tcb->Prio > OSTCBCurPtr->Prio) {                  /* See if mutex owner has a lower priority than current   */
        OS_TaskChangePrio(p_tcb, OSTCBCurPtr->Prio);
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
                 TRACE_OS_MUTEX_TASK_PRIO_INHERIT(p_tcb, p_tcb->Prio);
#endif
    }

    OS_Pend(&pend_data,                                     /* Block task pending on Mutex                            */
            (OS_PEND_OBJ *)((void *)p_mutex),
             OS_TASK_PEND_ON_MUTEX,
             timeout);

    OS_CRITICAL_EXIT_NO_SCHED();
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
    TRACE_OS_MUTEX_PEND_BLOCK(p_mutex);                     /* Record the event.                                      */
#endif
    OSSched();                                              /* Find the next highest priority task ready to run       */

    CPU_CRITICAL_ENTER();
    switch (OSTCBCurPtr->PendStatus) {
        case OS_STATUS_PEND_OK:                             /* We got the mutex                                       */
             if (p_ts != (CPU_TS *)0) {
                *p_ts  = OSTCBCurPtr->TS;
             }
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
             TRACE_OS_MUTEX_PEND(p_mutex);                  /* Record the event.                                      */
#endif
            *p_err = OS_ERR_NONE;
             break;

        case OS_STATUS_PEND_ABORT:                          /* Indicate that we aborted                               */
             if (p_ts != (CPU_TS *)0) {
                *p_ts  = OSTCBCurPtr->TS;
             }
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
             TRACE_OS_MUTEX_PEND_FAILED(p_mutex);           /* Record the event.                                      */
#endif
            *p_err = OS_ERR_PEND_ABORT;
             break;

        case OS_STATUS_PEND_TIMEOUT:                        /* Indicate that we didn't get mutex within timeout       */
             if (p_ts != (CPU_TS *)0) {
                *p_ts  = (CPU_TS  )0;
             }
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
             TRACE_OS_MUTEX_PEND_FAILED(p_mutex);           /* Record the event.                                      */
#endif
            *p_err = OS_ERR_TIMEOUT;
             break;

        case OS_STATUS_PEND_DEL:                            /* Indicate that object pended on has been deleted        */
             if (p_ts != (CPU_TS *)0) {
                *p_ts  = OSTCBCurPtr->TS;
             }
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
             TRACE_OS_MUTEX_PEND_FAILED(p_mutex);           /* Record the event.                                      */
#endif
            *p_err = OS_ERR_OBJ_DEL;
             break;

        default:
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
             TRACE_OS_MUTEX_PEND_FAILED(p_mutex);           /* Record the event.                                      */
#endif
            *p_err = OS_ERR_STATUS_INVALID;
             break;
    }
    CPU_CRITICAL_EXIT();
}


/*
************************************************************************************************************************
*                                               ABORT WAITING ON A MUTEX
*
* Description: This function aborts & readies any tasks currently waiting on a mutex.  This function should be used
*              to fault-abort the wait on the mutex, rather than to normally signal the mutex via OSMutexPost().
*
* Arguments  : p_mutex   is a pointer to the mutex
*
*              opt       determines the type of ABORT performed:
*
*                            OS_OPT_PEND_ABORT_1          ABORT wait for a single task (HPT) waiting on the mutex
*                            OS_OPT_PEND_ABORT_ALL        ABORT wait for ALL tasks that are  waiting on the mutex
*                            OS_OPT_POST_NO_SCHED         Do not call the scheduler
*
*              p_err     is a pointer to a variable that will contain an error code returned by this function.
*
*                            OS_ERR_NONE                  At least one task waiting on the mutex was readied and
*                                                         informed of the aborted wait; check return value for the
*                                                         number of tasks whose wait on the mutex was aborted.
*                            OS_ERR_OBJ_PTR_NULL          If 'p_mutex' is a NULL pointer.
*                            OS_ERR_OBJ_TYPE              If 'p_mutex' is not pointing at a mutex
*                            OS_ERR_OPT_INVALID           If you specified an invalid option
*                            OS_ERR_PEND_ABORT_ISR        If you attempted to call this function from an ISR
*                            OS_ERR_PEND_ABORT_NONE       No task were pending
*
* Returns    : == 0          if no tasks were waiting on the mutex, or upon error.
*              >  0          if one or more tasks waiting on the mutex are now readied and informed.
************************************************************************************************************************
*/

#if OS_CFG_MUTEX_PEND_ABORT_EN > 0u
OS_OBJ_QTY  OSMutexPendAbort (OS_MUTEX  *p_mutex,
                              OS_OPT     opt,
                              OS_ERR    *p_err)
{
    OS_PEND_LIST  *p_pend_list;
    OS_TCB        *p_tcb;
    OS_TCB        *p_tcb_owner;
    CPU_TS         ts;
    OS_OBJ_QTY     nbr_tasks;
    OS_PRIO        prio_new;
    CPU_SR_ALLOC();



#ifdef OS_SAFETY_CRITICAL
    if (p_err == (OS_ERR *)0) {
        OS_SAFETY_CRITICAL_EXCEPTION();
        return ((OS_OBJ_QTY)0u);
    }
#endif

#if OS_CFG_CALLED_FROM_ISR_CHK_EN > 0u
    if (OSIntNestingCtr > (OS_NESTING_CTR)0u) {                 /* Not allowed to Pend Abort from an ISR                */
       *p_err =  OS_ERR_PEND_ABORT_ISR;
        return ((OS_OBJ_QTY)0u);
    }
#endif

#if OS_CFG_ARG_CHK_EN > 0u
    if (p_mutex == (OS_MUTEX *)0) {                             /* Validate 'p_mutex'                                   */
       *p_err =  OS_ERR_OBJ_PTR_NULL;
        return ((OS_OBJ_QTY)0u);
    }
    switch (opt) {                                              /* Validate 'opt'                                       */
        case OS_OPT_PEND_ABORT_1:
        case OS_OPT_PEND_ABORT_ALL:
        case OS_OPT_PEND_ABORT_1   | OS_OPT_POST_NO_SCHED:
        case OS_OPT_PEND_ABORT_ALL | OS_OPT_POST_NO_SCHED:
             break;

        default:
            *p_err =  OS_ERR_OPT_INVALID;
             return ((OS_OBJ_QTY)0u);
    }
#endif

#if OS_CFG_OBJ_TYPE_CHK_EN > 0u
    if (p_mutex->Type != OS_OBJ_TYPE_MUTEX) {                   /* Make sure mutex was created                          */
       *p_err =  OS_ERR_OBJ_TYPE;
        return ((OS_OBJ_QTY)0u);
    }
#endif

    CPU_CRITICAL_ENTER();
    p_pend_list = &p_mutex->PendList;
    if (p_pend_list->NbrEntries == (OS_OBJ_QTY)0u) {            /* Any task waiting on mutex?                           */
        CPU_CRITICAL_EXIT();                                    /* No                                                   */
       *p_err =  OS_ERR_PEND_ABORT_NONE;
        return ((OS_OBJ_QTY)0u);
    }

    OS_CRITICAL_ENTER_CPU_EXIT();
    nbr_tasks = 0u;
    ts        = OS_TS_GET();                                    /* Get local time stamp so all tasks get the same time  */
    while (p_pend_list->NbrEntries > (OS_OBJ_QTY)0u) {
        p_tcb = p_pend_list->HeadPtr->TCBPtr;

        OS_PendAbort((OS_PEND_OBJ *)((void *)p_mutex),
                     p_tcb,
                     ts);

        p_tcb_owner = p_mutex->OwnerTCBPtr;
        prio_new    = p_tcb_owner->Prio;
        if ((p_tcb_owner->Prio != p_tcb_owner->BasePrio) &&
            (p_tcb_owner->Prio == p_tcb->Prio)) {               /* Has the owner inherited a priority?                  */
            prio_new = OS_MutexGrpPrioFindHighest(p_tcb_owner);
            prio_new = prio_new > p_tcb_owner->BasePrio ? p_tcb_owner->BasePrio : prio_new;
        }

        if(prio_new != p_tcb_owner->Prio) {
            OS_TaskChangePrio(p_tcb_owner, prio_new);
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
                          TRACE_OS_MUTEX_TASK_PRIO_DISINHERIT(p_tcb_owner, p_tcb_owner->Prio);
#endif
        }

        nbr_tasks++;
        if (opt != OS_OPT_PEND_ABORT_ALL) {                     /* Pend abort all tasks waiting?                        */
            break;                                              /* No                                                   */
        }
    }
    OS_CRITICAL_EXIT_NO_SCHED();

    if ((opt & OS_OPT_POST_NO_SCHED) == (OS_OPT)0u) {
        OSSched();                                              /* Run the scheduler                                    */
    }

   *p_err = OS_ERR_NONE;
    return (nbr_tasks);
}
#endif


/*
************************************************************************************************************************
*                                                   POST TO A MUTEX
*
* Description: This function signals a mutex
*
* Arguments  : p_mutex  is a pointer to the mutex
*
*              opt      is an option you can specify to alter the behavior of the post.  The choices are:
*
*                           OS_OPT_POST_NONE        No special option selected
*                           OS_OPT_POST_NO_SCHED    If you don't want the scheduler to be called after the post.
*
*              p_err    is a pointer to a variable that will contain an error code returned by this function.
*
*                           OS_ERR_NONE             The call was successful and the mutex was signaled.
*                           OS_ERR_MUTEX_NESTING    Mutex owner nested its use of the mutex
*                           OS_ERR_MUTEX_NOT_OWNER  If the task posting is not the Mutex owner
*                           OS_ERR_OBJ_PTR_NULL     If 'p_mutex' is a NULL pointer.
*                           OS_ERR_OBJ_TYPE         If 'p_mutex' is not pointing at a mutex
*                           OS_ERR_POST_ISR         If you attempted to post from an ISR
*
* Returns    : none
************************************************************************************************************************
*/

void  OSMutexPost (OS_MUTEX  *p_mutex,
                   OS_OPT     opt,
                   OS_ERR    *p_err)
{
    OS_PEND_LIST  *p_pend_list;
    OS_TCB        *p_tcb;
    CPU_TS         ts;
    OS_PRIO        prio_new;
    CPU_SR_ALLOC();



#ifdef OS_SAFETY_CRITICAL
    if (p_err == (OS_ERR *)0) {
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
        TRACE_OS_MUTEX_POST_FAILED(p_mutex);                /* Record the event.                                      */
#endif
        OS_SAFETY_CRITICAL_EXCEPTION();
        return;
    }
#endif

#if OS_CFG_CALLED_FROM_ISR_CHK_EN > 0u
    if (OSIntNestingCtr > (OS_NESTING_CTR)0) {              /* Not allowed to call from an ISR                        */
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
        TRACE_OS_MUTEX_POST_FAILED(p_mutex);                /* Record the event.                                      */
#endif
       *p_err = OS_ERR_POST_ISR;
        return;
    }
#endif

#if OS_CFG_ARG_CHK_EN > 0u
    if (p_mutex == (OS_MUTEX *)0) {                         /* Validate 'p_mutex'                                     */
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
        TRACE_OS_MUTEX_POST_FAILED(p_mutex);                /* Record the event.                                      */
#endif
       *p_err = OS_ERR_OBJ_PTR_NULL;
        return;
    }
    switch (opt) {                                          /* Validate 'opt'                                         */
        case OS_OPT_POST_NONE:
        case OS_OPT_POST_NO_SCHED:
             break;

        default:
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
             TRACE_OS_MUTEX_POST_FAILED(p_mutex);           /* Record the event.                                      */
#endif
            *p_err =  OS_ERR_OPT_INVALID;
             return;
    }
#endif

#if OS_CFG_OBJ_TYPE_CHK_EN > 0u
    if (p_mutex->Type != OS_OBJ_TYPE_MUTEX) {               /* Make sure mutex was created                            */
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
        TRACE_OS_MUTEX_POST_FAILED(p_mutex);                /* Record the event.                                      */
#endif
       *p_err = OS_ERR_OBJ_TYPE;
        return;
    }
#endif

    CPU_CRITICAL_ENTER();
    if (OSTCBCurPtr != p_mutex->OwnerTCBPtr) {              /* Make sure the mutex owner is releasing the mutex       */
        CPU_CRITICAL_EXIT();
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
        TRACE_OS_MUTEX_POST_FAILED(p_mutex);                /* Record the event.                                      */
#endif
       *p_err = OS_ERR_MUTEX_NOT_OWNER;
        return;
    }

#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
    TRACE_OS_MUTEX_POST(p_mutex);                           /* Record the event.                                      */
#endif

    OS_CRITICAL_ENTER_CPU_EXIT();
    ts          = OS_TS_GET();                              /* Get timestamp                                          */
    p_mutex->TS = ts;
    p_mutex->OwnerNestingCtr--;                             /* Decrement owner's nesting counter                      */
    if (p_mutex->OwnerNestingCtr > (OS_NESTING_CTR)0) {     /* Are we done with all nestings?                         */
        OS_CRITICAL_EXIT();                                 /* No                                                     */
       *p_err = OS_ERR_MUTEX_NESTING;
        return;
    }

    OS_MutexGrpRemove(OSTCBCurPtr, p_mutex);                /* Remove mutex from owner's group                        */

    p_pend_list = &p_mutex->PendList;
    if (p_pend_list->NbrEntries == (OS_OBJ_QTY)0) {         /* Any task waiting on mutex?                             */
        p_mutex->OwnerTCBPtr     = (OS_TCB       *)0;       /* No                                                     */
        p_mutex->OwnerNestingCtr = (OS_NESTING_CTR)0;
        OS_CRITICAL_EXIT();
       *p_err = OS_ERR_NONE;
        return;
    }
                                                            /* Yes                                                    */
    if (OSTCBCurPtr->Prio != OSTCBCurPtr->BasePrio) {       /* Has owner inherited a priority?                        */
        prio_new = OS_MutexGrpPrioFindHighest(OSTCBCurPtr); /* Yes, find highest priority pending                     */
        prio_new = prio_new > OSTCBCurPtr->BasePrio ? OSTCBCurPtr->BasePrio : prio_new;
        if (prio_new > OSTCBCurPtr->Prio) {
            OS_RdyListRemove(OSTCBCurPtr);
            OSTCBCurPtr->Prio = prio_new;                   /* Lower owner's priority back to its original one        */
#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
            TRACE_OS_MUTEX_TASK_PRIO_DISINHERIT(OSTCBCurPtr, prio_new);
#endif
            OS_PrioInsert(prio_new);
            OS_RdyListInsertTail(OSTCBCurPtr);              /* Insert owner in ready list at new priority             */
            OSPrioCur         = prio_new;
        }
    }
                                                            /* Get TCB from head of pend list                         */
    p_tcb                      = p_pend_list->HeadPtr->TCBPtr;
    p_mutex->OwnerTCBPtr       = p_tcb;                     /* Give mutex to new owner                                */
    p_mutex->OwnerNestingCtr   = (OS_NESTING_CTR)1;
    OS_MutexGrpAdd(p_tcb, p_mutex);
                                                            /* Post to mutex                                          */
    OS_Post((OS_PEND_OBJ *)((void *)p_mutex),
            (OS_TCB      *)p_tcb,
            (void        *)0,
            (OS_MSG_SIZE  )0,
            (CPU_TS       )ts);

    OS_CRITICAL_EXIT_NO_SCHED();

    if ((opt & OS_OPT_POST_NO_SCHED) == (OS_OPT)0) {
        OSSched();                                          /* Run the scheduler                                      */
    }

   *p_err = OS_ERR_NONE;
}


/*
************************************************************************************************************************
*                                            CLEAR THE CONTENTS OF A MUTEX
*
* Description: This function is called by OSMutexDel() to clear the contents of a mutex
*

* Argument(s): p_mutex      is a pointer to the mutex to clear
*              -------
*
* Returns    : none
*
* Note(s)    : This function is INTERNAL to uC/OS-III and your application should not call it.
************************************************************************************************************************
*/

void  OS_MutexClr (OS_MUTEX  *p_mutex)
{
#if OS_OBJ_TYPE_REQ > 0u
    p_mutex->Type              =  OS_OBJ_TYPE_NONE;         /* Mark the data structure as a NONE                      */
#endif
#if OS_CFG_DBG_EN > 0u
    p_mutex->NamePtr           = (CPU_CHAR     *)((void *)"?MUTEX");
#endif
    p_mutex->MutexGrpNextPtr   = (OS_MUTEX     *)0;
    p_mutex->OwnerTCBPtr       = (OS_TCB       *)0;
    p_mutex->OwnerNestingCtr   = (OS_NESTING_CTR)0;
    p_mutex->TS                = (CPU_TS        )0;
    OS_PendListInit(&p_mutex->PendList);                    /* Initialize the waiting list                            */
}


/*
************************************************************************************************************************
*                                          ADD/REMOVE MUTEX TO/FROM DEBUG LIST
*
* Description: These functions are called by uC/OS-III to add or remove a mutex to/from the debug list.
*
* Arguments  : p_mutex     is a pointer to the mutex to add/remove
*
* Returns    : none
*
* Note(s)    : These functions are INTERNAL to uC/OS-III and your application should not call it.
************************************************************************************************************************
*/


#if OS_CFG_DBG_EN > 0u
void  OS_MutexDbgListAdd (OS_MUTEX  *p_mutex)
{
    p_mutex->DbgNamePtr               = (CPU_CHAR *)((void *)" ");
    p_mutex->DbgPrevPtr               = (OS_MUTEX *)0;
    if (OSMutexDbgListPtr == (OS_MUTEX *)0) {
        p_mutex->DbgNextPtr           = (OS_MUTEX *)0;
    } else {
        p_mutex->DbgNextPtr           =  OSMutexDbgListPtr;
        OSMutexDbgListPtr->DbgPrevPtr =  p_mutex;
    }
    OSMutexDbgListPtr                 =  p_mutex;
}



void  OS_MutexDbgListRemove (OS_MUTEX  *p_mutex)
{
    OS_MUTEX  *p_mutex_next;
    OS_MUTEX  *p_mutex_prev;


    p_mutex_prev = p_mutex->DbgPrevPtr;
    p_mutex_next = p_mutex->DbgNextPtr;

    if (p_mutex_prev == (OS_MUTEX *)0) {
        OSMutexDbgListPtr = p_mutex_next;
        if (p_mutex_next != (OS_MUTEX *)0) {
            p_mutex_next->DbgPrevPtr = (OS_MUTEX *)0;
        }
        p_mutex->DbgNextPtr = (OS_MUTEX *)0;

    } else if (p_mutex_next == (OS_MUTEX *)0) {
        p_mutex_prev->DbgNextPtr = (OS_MUTEX *)0;
        p_mutex->DbgPrevPtr      = (OS_MUTEX *)0;

    } else {
        p_mutex_prev->DbgNextPtr =  p_mutex_next;
        p_mutex_next->DbgPrevPtr =  p_mutex_prev;
        p_mutex->DbgNextPtr      = (OS_MUTEX *)0;
        p_mutex->DbgPrevPtr      = (OS_MUTEX *)0;
    }
}
#endif


/*
************************************************************************************************************************
*                                                MUTEX INITIALIZATION
*
* Description: This function is called by OSInit() to initialize the mutex management.
*

* Argument(s): p_err        is a pointer to a variable that will contain an error code returned by this function.
*
*                                OS_ERR_NONE     the call was successful
*
* Returns    : none
*
* Note(s)    : 1) This function is INTERNAL to uC/OS-III and your application MUST NOT call it.
************************************************************************************************************************
*/

void  OS_MutexInit (OS_ERR  *p_err)
{
#ifdef OS_SAFETY_CRITICAL
    if (p_err == (OS_ERR *)0) {
        OS_SAFETY_CRITICAL_EXCEPTION();
        return;
    }
#endif

#if OS_CFG_DBG_EN > 0u
    OSMutexDbgListPtr = (OS_MUTEX *)0;
#endif

    OSMutexQty        = (OS_OBJ_QTY)0;
   *p_err             =  OS_ERR_NONE;
}


/*
************************************************************************************************************************
*                                               MUTEX GROUP ADD
*
* Description: This function is called by the kernel to add a mutex to a task's mutex group.
*

* Argument(s): p_tcb        is a pointer to the tcb of the task to give the mutex to.
*
*              p_mutex      is a point to the mutex to add to the group.
*
*
* Returns    : none
*
* Note(s)    : 1) This function is INTERNAL to uC/OS-III and your application MUST NOT call it.
************************************************************************************************************************
*/

void  OS_MutexGrpAdd (OS_TCB  *p_tcb, OS_MUTEX  *p_mutex)
{
    p_mutex->MutexGrpNextPtr = p_tcb->MutexGrpHeadPtr;      /* The mutex grp is not sorted add to head of list.       */
    p_tcb->MutexGrpHeadPtr   = p_mutex;
}


/*
************************************************************************************************************************
*                                              MUTEX GROUP REMOVE
*
* Description: This function is called by the kernel to remove a mutex to a task's mutex group.
*

* Argument(s): p_tcb        is a pointer to the tcb of the task to remove the mutex from.
*
*              p_mutex      is a point to the mutex to remove from the group.
*
*
* Returns    : none
*
* Note(s)    : 1) This function is INTERNAL to uC/OS-III and your application MUST NOT call it.
************************************************************************************************************************
*/

void  OS_MutexGrpRemove (OS_TCB  *p_tcb, OS_MUTEX  *p_mutex)
{
    OS_MUTEX  **pp_mutex;

    pp_mutex = &p_tcb->MutexGrpHeadPtr;

    while(*pp_mutex != p_mutex) {
        pp_mutex = &(*pp_mutex)->MutexGrpNextPtr;
    }

    *pp_mutex = (*pp_mutex)->MutexGrpNextPtr;
}


/*
************************************************************************************************************************
*                                              MUTEX FIND HIGHEST PENDING
*
* Description: This function is called by the kernel to find the highest task pending on any mutex from a group.
*

* Argument(s): p_tcb        is a pointer to the tcb of the task to process.
*
*
* Returns    : Highest priority pending or OS_CFG_PRIO_MAX - 1u if none found.
*
* Note(s)    : 1) This function is INTERNAL to uC/OS-III and your application MUST NOT call it.
************************************************************************************************************************
*/

OS_PRIO  OS_MutexGrpPrioFindHighest (OS_TCB  *p_tcb)
{
    OS_MUTEX     **pp_mutex;
    OS_PRIO        highest_prio;
    OS_PRIO        prio;
    OS_PEND_DATA  *p_head;


    highest_prio = OS_CFG_PRIO_MAX - 1u;
    pp_mutex = &p_tcb->MutexGrpHeadPtr;

    while(*pp_mutex != (OS_MUTEX *)0) {
        p_head = (*pp_mutex)->PendList.HeadPtr;
        if (p_head!= (OS_PEND_DATA *)0) {
            prio = p_head->TCBPtr->Prio;
            if(prio < highest_prio) {
                highest_prio = prio;
            }
        }
        pp_mutex = &(*pp_mutex)->MutexGrpNextPtr;
    }

    return (highest_prio);
}


/*
************************************************************************************************************************
*                                               MUTEX GROUP POST ALL
*
* Description: This function is called by the kernel to post (release) all the mutex from a group. Used when deleting
*              a task.
*

* Argument(s): p_tcb        is a pointer to the tcb of the task to process.
*
*
* Returns    : none.
*
* Note(s)    : 1) This function is INTERNAL to uC/OS-III and your application MUST NOT call it.
************************************************************************************************************************
*/

void  OS_MutexGrpPostAll (OS_TCB  *p_tcb)
{
    OS_MUTEX      *p_mutex;
    OS_MUTEX      *p_mutex_next;
    CPU_TS         ts;
    OS_PEND_LIST  *p_pend_list;
    OS_TCB        *p_tcb_new;


    p_mutex = p_tcb->MutexGrpHeadPtr;

    while(p_mutex != (OS_MUTEX *)0) {

#if (defined(TRACE_CFG_EN) && (TRACE_CFG_EN > 0u))
    TRACE_OS_MUTEX_POST(p_mutex);                               /* Record the event.                                    */
#endif

        p_mutex_next = p_mutex->MutexGrpNextPtr;
        ts           = OS_TS_GET();                             /* Get timestamp                                        */
        p_mutex->TS  = ts;
        OS_MutexGrpRemove(p_tcb,  p_mutex);                     /* Remove mutex from owner's group                      */

        p_pend_list = &p_mutex->PendList;
        if (p_pend_list->NbrEntries == (OS_OBJ_QTY)0) {         /* Any task waiting on mutex?                           */
            p_mutex->OwnerNestingCtr =           0u;            /* Decrement owner's nesting counter                    */
            p_mutex->OwnerTCBPtr     = (OS_TCB *)0;             /* No                                                   */
        } else {
                                                                /* Get TCB from head of pend list                       */
            p_tcb_new = p_pend_list->HeadPtr->TCBPtr;
            p_mutex->OwnerTCBPtr     = p_tcb;                   /* Give mutex to new owner                              */
            p_mutex->OwnerNestingCtr = 1u;
            OS_MutexGrpAdd(p_tcb_new, p_mutex);
                                                                /* Post to mutex                                        */
            OS_Post((OS_PEND_OBJ *)((void *)p_mutex),
                                            p_tcb_new,
                                    (void *)0,
                                            0u,
                                            ts);
        }

        p_mutex = p_mutex_next;
    }

}

#endif                                                      /* OS_CFG_MUTEX_EN                                        */
