/*
************************************************************************************************************************
*                                                      uC/OS-III
*                                                 The Real-Time Kernel
*
*                                  (c) Copyright 2009-2015; Micrium, Inc.; Weston, FL
*                           All rights reserved.  Protected by international copyright laws.
*
*                                               PEND ON MULTIPLE OBJECTS
*
* File    : OS_PEND_MULTI.C
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
const  CPU_CHAR  *os_pend_multi__c = "$Id: $";
#endif


#if (((OS_CFG_Q_EN > 0u) || (OS_CFG_SEM_EN > 0u)) && (OS_CFG_PEND_MULTI_EN > 0u))
/*
************************************************************************************************************************
*                                               PEND ON MULTIPLE OBJECTS
*
* Description: This function pends on multiple objects.  The objects pended on MUST be either semaphores or message
*              queues.  If multiple objects are ready at the start of the pend call, then all available objects that
*              are ready will be indicated to the caller.  If the task must pend on the multiple events then, as soon
*              as one of the object is either posted, aborted or deleted, the task will be readied.
*
*              This function only allows you to pend on semaphores and/or message queues.
*
* Arguments  : p_pend_data_tbl   is a pointer to an array of type OS_PEND_DATA which contains a list of all the
*                                objects we will be waiting on.  The caller must declare an array of OS_PEND_DATA
*                                and initialize the .PendObjPtr (see below) with a pointer to the object (semaphore or
*                                message queue) to pend on.
*
*                                    OS_PEND_DATA  MyPendArray[?];
*
*                                The OS_PEND_DATA field are as follows:
*
*                                    OS_PEND_DATA  *PrevPtr;      Used to link OS_PEND_DATA objects
*                                    OS_PEND_DATA  *NextPtr;      Used to link OS_PEND_DATA objects
*                                    OS_TCB        *TCBPtr;       Pointer to the TCB that is pending on multiple objects
*                                    OS_PEND_OBJ   *PendObjPtr;   USER supplied field which is a pointer to the
*                                                                 semaphore or message queue you want to pend on.  When
*                                                                 you call OSPendMulti() you MUST fill this field for
*                                                                 each of the objects you want to pend on.
*                                    OS_PEND_OBJ   *RdyObjPtr;    OSPendMulti() will return the object that was posted,
*                                                                 aborted or deleted in this field.
*                                    void          *RdyMsgPtr;    OSPendMulti() will fill in this field if the object
*                                                                 posted was a message queue.  This corresponds to the
*                                                                 message posted.
*                                    OS_MSG_SIZE    RdyMsgSize;   OSPendMulti() will fill in this field if the object
*                                                                 posted was a message queue.  This corresponds to the
*                                                                 size of the message posted.
*                                    CPU_TS         RdyTS;        OSPendMulti() will fill in this field if the object
*                                                                 was a message queue.  This corresponds to the time
*                                                                 stamp when the message was posted.  However, if the
*                                                                 object is a semaphore and the object is already ready
*                                                                 the this field will be set to (CPU_TS)0 because it's
*                                                                 not possible to know when the semaphore was posted.
*
*              tbl_size      is the size (in number of elements) of the OS_PEND_DATA array passed to this function.  In
*                            other words, if the called needs to pend on 4 separate objects (semaphores and/or queues)
*                            then you would pass 4 to this call.
*
*              timeout       is an optional timeout period (in clock ticks).  If non-zero, your task will wait any of
*                            the objects up to the amount of time specified by this argument. If you specify 0, however,
*                            your task will wait forever for the specified objects or, until an object is posted,
*                            aborted or deleted.
*
*              opt           determines whether the user wants to block if none of the objects are available.
*
*                                OS_OPT_PEND_BLOCKING
*                                OS_OPT_PEND_NON_BLOCKING
*
*              p_err         is a pointer to where an error message will be deposited.  Possible error messages are:
*
*                                OS_ERR_NONE              The call was successful and your task owns the resources or,
*                                                         the objects you are waiting for occurred. Check the .RdyObjPtr
*                                                         fields to know which objects have been posted.
*                                OS_ERR_OBJ_TYPE          If any of the .PendPtr is NOT a semaphore or a message queue
*                                OS_ERR_OPT_INVALID       If you specified an invalid option for 'opt'
*                                OS_ERR_PEND_ABORT        The wait on the events was aborted; check the .RdyObjPtr fields
*                                                         for which objects were aborted.
*                                OS_ERR_PEND_DEL          The wait on the events was aborted; check the .RdyObjPtr fields
*                                                         for which objects were aborted.
*                                OS_ERR_PEND_ISR          If you called this function from an ISR
*                                OS_ERR_PEND_LOCKED       If you called this function when the scheduler is locked.
*                                OS_ERR_PEND_WOULD_BLOCK  If the caller didn't want to block and no object ready
*                                OS_ERR_STATUS_INVALID    Invalid pend status
*                                OS_ERR_PTR_INVALID       If you passes a NULL pointer of 'p_pend_data_tbl'
*                                OS_ERR_TIMEOUT           The objects were not posted within the specified 'timeout'.
*
* Returns    : >  0          the number of objects returned as ready, aborted or deleted
*              == 0          if no events are returned as ready because of timeout or upon error.
************************************************************************************************************************
*/

OS_OBJ_QTY  OSPendMulti (OS_PEND_DATA  *p_pend_data_tbl,
                         OS_OBJ_QTY     tbl_size,
                         OS_TICK        timeout,
                         OS_OPT         opt,
                         OS_ERR        *p_err)
{
    CPU_BOOLEAN   valid;
    OS_OBJ_QTY    nbr_obj_rdy;
    CPU_SR_ALLOC();



#ifdef OS_SAFETY_CRITICAL
    if (p_err == (OS_ERR *)0) {
        OS_SAFETY_CRITICAL_EXCEPTION();
        return ((OS_OBJ_QTY)0);
    }
#endif

#if OS_CFG_CALLED_FROM_ISR_CHK_EN > 0u
    if (OSIntNestingCtr > (OS_NESTING_CTR)0) {              /* Can't pend from an ISR                                 */
       *p_err = OS_ERR_PEND_ISR;
        return ((OS_OBJ_QTY)0);
    }
#endif

#if OS_CFG_ARG_CHK_EN > 0u
    if (p_pend_data_tbl == (OS_PEND_DATA *)0) {             /* Validate 'p_pend_data_tbl'                             */
       *p_err = OS_ERR_PTR_INVALID;
        return ((OS_OBJ_QTY)0);
    }
    if (tbl_size == (OS_OBJ_QTY)0) {                        /* Array size must be > 0                                 */
       *p_err = OS_ERR_PTR_INVALID;
        return ((OS_OBJ_QTY)0);
    }
    switch (opt) {
        case OS_OPT_PEND_BLOCKING:
        case OS_OPT_PEND_NON_BLOCKING:
             break;

        default:
            *p_err = OS_ERR_OPT_INVALID;
             return ((OS_OBJ_QTY)0);
    }
#endif

    valid = OS_PendMultiValidate(p_pend_data_tbl,           /* -------- Validate objects to be OS_SEM or OS_Q ------- */
                                 tbl_size);
    if (valid == DEF_FALSE) {
       *p_err = OS_ERR_OBJ_TYPE;                            /* Invalid, not OS_SEM or OS_Q                            */
        return ((OS_OBJ_QTY)0);
    }


    CPU_CRITICAL_ENTER();
    nbr_obj_rdy = OS_PendMultiGetRdy(p_pend_data_tbl,       /* --------- SEE IF OBJECT(s) HAVE BEEN POSTED ---------- */
                                     tbl_size);
    if (nbr_obj_rdy > (OS_OBJ_QTY)0) {
        CPU_CRITICAL_EXIT();
       *p_err = OS_ERR_NONE;
        return ((OS_OBJ_QTY)nbr_obj_rdy);
    }

    if ((opt & OS_OPT_PEND_NON_BLOCKING) != (OS_OPT)0) {    /* Caller wants to block if not available?                */
        CPU_CRITICAL_EXIT();
       *p_err = OS_ERR_PEND_WOULD_BLOCK;                    /* No                                                     */
        return ((OS_OBJ_QTY)0);
    } else {
        if (OSSchedLockNestingCtr > (OS_NESTING_CTR)0) {    /* Can't pend when the scheduler is locked                */
            CPU_CRITICAL_EXIT();
           *p_err = OS_ERR_SCHED_LOCKED;
            return ((OS_OBJ_QTY)0);
        }
    }
                                                            /* Lock the scheduler/re-enable interrupts                */
    OS_CRITICAL_ENTER_CPU_EXIT();
                                                            /* ------ NO OBJECT READY, PEND ON MULTIPLE OBJECTS ----- */
    OS_PendMultiWait(p_pend_data_tbl,                       /* Suspend task until object posted or timeout occurs     */
                     tbl_size,
                     timeout);

    OS_CRITICAL_EXIT_NO_SCHED();

    OSSched();                                              /* Find next highest priority task ready                  */

    CPU_CRITICAL_ENTER();
    switch (OSTCBCurPtr->PendStatus) {
        case OS_STATUS_PEND_OK:                             /* We got one of the objects posted to                    */
            *p_err = OS_ERR_NONE;
             break;

        case OS_STATUS_PEND_ABORT:                          /* Indicate that the multi-pend was aborted               */
            *p_err = OS_ERR_PEND_ABORT;
             break;

        case OS_STATUS_PEND_TIMEOUT:                        /* Indicate that we didn't get semaphore within timeout   */
            *p_err = OS_ERR_TIMEOUT;
             break;

        case OS_STATUS_PEND_DEL:                            /* Indicate that an object pended on has been deleted     */
            *p_err = OS_ERR_OBJ_DEL;
            break;

        default:
            *p_err = OS_ERR_STATUS_INVALID;
             break;
    }

    OSTCBCurPtr->PendStatus = OS_STATUS_PEND_OK;
    CPU_CRITICAL_EXIT();

    return ((OS_OBJ_QTY)1);
}


/*
************************************************************************************************************************
*                                              GET A LIST OF OBJECTS READY
*
* Description: This function is called by OSPendMulti() to obtain the list of object that are ready.
*
* Arguments  : p_pend_data_tbl   is a pointer to an array of OS_PEND_DATA
*              ---------------
*
*              tbl_size          is the size of the array
*
* Returns    :  > 0              the number of objects ready
*              == 0              if no object ready
*
* Note       : This function is INTERNAL to uC/OS-III and your application should not call it.
************************************************************************************************************************
*/

OS_OBJ_QTY  OS_PendMultiGetRdy (OS_PEND_DATA  *p_pend_data_tbl,
                                OS_OBJ_QTY     tbl_size)
{
    OS_OBJ_QTY   i;
    OS_OBJ_QTY   nbr_obj_rdy;
#if OS_CFG_Q_EN > 0u
    OS_ERR       err;
    OS_MSG_SIZE  msg_size;
    OS_Q        *p_q;
    void        *p_void;
    CPU_TS       ts;
#endif
#if OS_CFG_SEM_EN  > 0u
    OS_SEM      *p_sem;
#endif



    nbr_obj_rdy = (OS_OBJ_QTY)0;
    for (i = 0u; i < tbl_size; i++) {
        p_pend_data_tbl->RdyObjPtr  = (OS_PEND_OBJ  *)0;         /* Clear all fields                                  */
        p_pend_data_tbl->RdyMsgPtr  = (void         *)0;
        p_pend_data_tbl->RdyMsgSize = (OS_MSG_SIZE   )0;
        p_pend_data_tbl->RdyTS      = (CPU_TS        )0;
        p_pend_data_tbl->NextPtr    = (OS_PEND_DATA *)0;
        p_pend_data_tbl->PrevPtr    = (OS_PEND_DATA *)0;
        p_pend_data_tbl->TCBPtr     = (OS_TCB       *)0;
#if OS_CFG_Q_EN > 0u
        p_q = (OS_Q *)((void *)p_pend_data_tbl->PendObjPtr);     /* Assume we are pointing to a message queue object  */
        if (p_q->Type == OS_OBJ_TYPE_Q) {                        /* Is it a message queue?                            */
            p_void = OS_MsgQGet(&p_q->MsgQ,                      /* Yes, Any message waiting in the message queue?    */
                                &msg_size,
                                &ts,
                                &err);
            if (err == OS_ERR_NONE) {
                p_pend_data_tbl->RdyObjPtr  = p_pend_data_tbl->PendObjPtr;
                p_pend_data_tbl->RdyMsgPtr  = p_void;            /*      Yes, save the message received               */
                p_pend_data_tbl->RdyMsgSize = msg_size;
                p_pend_data_tbl->RdyTS      = ts;
                nbr_obj_rdy++;
            }
        }
#endif

#if OS_CFG_SEM_EN > 0u
        p_sem = (OS_SEM *)((void *)p_pend_data_tbl->PendObjPtr); /* Assume we are pointing to a semaphore object      */
        if (p_sem->Type == OS_OBJ_TYPE_SEM) {                    /* Is it a semaphore?                                */
            if (p_sem->Ctr > 0u) {                               /* Yes, Semaphore has been signaled?                 */
                p_sem->Ctr--;                                    /*      Yes, caller may proceed                      */
                p_pend_data_tbl->RdyObjPtr  = p_pend_data_tbl->PendObjPtr;
                p_pend_data_tbl->RdyTS      = p_sem->TS;
                nbr_obj_rdy++;
            }
        }
#endif

        p_pend_data_tbl++;
    }
    return (nbr_obj_rdy);
}


/*
************************************************************************************************************************
*                                 VERIFY THAT OBJECTS PENDED ON ARE EITHER SEMAPHORES or QUEUES
*
* Description: This function is called by OSPendMulti() to verify that we are multi-pending on either semaphores or
*              message queues.
*
* Arguments  : p_pend_data_tbl    is a pointer to an array of OS_PEND_DATA
*              ---------------
*
*              tbl_size           is the size of the array
*
* Returns    : TRUE               if all objects pended on are either semaphores of queues
*              FALSE              if at least one object is not a semaphore or queue.
*
* Note       : This function is INTERNAL to uC/OS-III and your application should not call it.
************************************************************************************************************************
*/

CPU_BOOLEAN  OS_PendMultiValidate (OS_PEND_DATA  *p_pend_data_tbl,
                                   OS_OBJ_QTY     tbl_size)
{
    OS_OBJ_QTY  i;
    OS_OBJ_QTY  ctr;
#if OS_CFG_SEM_EN  > 0u
    OS_SEM      *p_sem;
#endif
#if OS_CFG_Q_EN > 0u
    OS_Q        *p_q;
#endif


    for (i = 0u; i < tbl_size; i++) {
        if (p_pend_data_tbl->PendObjPtr == (OS_PEND_OBJ *)0) {   /* All .PendObjPtr in the table MUST be non NULL     */
            return (DEF_FALSE);
        }

        ctr = 0u;
#if OS_CFG_SEM_EN  > 0u
        p_sem = (OS_SEM *)((void *)p_pend_data_tbl->PendObjPtr); /* All objects to pend on must be of type OS_SEM ... */
        if (p_sem->Type == OS_OBJ_TYPE_SEM) {
            ctr++;
        }
#endif

#if OS_CFG_Q_EN > 0u
        p_q = (OS_Q *)((void *)p_pend_data_tbl->PendObjPtr);     /* ... or of type OS_Q                               */
        if (p_q->Type == OS_OBJ_TYPE_Q) {
            ctr++;
        }
#endif

        if (ctr == (OS_OBJ_QTY)0) {
            return (DEF_FALSE);                                  /* Found at least one invalid object type            */
        }
        p_pend_data_tbl++;
    }
    return (DEF_TRUE);
}


/*
************************************************************************************************************************
*                                 MAKE TASK WAIT FOR ANY OF MULTIPLE EVENTS TO OCCUR
*
* Description: This function is called by OSPendMulti() to suspend a task because any one of multiple objects that have
*              not been posted to.
*
* Arguments  : p_pend_data_tbl    is a pointer to an array of OS_PEND_DATA
*              ---------------
*
*              tbl_size           is the size of the array
*
*              timeout            is the timeout to wait in case none of the objects become ready
*
* Returns    : none
*
* Note       : This function is INTERNAL to uC/OS-III and your application should not call it.
************************************************************************************************************************
*/

void  OS_PendMultiWait (OS_PEND_DATA  *p_pend_data_tbl,
                        OS_OBJ_QTY     tbl_size,
                        OS_TICK        timeout)
{
    OS_OBJ_QTY      i;
    OS_PEND_LIST   *p_pend_list;

#if OS_CFG_Q_EN > 0u
    OS_Q           *p_q;
#endif

#if OS_CFG_SEM_EN > 0u
    OS_SEM         *p_sem;
#endif



    OSTCBCurPtr->PendOn             = OS_TASK_PEND_ON_MULTI;   /* Resource not available, wait until it is            */
    OSTCBCurPtr->PendStatus         = OS_STATUS_PEND_OK;
    OSTCBCurPtr->PendDataTblEntries = tbl_size;
    OSTCBCurPtr->PendDataTblPtr     = p_pend_data_tbl;

    OS_TaskBlock(OSTCBCurPtr,                                  /* Block the task waiting for object to be posted ...  */
                 timeout);                                     /* ... but with a timeout if not                       */

    for (i = 0u; i < tbl_size; i++) {
        p_pend_data_tbl->TCBPtr = OSTCBCurPtr;                 /* Every entry points back to the TCB of the task      */

#if OS_CFG_SEM_EN > 0u
        p_sem = (OS_SEM *)((void *)p_pend_data_tbl->PendObjPtr);
        if (p_sem->Type == OS_OBJ_TYPE_SEM) {
            p_pend_list = &p_sem->PendList;
            OS_PendListInsertPrio(p_pend_list,
                                  p_pend_data_tbl);
        }
#endif

#if OS_CFG_Q_EN > 0u
        p_q = (OS_Q *)((void *)p_pend_data_tbl->PendObjPtr);
        if (p_q->Type == OS_OBJ_TYPE_Q) {
            p_pend_list = &p_q->PendList;
            OS_PendListInsertPrio(p_pend_list,
                                  p_pend_data_tbl);
        }
#endif

        p_pend_data_tbl++;
    }
}

#endif
