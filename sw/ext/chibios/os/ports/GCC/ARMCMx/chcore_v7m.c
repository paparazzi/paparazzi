/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012,2013 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

/**
 * @file    GCC/ARMCMx/chcore_v7m.c
 * @brief   ARMv7-M architecture port code.
 *
 * @addtogroup ARMCMx_V7M_CORE
 * @{
 */

#include "ch.h"

/*===========================================================================*/
/* Port interrupt handlers.                                                  */
/*===========================================================================*/

/**
 * @brief   System Timer vector.
 * @details This interrupt is used as system tick.
 * @note    The timer must be initialized in the startup code.
 */
CH_IRQ_HANDLER(SysTickVector) {

  CH_IRQ_PROLOGUE();

  chSysLockFromIsr();
  chSysTimerHandlerI();
  chSysUnlockFromIsr();

  CH_IRQ_EPILOGUE();
}

#if !CORTEX_SIMPLIFIED_PRIORITY || defined(__DOXYGEN__)
/**
 * @brief   SVC vector.
 * @details The SVC vector is used for exception mode re-entering after a
 *          context switch.
 * @note    The PendSV vector is only used in advanced kernel mode.
 */
void SVCallVector(void) {
  struct extctx *ctxp;

  /* Current PSP value.*/
  asm volatile ("mrs     %0, PSP" : "=r" (ctxp) : : "memory");

  /* Discarding the current exception context and positioning the stack to
     point to the real one.*/
  ctxp++;

#if CORTEX_USE_FPU
  /* Restoring the special register SCB_FPCCR.*/
  SCB_FPCCR = (uint32_t)ctxp->fpccr;
  SCB_FPCAR = SCB_FPCAR + sizeof (struct extctx);
#endif
  asm volatile ("msr     PSP, %0" : : "r" (ctxp) : "memory");
  port_unlock_from_isr();
}
#endif /* !CORTEX_SIMPLIFIED_PRIORITY */

#if CORTEX_SIMPLIFIED_PRIORITY || defined(__DOXYGEN__)
/**
 * @brief   PendSV vector.
 * @details The PendSV vector is used for exception mode re-entering after a
 *          context switch.
 * @note    The PendSV vector is only used in compact kernel mode.
 */
void PendSVVector(void) {
  struct extctx *ctxp;

  /* Current PSP value.*/
  asm volatile ("mrs     %0, PSP" : "=r" (ctxp) : : "memory");

  /* Discarding the current exception context and positioning the stack to
     point to the real one.*/
  ctxp++;

#if CORTEX_USE_FPU
  /* Restoring the special register SCB_FPCCR.*/
  SCB_FPCCR = (uint32_t)ctxp->fpccr;
  SCB_FPCAR = SCB_FPCAR + sizeof (struct extctx);
#endif
  asm volatile ("msr     PSP, %0" : : "r" (ctxp) : "memory");
}
#endif /* CORTEX_SIMPLIFIED_PRIORITY */

/*===========================================================================*/
/* Port exported functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Port-related initialization code.
 */
void _port_init(void) {

  /* Initialization of the vector table and priority related settings.*/
  SCB_VTOR = CORTEX_VTOR_INIT;
  SCB_AIRCR = AIRCR_VECTKEY | AIRCR_PRIGROUP(CORTEX_PRIGROUP_INIT);

  /* Initialization of the system vectors used by the port.*/
  nvicSetSystemHandlerPriority(HANDLER_SVCALL,
    CORTEX_PRIORITY_MASK(CORTEX_PRIORITY_SVCALL));
  nvicSetSystemHandlerPriority(HANDLER_PENDSV,
    CORTEX_PRIORITY_MASK(CORTEX_PRIORITY_PENDSV));
  nvicSetSystemHandlerPriority(HANDLER_SYSTICK,
    CORTEX_PRIORITY_MASK(CORTEX_PRIORITY_SYSTICK));
}

#if !CH_OPTIMIZE_SPEED
void _port_lock(void) {
  register uint32_t tmp asm ("r3") = CORTEX_BASEPRI_KERNEL;
  asm volatile ("msr     BASEPRI, %0" : : "r" (tmp) : "memory");
}

void _port_unlock(void) {
  register uint32_t tmp asm ("r3") = CORTEX_BASEPRI_DISABLED;
  asm volatile ("msr     BASEPRI, %0" : : "r" (tmp) : "memory");
}
#endif

/**
 * @brief   Exception exit redirection to _port_switch_from_isr().
 */
void _port_irq_epilogue(void) {

  port_lock_from_isr();
  if ((SCB_ICSR & ICSR_RETTOBASE) != 0) {
    struct extctx *ctxp;

    /* Current PSP value.*/
    asm volatile ("mrs     %0, PSP" : "=r" (ctxp) : : "memory");

    /* Adding an artificial exception return context, there is no need to
       populate it fully.*/
    ctxp--;
    asm volatile ("msr     PSP, %0" : : "r" (ctxp) : "memory");
    ctxp->xpsr = (regarm_t)0x01000000;

    /* The exit sequence is different depending on if a preemption is
       required or not.*/
    if (chSchIsPreemptionRequired()) {
      /* Preemption is required we need to enforce a context switch.*/
      ctxp->pc = (void *)_port_switch_from_isr;
#if CORTEX_USE_FPU
      /* Triggering a lazy FPU state save.*/
      asm volatile ("vmrs    APSR_nzcv, FPSCR" : : : "memory");
#endif
    }
    else {
      /* Preemption not required, we just need to exit the exception
         atomically.*/
      ctxp->pc = (void *)_port_exit_from_isr;
    }

#if CORTEX_USE_FPU
    {
      uint32_t fpccr;

      /* Saving the special register SCB_FPCCR into the reserved offset of
         the Cortex-M4 exception frame.*/
      (ctxp + 1)->fpccr = (regarm_t)(fpccr = SCB_FPCCR);

      /* Now the FPCCR is modified in order to not restore the FPU status
         from the artificial return context.*/
      SCB_FPCCR = fpccr | FPCCR_LSPACT;
    }
#endif

    /* Note, returning without unlocking is intentional, this is done in
       order to keep the rest of the context switch atomic.*/
    return;
  }
  port_unlock_from_isr();
}

/**
 * @brief   Post-IRQ switch code.
 * @details Exception handlers return here for context switching.
 */
#if !defined(__DOXYGEN__)
__attribute__((naked))
#endif
void _port_switch_from_isr(void) {

  dbg_check_lock();
  chSchDoReschedule();
  dbg_check_unlock();
  asm volatile ("_port_exit_from_isr:" : : : "memory");
#if !CORTEX_SIMPLIFIED_PRIORITY || defined(__DOXYGEN__)
  asm volatile ("svc     #0");
#else /* CORTEX_SIMPLIFIED_PRIORITY */
  SCB_ICSR = ICSR_PENDSVSET;
  port_unlock();
  while (TRUE)
    ;
#endif /* CORTEX_SIMPLIFIED_PRIORITY */
}

/**
 * @brief   Performs a context switch between two threads.
 * @details This is the most critical code in any port, this function
 *          is responsible for the context switch between 2 threads.
 * @note    The implementation of this code affects <b>directly</b> the context
 *          switch performance so optimize here as much as you can.
 *
 * @param[in] ntp       the thread to be switched in
 * @param[in] otp       the thread to be switched out
 */
#if !defined(__DOXYGEN__)
__attribute__((naked))
#endif
void _port_switch(Thread *ntp, Thread *otp) {

  asm volatile ("push    {r4, r5, r6, r7, r8, r9, r10, r11, lr}"
                : : : "memory");
#if CORTEX_USE_FPU
  asm volatile ("vpush   {s16-s31}" : : : "memory");
#endif

  asm volatile ("str     sp, [%1, #12]                          \n\t"
                "ldr     sp, [%0, #12]" : : "r" (ntp), "r" (otp));

#if CORTEX_USE_FPU
  asm volatile ("vpop    {s16-s31}" : : : "memory");
#endif
  asm volatile ("pop     {r4, r5, r6, r7, r8, r9, r10, r11, pc}"
                : : : "memory");
}

/**
 * @brief   Start a thread by invoking its work function.
 * @details If the work function returns @p chThdExit() is automatically
 *          invoked.
 */
void _port_thread_start(void) {

  chSysUnlock();
  asm volatile ("mov     r0, r5                                 \n\t"
                "blx     r4                                     \n\t"
                "bl      chThdExit");
}

/** @} */
