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
*/

/**
 * @file    arch/stm32/stm32f4_chibios_vectors.c
 *
 * File is modified to route IRQ_HANDLERS to opencm3 handlers
 */


#include "stm32f4_chibios_vectors.h"
#include "mcuconf.h"

#include <ch.h>

/**
 * @brief   Type of an IRQ vector.
 */
typedef void (*irq_vector_t)(void);
enum HardwareFaultType {HardwareFault_NONE, HardwareFault_BUS, HardwareFault_MEM, HardwareFault_USAGE};
static enum HardwareFaultType hardwareFaultType = HardwareFault_NONE;
void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress)  __attribute__((unused));

/**
 * @brief   Type of a structure representing the whole vectors table.
 */
typedef struct {
  uint32_t      *init_stack;
  irq_vector_t  reset_vector;
  irq_vector_t  nmi_vector;
  irq_vector_t  hardfault_vector;
  irq_vector_t  memmanage_vector;
  irq_vector_t  busfault_vector;
  irq_vector_t  usagefault_vector;
  irq_vector_t  vector1c;
  irq_vector_t  vector20;
  irq_vector_t  vector24;
  irq_vector_t  vector28;
  irq_vector_t  svcall_vector;
  irq_vector_t  debugmonitor_vector;
  irq_vector_t  vector34;
  irq_vector_t  pendsv_vector;
  irq_vector_t  systick_vector;
  irq_vector_t  vectors[82];
} vectors_t;

#if !defined(__DOXYGEN__)




extern uint32_t __main_stack_end__;
extern void ResetHandler(void);
extern void NMIVector(void);
extern void HardFaultVector(void);
extern void MemManageVector(void);
extern void BusFaultVector(void);
extern void UsageFaultVector(void);
extern void Vector1C(void);
extern void Vector20(void);
extern void Vector24(void);
extern void Vector28(void);
extern void SVCallVector(void);
extern void DebugMonitorVector(void);
extern void Vector34(void);
extern void PendSVVector(void);
extern void SysTickVector(void);
extern void Vector40(void);
extern void Vector44(void);
extern void Vector48(void);
extern void Vector4C(void);
extern void Vector50(void);
extern void Vector54(void);
extern void Vector58(void);
extern void Vector5C(void);
extern void Vector60(void);
extern void Vector64(void);
extern void Vector68(void);
extern void Vector6C(void);
extern void Vector70(void);
extern void Vector74(void);
extern void Vector78(void);
extern void Vector7C(void);
extern void Vector80(void);
extern void Vector84(void);
extern void Vector88(void);
extern void Vector8C(void);
extern void Vector90(void);
extern void Vector94(void);
extern void Vector98(void);
extern void Vector9C(void);
extern void VectorA0(void);
extern void VectorA4(void);
extern void VectorA8(void);
extern void VectorAC(void);
extern void VectorB0(void);
extern void VectorB4(void);
extern void VectorB8(void);
extern void VectorBC(void);
extern void VectorC0(void);
extern void VectorC4(void);
extern void VectorC8(void);
extern void VectorCC(void);
extern void VectorD0(void);
extern void VectorD4(void);
extern void VectorD8(void);
extern void VectorDC(void);
extern void VectorE0(void);
extern void VectorE4(void);
extern void VectorE8(void);
extern void VectorEC(void);
extern void VectorF0(void);
extern void VectorF4(void);
extern void VectorF8(void);
extern void VectorFC(void);
extern void Vector100(void);
extern void Vector104(void);
extern void Vector108(void);
extern void Vector10C(void);
extern void Vector110(void);
extern void Vector114(void);
extern void Vector118(void);
extern void Vector11C(void);
extern void Vector120(void);
extern void Vector124(void);
extern void Vector128(void);
extern void Vector12C(void);
extern void Vector130(void);
extern void Vector134(void);
extern void Vector138(void);
extern void Vector13C(void);
extern void Vector140(void);
extern void Vector144(void);
extern void Vector148(void);
extern void Vector14C(void);
extern void Vector150(void);
extern void Vector154(void);
extern void Vector158(void);
extern void Vector15C(void);
extern void Vector160(void);
extern void Vector164(void);
extern void Vector168(void);
extern void Vector16C(void);
extern void Vector170(void);
extern void Vector174(void);
extern void Vector178(void);
extern void Vector17C(void);
extern void Vector180(void);
extern void Vector184(void);
#endif

/**
 * @brief   STM32 vectors table.
 */
#if !defined(__DOXYGEN__)
__attribute__((used, section("vectors")))
#endif
vectors_t _vectors = {
  &__main_stack_end__, ResetHandler,       NMIVector,          HardFaultVector,
  MemManageVector,    BusFaultVector,     UsageFaultVector,   Vector1C,
  Vector20,           Vector24,           Vector28,           SVCallVector,
  DebugMonitorVector, Vector34,           PendSVVector,       SysTickVector,
  {
    DECLARE_IRQS
  }
};

/**
 * @brief   Unhandled exceptions handler.
 * @details Any undefined exception vector points to this function by default.
 *          This function simply stops the system into an infinite loop.
 *
 * @notapi
 */



static void  _unhandled_exception(void) __attribute__((naked));


void _unhandled_exception(void)
{
  __asm volatile
  (
    " tst lr, #4                                                \n"
    " ite eq                                                    \n"
    " mrseq r0, msp                                             \n"
    " mrsne r0, psp                                             \n"
    " ldr r1, [r0, #24]                                         \n"
    " ldr r2, handler2_address_const                            \n"
    " bx r2                                                     \n"
    " handler2_address_const: .word prvGetRegistersFromStack    \n"
  );
}


void _unhandled_exception_NMIVector(void)
{
  while (TRUE);
}

void _unhandled_exception_MemManageVector(void)
{
  hardwareFaultType = HardwareFault_MEM;
  _unhandled_exception();
}
void _unhandled_exception_BusFaultVector(void)
{
  hardwareFaultType = HardwareFault_BUS;
  _unhandled_exception();
}
void _unhandled_exception_UsageFaultVector(void)
{
  hardwareFaultType = HardwareFault_USAGE;
  _unhandled_exception();
}

void NMIVector(void) __attribute__((weak, alias("_unhandled_exception_NMIVector")));
void HardFaultVector(void) __attribute__((weak, alias("_unhandled_exception")));
void MemManageVector(void) __attribute__((weak, alias("_unhandled_exception_MemManageVector")));
void BusFaultVector(void) __attribute__((weak, alias("_unhandled_exception_BusFaultVector")));
void UsageFaultVector(void) __attribute__((weak, alias("_unhandled_exception_UsageFaultVector")));
void Vector1C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector20(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector24(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector28(void) __attribute__((weak, alias("_unhandled_exception")));
void SVCallVector(void) __attribute__((weak, alias("_unhandled_exception")));
void DebugMonitorVector(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector34(void) __attribute__((weak, alias("_unhandled_exception")));
void PendSVVector(void) __attribute__((weak, alias("_unhandled_exception")));
void SysTickVector(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector40(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector44(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector48(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector4C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector50(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector54(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector58(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector5C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector60(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector64(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector68(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector6C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector70(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector74(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector78(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector7C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector80(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector84(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector88(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector8C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector90(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector94(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector98(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector9C(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorA0(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorA4(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorA8(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorAC(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorB0(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorB4(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorB8(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorBC(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorC0(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorC4(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorC8(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorCC(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorD0(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorD4(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorD8(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorDC(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorE0(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorE4(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorE8(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorEC(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorF0(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorF4(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorF8(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorFC(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector100(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector104(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector108(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector10C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector110(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector114(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector118(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector11C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector120(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector124(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector128(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector12C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector130(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector134(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector138(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector13C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector140(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector144(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector148(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector14C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector150(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector154(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector158(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector15C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector160(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector164(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector168(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector16C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector170(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector174(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector178(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector17C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector180(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector184(void) __attribute__((weak, alias("_unhandled_exception")));


void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress)
{
  /* These are volatile to try and prevent the compiler/linker optimising them
     away as the variables never actually get used.  If the debugger won't show the
     values of the variables, make them global my moving their declaration outside
     of this function. */

  /*
    When entering hard fault, the stm32 push the value of register on the stack, and loop forever.
    You can easily retrieve address of faulty instruction in variable pc,
    status in variable psr.

    in gdb, after when hard-fault is triggered, you just have to type some commands :

    ° print /x pc => retrieve instruction
    ° info line *pc : see the source code of the line which has triggered exception
    ° disassemble pc : see the assembly of the address which has triggered exception
   */

  volatile uint32_t r0 __attribute__((unused));
  volatile uint32_t r1 __attribute__((unused));
  volatile uint32_t r2 __attribute__((unused));
  volatile uint32_t r3 __attribute__((unused));
  volatile uint32_t r12 __attribute__((unused));
  volatile uint32_t lr __attribute__((unused)); /* Link register. */
  volatile uint32_t pc __attribute__((unused)); /* Program counter. */
  volatile uint32_t psr __attribute__((unused));/* Program status register. */

  r0 = pulFaultStackAddress[ 0 ];
  r1 = pulFaultStackAddress[ 1 ];
  r2 = pulFaultStackAddress[ 2 ];
  r3 = pulFaultStackAddress[ 3 ];

  r12 = pulFaultStackAddress[ 4 ];
  lr = pulFaultStackAddress[ 5 ];
  pc = pulFaultStackAddress[ 6 ];
  psr = pulFaultStackAddress[ 7 ];

  /* When the following line is hit, the variables contain the register values. */
  while (1);
}

