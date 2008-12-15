/******************************************************************************
 *
 * $RCSfile$
 * $Revision$
 *
 * Header file for Philips LPC ARM Processors.
 * Copyright 2004 R O SoftWare
 *
 * No guarantees, warrantees, or promises, implied or otherwise.
 * May be used for hobby or commercial purposes provided copyright
 * notice remains intact.
 *
 *****************************************************************************/
#ifndef INC_LPC_TMR_H
#define INC_LPC_TMR_H

// Timer & PWM Registers
typedef struct
{
  REG32 ir;                             // Interrupt Register
  REG32 tcr;                            // Timer Control Register
  REG32 tc;                             // Timer Counter
  REG32 pr;                             // Prescale Register
  REG32 pc;                             // Prescale Counter Register
  REG32 mcr;                            // Match Control Register
  REG32 mr0;                            // Match Register 0
  REG32 mr1;                            // Match Register 1
  REG32 mr2;                            // Match Register 2
  REG32 mr3;                            // Match Register 3
  REG32 ccr;                            // Capture Control Register
  REG32 cr0;                            // Capture Register 0
  REG32 cr1;                            // Capture Register 1
  REG32 cr2;                            // Capture Register 2
  REG32 cr3;                            // Capture Register 3
  REG32 emr;                            // External Match Register
  REG32 mr4;                            // Match Register 4
  REG32 mr5;                            // Match Register 5
  REG32 mr6;                            // Match Register 6
  REG32 pcr;                            // Control Register
  REG32 ler;                            // Latch Enable Register
} pwmTmrRegs_t;

// Timer Interrupt Register Bit Definitions
#define TIR_MR0I    (1 << 0)            // Interrupt flag for match channel 0
#define TIR_MR1I    (1 << 1)            // Interrupt flag for match channel 1
#define TIR_MR2I    (1 << 2)            // Interrupt flag for match channel 2
#define TIR_MR3I    (1 << 3)            // Interrupt flag for match channel 3
#define TIR_CR0I    (1 << 4)            // Interrupt flag for capture channel 0 event
#define TIR_CR1I    (1 << 5)            // Interrupt flag for capture channel 1 event
#define TIR_CR2I    (1 << 6)            // Interrupt flag for capture channel 2 event
#define TIR_CR3I    (1 << 7)            // Interrupt flag for capture channel 3 event

// Timer Control Register Bit Definitions
#define TCR_ENABLE  (1 << 0)
#define TCR_RESET   (1 << 1)


// Timer Match Control Register Bit Definitions
#define TMCR_MR0_I  (1 << 0)            // Enable Interrupt when MR0 matches TC
#define TMCR_MR0_R  (1 << 1)            // Enable Reset of TC upon MR0 match
#define TMCR_MR0_S  (1 << 2)            // Enable Stop of TC upon MR0 match
#define TMCR_MR1_I  (1 << 3)            // Enable Interrupt when MR1 matches TC
#define TMCR_MR1_R  (1 << 4)            // Enable Reset of TC upon MR1 match
#define TMCR_MR1_S  (1 << 5)            // Enable Stop of TC upon MR1 match
#define TMCR_MR2_I  (1 << 6)            // Enable Interrupt when MR2 matches TC
#define TMCR_MR2_R  (1 << 7)            // Enable Reset of TC upon MR2 match
#define TMCR_MR2_S  (1 << 8)            // Enable Stop of TC upon MR2 match
#define TMCR_MR3_I  (1 << 9)            // Enable Interrupt when MR3 matches TC
#define TMCR_MR3_R  (1 << 10)           // Enable Reset of TC upon MR3 match
#define TMCR_MR3_S  (1 << 11)           // Enable Stop of TC upon MR3 match

/* PWMIR ( Interrupt Register ) bits definitions      */
#define PWMIR_MR0I  _BV(0)            /* Interrupt flag for match channel 0 */
#define PWMIR_MR1I  _BV(1)            /* Interrupt flag for match channel 1 */
#define PWMIR_MR2I  _BV(2)            /* Interrupt flag for match channel 2 */
#define PWMIR_MR3I  _BV(3)            /* Interrupt flag for match channel 3 */
#define PWMIR_MR4I  _BV(8)            /* Interrupt flag for match channel 4 */
#define PWMIR_MR5I  _BV(9)            /* Interrupt flag for match channel 5 */
#define PWMIR_MR6I  _BV(10)           /* Interrupt flag for match channel 6 */
#define PWMIR_MASK  (0x070F)

/* PWMTCR ( Timer Control Register ) bits definitions */
#define PWMTCR_COUNTER_ENABLE _BV(0) /* enable PWM timer counter */
#define PWMTCR_COUNTER_RESET  _BV(1) /* reset PWM timer counter  */
#define PWMTCR_PWM_ENABLE     _BV(3) /* enable PWM mode          */

/* PWMMCR ( Match Control Register ) bits definitions */
#define PWMMCR_MR0I (1 << 0)   /* enable interrupt on match channel 0 */
#define PWMMCR_MR0R (1 << 1)   /* enable reset on match channel 0     */
#define PWMMCR_MR0S (1 << 2)   /* enable stop on match channel 0      */
#define PWMMCR_MR1I (1 << 3)   /* enable interrupt on match channel 1 */
#define PWMMCR_MR1R (1 << 4)   /* enable reset on match channel 1     */
#define PWMMCR_MR1S (1 << 5)   /* enable stop on match channel 1      */
#define PWMMCR_MR2I (1 << 6)   /* enable interrupt on match channel 2 */
#define PWMMCR_MR2R (1 << 7)   /* enable reset on match channel 2     */
#define PWMMCR_MR2S (1 << 8)   /* enable stop on match channel 2      */
#define PWMMCR_MR3I (1 << 9)   /* enable interrupt on match channel 3 */
#define PWMMCR_MR3R (1 << 10)  /* enable reset on match channel 3     */
#define PWMMCR_MR3S (1 << 11)  /* enable stop on match channel 3      */
#define PWMMCR_MR4I (1 << 12)  /* enable interrupt on match channel 4 */
#define PWMMCR_MR4R (1 << 13)  /* enable reset on match channel 4     */
#define PWMMCR_MR4S (1 << 14)  /* enable stop on match channel 4      */
#define PWMMCR_MR5I (1 << 15)  /* enable interrupt on match channel 5 */
#define PWMMCR_MR5R (1 << 16)  /* enable reset on match channel 5     */
#define PWMMCR_MR5S (1 << 17)  /* enable stop on match channel 5      */
#define PWMMCR_MR6I (1 << 18)  /* enable interrupt on match channel 6 */
#define PWMMCR_MR6R (1 << 19)  /* enable reset on match channel 6     */
#define PWMMCR_MR6S (1 << 20)  /* enable stop on match channel 6      */

/* PWMPCR ( Control Register ) bit definitions */
#define PWMPCR_SEL2 _BV(2)     /* select double edge for PWM2 output  */
#define PWMPCR_SEL3 _BV(3)     /* select double edge for PWM3 output  */
#define PWMPCR_SEL4 _BV(4)     /* select double edge for PWM4 output  */
#define PWMPCR_SEL5 _BV(5)     /* select double edge for PWM5 output  */
#define PWMPCR_SEL6 _BV(6)     /* select double edge for PWM6 output  */
#define PWMPCR_ENA1 _BV(9)     /* PWM1 output enabled                 */
#define PWMPCR_ENA2 _BV(10)    /* PWM2 output enabled                 */
#define PWMPCR_ENA3 _BV(11)    /* PWM3 output enabled                 */
#define PWMPCR_ENA4 _BV(12)    /* PWM4 output enabled                 */
#define PWMPCR_ENA5 _BV(13)    /* PWM5 output enabled                 */
#define PWMPCR_ENA6 _BV(14)    /* PWM6 output enabled                 */

/* PWMLER ( Latch Enable Register ) bit definitions */
#define PWMLER_LATCH0 _BV(0)   /* latch last MATCH0 register value    */
#define PWMLER_LATCH1 _BV(1)   /* latch last MATCH1 register value    */
#define PWMLER_LATCH2 _BV(2)   /* latch last MATCH2 register value    */
#define PWMLER_LATCH3 _BV(3)   /* latch last MATCH3 register value    */
#define PWMLER_LATCH4 _BV(4)   /* latch last MATCH4 register value    */
#define PWMLER_LATCH5 _BV(5)   /* latch last MATCH5 register value    */
#define PWMLER_LATCH6 _BV(6)   /* latch last MATCH6 register value    */


// Timer Capture Control Register Bit Definitions
#define TCCR_CR0_R (1 << 0)            // Enable Rising edge on CAPn.0 will load TC to CR0
#define TCCR_CR0_F (1 << 1)            // Enable Falling edge on CAPn.0 will load TC to CR0
#define TCCR_CR0_I (1 << 2)            // Enable Interrupt on load of CR0
#define TCCR_CR1_R (1 << 3)            // Enable Rising edge on CAPn.1 will load TC to CR1
#define TCCR_CR1_F (1 << 4)            // Enable Falling edge on CAPn.1 will load TC to CR1
#define TCCR_CR1_I (1 << 5)            // Enable Interrupt on load of CR1
#define TCCR_CR2_R (1 << 6)            // Enable Rising edge on CAPn.2 will load TC to CR2
#define TCCR_CR2_F (1 << 7)            // Enable Falling edge on CAPn.2 will load TC to CR2
#define TCCR_CR2_I (1 << 8)            // Enable Interrupt on load of CR2
#define TCCR_CR3_R (1 << 9)            // Enable Rising edge on CAPn.3 will load TC to CR3
#define TCCR_CR3_F (1 << 10)           // Enable Falling edge on CAPn.3 will load TC to CR3
#define TCCR_CR3_I (1 << 11)           // Enable Interrupt on load of CR3


// Timer External Match Register
#define TEMR_EM0    (1 << 0)             // reflects state of output match 0
#define TEMR_EM1    (1 << 1)             // reflects state of output match 1
#define TEMR_EM2    (1 << 2)             // reflects state of output match 2
#define TEMR_EM3    (1 << 3)             // reflects state of output match 3
#define TEMR_EMC0_0 (0 << 4)             // configure match 0 pin behaviour
#define TEMR_EMC0_1 (1 << 4)             // configure match 0 pin behaviour
#define TEMR_EMC0_2 (2 << 4)             // configure match 0 pin behaviour
#define TEMR_EMC0_3 (3 << 4)             // configure match 0 pin behaviour
#define TEMR_EMC1_0 (0 << 6)             // configure match 1 pin behaviour
#define TEMR_EMC1_1 (1 << 6)             // configure match 1 pin behaviour
#define TEMR_EMC1_2 (2 << 6)             // configure match 1 pin behaviour
#define TEMR_EMC1_3 (3 << 6)             // configure match 0 pin behaviour
#define TEMR_EMC2_0 (0 << 8)             // configure match 1 pin behaviour
#define TEMR_EMC2_1 (1 << 8)             // configure match 1 pin behaviour
#define TEMR_EMC2_2 (2 << 8)             // configure match 1 pin behaviour
#define TEMR_EMC2_3 (3 << 8)             // configure match 0 pin behaviour
#define TEMR_EMC3_0 (0 << 10)            // configure match 1 pin behaviour
#define TEMR_EMC3_1 (1 << 10)            // configure match 1 pin behaviour
#define TEMR_EMC3_2 (2 << 10)            // configure match 1 pin behaviour
#define TEMR_EMC3_3 (3 << 10)            // configure match 0 pin behaviour


#endif
