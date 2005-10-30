/******************************************************************************
 *
 * $RCSfile$
 * $Revision$
 *
 * This module provides the interface definitions for sysTime.c
 * Copyright 2004, R O SoftWare
 * No guarantees, warrantees, or promises, implied or otherwise.
 * May be used for hobby or commercial purposes provided copyright
 * notice remains intact.
 *
 *****************************************************************************/
#ifndef INC_SYS_TIME_H
#define INC_SYS_TIME_H

#include "types.h"
#include "LPC21xx.h"
#include "config.h"

// Note: with a PCLK = CCLK/2 = 60MHz/2 and a Prescale divider of 3, we
// have a resolution of 100nSec.  Given the timer's counter register is
// 32-bits, we must make a call to one of the sysTime functions at least
// every ~430 sec.

// setup parameters
#define T0_PCLK_DIV     3
#define sysTICSperSEC   (PCLK / T0_PCLK_DIV)

// some helpful times for pause()
#define ONE_MS          (uint32_t)((  1e-3 * sysTICSperSEC) + .5)
#define TWO_MS          (uint32_t)((  2e-3 * sysTICSperSEC) + .5)
#define FIVE_MS         (uint32_t)((  5e-3 * sysTICSperSEC) + .5)
#define TEN_MS          (uint32_t)(( 10e-3 * sysTICSperSEC) + .5)
#define TWENTY_MS       (uint32_t)(( 20e-3 * sysTICSperSEC) + .5)
#define THIRTY_MS       (uint32_t)(( 30e-3 * sysTICSperSEC) + .5)
#define FIFTY_MS        (uint32_t)(( 50e-3 * sysTICSperSEC) + .5)
#define HUNDRED_MS      (uint32_t)((100e-3 * sysTICSperSEC) + .5)
#define ONE_FIFTY_MS    (uint32_t)((150e-3 * sysTICSperSEC) + .5)
#define QUARTER_SEC     (uint32_t)((250e-3 * sysTICSperSEC) + .5)
#define HALF_SEC        (uint32_t)((500e-3 * sysTICSperSEC) + .5)
#define ONE_SEC         (uint32_t)(( 1.0   * sysTICSperSEC) + .5)
#define TWO_SEC         (uint32_t)(( 2.0   * sysTICSperSEC) + .5)
#define FIVE_SEC        (uint32_t)(( 5.0   * sysTICSperSEC) + .5)
#define TEN_SEC         (uint32_t)((10.0   * sysTICSperSEC) + .5)


/******************************************************************************
 *
 * Function Name: initSysTime()
 *
 * Description:
 *    This function initializes the LPC's Timer 0 for use as the system
 *    timer.
 *
 * Calling Sequence: 
 *    void
 *
 * Returns:
 *    void
 *
 *****************************************************************************/
void initSysTime(void);

/******************************************************************************
 *
 * Function Name: getSysTICs()
 *
 * Description:
 *    This function returns the current system time in TICs.
 *
 * Calling Sequence: 
 *    void
 *
 * Returns:
 *    The current time in TICs
 *
 *****************************************************************************/
uint32_t getSysTICs(void);

/******************************************************************************
 *
 * Function Name: getElapsedSysTICs()
 *
 * Description:
 *    This function then returns the difference in TICs between the
 *    given starting time and the current system time.
 *
 * Calling Sequence: 
 *    The starting time.
 *
 * Returns:
 *    The time difference.
 *
 *****************************************************************************/
uint32_t getElapsedSysTICs(uint32_t startTime);

/******************************************************************************
 *
 * Function Name: pause()
 *
 * Description:
 *    This function does not return until the specified 'duration' in
 *    TICs has elapsed.
 *
 * Calling Sequence: 
 *    duration - length of time in TICs to wait before returning
 *
 * Returns:
 *    void
 *
 *****************************************************************************/
void pause(uint32_t duration);

#endif
