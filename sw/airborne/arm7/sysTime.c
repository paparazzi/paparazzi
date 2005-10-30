/******************************************************************************
 *
 * $RCSfile$
 * $Revision$
 *
 * This module provides the interface routines for initializing and
 * accessing the system timing functions.
 * Copyright 2004, R O SoftWare
 * No guarantees, warrantees, or promises, implied or otherwise.
 * May be used for hobby or commercial purposes provided copyright
 * notice remains intact.
 *
 *****************************************************************************/
#include "types.h"
#include "LPC21xx.h"
#include "config.h"
#include "sysTime.h"

static uint32_t sysTICs;
static uint32_t lastT0TC;

/******************************************************************************
 *
 * Function Name: initSysTime()
 *
 * Description:
 *    This function initializes the LPC's Timer 0 for use as the system timer.
 *
 * Calling Sequence: 
 *    void
 *
 * Returns:
 *    void
 *
 *****************************************************************************/
void initSysTime(void)
{
  // setup Timer 1 to count forever
  T0TCR = TCR_RESET;                    // reset & disable timer 0
  T0PR = T0_PCLK_DIV - 1;               // set the prescale divider
  T0MCR = 0;                            // disable match registers
  T0CCR = 0;                            // disable compare registers
  T0EMR = 0;                            // disable external match register
  T0TCR = TCR_ENABLE;                   // enable timer 0
  sysTICs = 0;
}

/******************************************************************************
 *
 * Function Name: getSysTICs()
 *
 * Description:
 *    This function returns the current syetem time in TICs.
 *
 * Calling Sequence: 
 *    void
 *
 * Returns:
 *    The current time in TICs as represented by sysTICs
 *
 *****************************************************************************/
uint32_t getSysTICs(void)
{
  uint32_t now = T0TC;

  sysTICs += (uint32_t)(now - lastT0TC);
  lastT0TC = now;
  return sysTICs;
}


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
uint32_t getElapsedSysTICs(uint32_t startTime)
{
  return getSysTICs() - startTime;
}


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
void pause(uint32_t duration)
{
  uint32_t startTime = getSysTICs();

  while (getElapsedSysTICs(startTime) < duration)
    WDOG();
}
