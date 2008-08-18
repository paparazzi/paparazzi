/*
 * $Id$
 *  
 * Copyright (C) 2008  Antoine Drouin
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 */

#include "max1167.h"

#include "led.h"

static void EXTINT0_ISR(void) __attribute__((naked));

extern void max1167_hw_init( void ) {
  
  /* SS pin is output */
  SetBit(MAX1167_SS_IODIR, MAX1167_SS_PIN);
  /* unselected max1167 */
  Max1167Unselect();

  /* connect P0.16 to extint0 (EOC) */
  MAX1167_EOC_PINSEL |= MAX1167_EOC_PINSEL_VAL << MAX1167_EOC_PINSEL_BIT;
  /* extint0 is edge trigered */
  SetBit(EXTMODE, MAX1167_EOC_EINT);
  /* extint0 is trigered on falling edge */
  ClearBit(EXTPOLAR, MAX1167_EOC_EINT);
  /* clear pending extint0 before enabling interrupts */
  SetBit(EXTINT, MAX1167_EOC_EINT);

   /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT( VIC_EINT0 );                     // EXTINT0 selected as IRQ
  VICIntEnable = VIC_BIT( VIC_EINT0 );                       // EXTINT0 interrupt enabled
  _VIC_CNTL(MAX1167_EOC_VIC_SLOT) = VIC_ENABLE | VIC_EINT0;
  _VIC_ADDR(MAX1167_EOC_VIC_SLOT) = (uint32_t)EXTINT0_ISR;   // address of the ISR 
}


void max1167_read( void ) {
  ASSERT((max1167_status == STA_MAX1167_IDLE),		\
	 DEBUG_MAX_1117, MAX1167_ERR_READ_OVERUN);
  /* select max1167 */ 
  Max1167Select();
  /* enable SPI */
  SpiClearRti();
  SpiDisableRti();
  SpiEnable();
  /* write control byte - wait EOC on extint */
  const uint8_t control_byte = 1 << 0 | 1 << 3 | 2 << 5;
  SpiSend(control_byte);
  max1167_status = STA_MAX1167_SENDING_REQ;
}

void EXTINT0_ISR(void) {
  ISR_ENTRY();

  ASSERT((max1167_status == STA_MAX1167_SENDING_REQ),	\
	 DEBUG_MAX_1117, MAX1167_ERR_SPURIOUS_EOC);
  /* trigger 6 bytes read */
  SpiSend(0);
  SpiSend(0);
  SpiSend(0);
  SpiSend(0);
  SpiSend(0);
  SpiSend(0);
  SpiClearRti();
  SpiEnableRti();
  max1167_status = STA_MAX1167_READING_RES;
  
  SetBit(EXTINT, MAX1167_EOC_EINT);   /* clear extint0 */
  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */

  ISR_EXIT();
}
