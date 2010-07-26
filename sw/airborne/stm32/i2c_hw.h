/*
 * $Id$
 *  
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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
 */

/*
 * Hardware level I2C handling
 */

#ifndef I2C_HW_H
#define I2C_HW_H

#include <stm32/i2c.h>

#ifdef USE_I2C1

extern void i2c1_hw_init(void);

extern void i2c1_ev_irq_handler(void);
extern void i2c1_er_irq_handler(void);

extern uint16_t i2c_errc_ack_fail;
extern uint16_t i2c_errc_miss_start_stop;
extern uint16_t i2c_errc_arb_lost;
extern uint16_t i2c_errc_over_under;
extern uint16_t i2c_errc_pec_recep;
extern uint16_t i2c_errc_timeout_tlow;
extern uint16_t i2c_errc_smbus_alert;


#define I2c1SendStart() { I2C_GenerateSTART(I2C1, ENABLE); I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);}

#ifdef I2C1_STOP_HANDLER
#include I2C1_STOP_HANDLER_HEADER
#define I2c1StopHandler() I2C1_STOP_HANDLER()
#else 
#define I2c1StopHandler() {}
#endif /* I2C1_STOP_HANDLER */

#endif /* USE_I2C1 */



#ifdef USE_I2C2

/* 
   This is a hook for a caller module to provide
   code to be called in the interrupt handler
   at the end of a transfert 
*/
#ifdef I2C2_STOP_HANDLER
#include I2C2_STOP_HANDLER_HEADER
#define I2c2StopHandler() I2C2_STOP_HANDLER()
#else 
#define I2c2StopHandler() {}
#endif /* I2C2_STOP_HANDLER */


extern void i2c2_hw_init(void);
extern void i2c2_ev_irq_handler(void);
extern void i2c2_er_irq_handler(void);

#define I2c2SendStart() { I2C_GenerateSTART(I2C2, ENABLE); I2C_ITConfig(I2C2, I2C_IT_EVT, ENABLE);}

#endif /* USE_I2C2 */


#endif /* I2C_HW_H */
