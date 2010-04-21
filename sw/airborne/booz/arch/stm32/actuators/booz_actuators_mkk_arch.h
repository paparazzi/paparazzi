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

#ifndef BOOZ_ACTUATORS_MKK_ARCH_H
#define BOOZ_ACTUATORS_MKK_ARCH_H

#include <stm32/tim.h>
#include <stm32/gpio.h>




#define BoozActuatorsMkkArchSend() {		\
    /*DEBUG5_T();*/				\
    TIM_SetCounter(TIM2, 0);			\
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update); \
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);	\
  }

#endif /* BOOZ_ACTUATORS_MKK_ARCH_H */
