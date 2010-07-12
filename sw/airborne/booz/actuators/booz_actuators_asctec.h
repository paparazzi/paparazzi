/*
 * $Id$
 *  
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#ifndef BOOZ_ACTUATORS_ASCTEC_H
#define BOOZ_ACTUATORS_ASCTEC_H

enum actuators_astec_cmd { NONE,
			   TEST,
			   REVERSE,
			   SET_ADDR };

enum actuators_astec_addr { FRONT,
			    BACK,
			    LEFT,
			    RIGHT };

/* this is for the v1 protocol which does its own mixing */
enum actuators_astec_cmds { PITCH,
			    ROLL,
			    YAW,
			    THRUST,
                            CMD_NB };

struct ActuatorsAsctec {
  enum actuators_astec_cmd cmd;
  enum actuators_astec_addr cur_addr;
  enum actuators_astec_addr new_addr;
  int32_t cmds[CMD_NB];
  volatile bool_t  i2c_done;
  volatile uint32_t nb_err;
};


extern struct ActuatorsAsctec actuators_asctec; 

#define booz_actuators_asctec_SetCommand(_v) {  \
    actuators_asctec.cmd = _v;			\
}

#define booz_actuators_asctec_SetAddr(_v) {	\
    actuators_asctec.new_addr = _v;		\
}


#endif /* BOOZ_ACTUATORS_ASCTEC_H */
