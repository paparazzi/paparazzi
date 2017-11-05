/*
 * Copyright (C) Freek van Tienen
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/bebop_ae_awb.h"
 * @author Freek van Tienen, Kirk Scheper
 * Auto exposure and Auto white balancing for the Bebop 1 and 2
 */

#ifndef BEBOP_AE_AWB_H
#define BEBOP_AE_AWB_H

#define MAX_HIST_Y 256 - 20
#define MIN_HIST_Y 16

extern uint32_t histogram_plot[256];

extern float ae_awb_gain;
extern float awb_avgU;
extern float awb_avgV;
extern uint32_t awb_nb_pixels;

extern float ae_exposure_gain;
extern uint8_t ae_dark_bins;
extern uint8_t ae_bright_bins;
extern uint8_t ae_middle_index;
extern float ae_bright_ignore;
extern float ae_dark_ignore;
extern float ae_current_level;

extern void bebop_ae_awb_init(void);
extern void bebop_awb_reset(void);
extern void bebop_ae_reset(void);
extern void bebop_ae_awb_periodic(void);

#endif

