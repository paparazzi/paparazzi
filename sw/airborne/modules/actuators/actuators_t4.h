/*
 * Copyright (C) 2024 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/actuators/actuators_t4.h"
 * @author Alessandro Mancinelli, Sunyou Hwang, OpenUAS
 * @brief Uses a T4 Actuators Board as fly by wire system. This Board can control serial bus servos, ESC's and PWM servos, with as big benefir providing real time telemetry in return into the autopilot state.
 * Read more on how to create your own T4 Board here: https://github.com/tudelft/t4_actuators_board/
 */

#ifndef ACTUATORS_T4_H
#define ACTUATORS_T4_H

#include "modules/actuators/actuators_t4_arch.h"
#include "actuators_t4_uart.h" //#include "modules/actuators/actuators_t4_uart.h"

/** Arch dependent init file.
 * implemented in arch files
 */
extern void actuators_t4_arch_init(void);

#define ActuatorsT4Init() actuators_t4_arch_init()

#endif /* ACTUATORS_T4_H */
