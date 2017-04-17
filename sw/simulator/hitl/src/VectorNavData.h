/*
 * Copyright (C) 2015 Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
 * Utah State University, http://aggieair.usu.edu/
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
/**
 * @file Sender.h
 *
 * HITL demo version - class holding data for VectorNav
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#ifndef INCLUDE_VECTORNAVDATA_H_
#define INCLUDE_VECTORNAVDATA_H_

class VectorNavData
{
public:
  uint64_t TimeStartup;
  float YawPitchRoll[3] = {0};
  float AngularRate[3] = {0};
  double Position[3] = {0};
  float Velocity[3] = {0};
  float Accel[3] = {0};
  uint64_t Tow;
  uint8_t NumSats;
  uint8_t Fix;
  float PosU[3] = {0};
  float VelU;
  float LinearAccelBody[3] = {0};
  float YprU[3] = {0};
  uint16_t InsStatus;
  float VelBody[3] = {0};
};

#endif /* INCLUDE_VECTORNAVDATA_H_ */
