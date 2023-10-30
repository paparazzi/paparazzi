/*
 * Copyright (C) 2015 Tomaso De Ponti <t.m.l.deponti@tudelft.nl>
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
 * @file firmwares/rotorcraft/guidance/guidance_oneloop.c
 *
 * A dummy guidance module to run the oneloop_andi controller
 *
 */

#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/guidance/guidance_oneloop.h"
#include "firmwares/rotorcraft/oneloop/oneloop_andi.h"

void guidance_h_run_enter(void)
{
  oneloop_andi_enter(false);
}

void guidance_v_run_enter(void)
{
  // nothing to do
}

static struct VerticalGuidance *_gv       = &guidance_v;
static enum GuidanceOneloop_VMode _v_mode = GUIDANCE_ONELOOP_V_POS;

struct StabilizationSetpoint guidance_h_run_pos(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_oneloop_run_mode(in_flight, gh, _gv, GUIDANCE_ONELOOP_H_POS, _v_mode);
}

struct StabilizationSetpoint guidance_h_run_speed(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_oneloop_run_mode(in_flight, gh, _gv, GUIDANCE_ONELOOP_H_SPEED, _v_mode);
}

struct StabilizationSetpoint guidance_h_run_accel(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_oneloop_run_mode(in_flight, gh, _gv, GUIDANCE_ONELOOP_H_ACCEL, _v_mode);
}

int32_t guidance_v_run_pos(bool in_flight UNUSED, struct VerticalGuidance *gv)
{
  _gv = gv;
  _v_mode = GUIDANCE_ONELOOP_V_POS;
  return 0; // nothing to do
}

int32_t guidance_v_run_speed(bool in_flight UNUSED, struct VerticalGuidance *gv)
{
  _gv = gv;
  _v_mode = GUIDANCE_ONELOOP_V_SPEED;
  return 0; // nothing to do
}

int32_t guidance_v_run_accel(bool in_flight UNUSED, struct VerticalGuidance *gv)
{
  _gv = gv;
  _v_mode = GUIDANCE_ONELOOP_V_ACCEL;  
  return 0; // nothing to do
}

struct StabilizationSetpoint guidance_oneloop_run_mode(bool in_flight, struct HorizontalGuidance *gh, struct VerticalGuidance *gv, enum GuidanceOneloop_HMode h_mode, enum GuidanceOneloop_VMode v_mode)
{
  struct FloatVect3 PSA_des    = { 0 };
  int    rm_order_h = 3;
  int    rm_order_v = 3;
  // Oneloop controller wants desired targets and handles reference generation internally
  if (h_mode == GUIDANCE_ONELOOP_H_POS) {
    PSA_des.x   = POS_FLOAT_OF_BFP(gh->sp.pos.x);
    PSA_des.y   = POS_FLOAT_OF_BFP(gh->sp.pos.y);
    rm_order_h  = 3;
  }
  else if (h_mode == GUIDANCE_ONELOOP_H_SPEED) {
    PSA_des.x   = SPEED_FLOAT_OF_BFP(gh->sp.speed.x);
    PSA_des.y   = SPEED_FLOAT_OF_BFP(gh->sp.speed.y);
    PSA_des.z   = SPEED_FLOAT_OF_BFP(gv->zd_sp);
    rm_order_h  = 2;
  }
  else { // H_ACCEL
    PSA_des.x   = ACCEL_FLOAT_OF_BFP(gh->ref.accel.x);
    PSA_des.y   = ACCEL_FLOAT_OF_BFP(gh->ref.accel.y);
    PSA_des.z   = ACCEL_FLOAT_OF_BFP(gv->zdd_ref);
    rm_order_h  = 1;
  }

  if (v_mode == GUIDANCE_ONELOOP_V_POS){
    PSA_des.z   = POS_FLOAT_OF_BFP(gv->z_sp);
    rm_order_v  = 3;
  }
  else if (v_mode == GUIDANCE_ONELOOP_V_SPEED) {
    PSA_des.z   = SPEED_FLOAT_OF_BFP(gv->zd_sp);
    rm_order_v  = 2;
  }
  else { // H_ACCEL
    PSA_des.z   = ACCEL_FLOAT_OF_BFP(gv->zdd_ref); //why is there not acceleration SP and only REF?
    rm_order_v  = 1;
  }
  half_loop = false;
  oneloop_andi_run(in_flight, half_loop, PSA_des, rm_order_h, rm_order_v);
  struct StabilizationSetpoint sp = { 0 };
  return sp; 
}