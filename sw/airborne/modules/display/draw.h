/*
 * Copyright (C) 2026  Maël FEURGARD
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

/** \file draw.h
 *  \brief Helpers for the DRAW message
 *
 */

#ifndef DRAW_H
#define DRAW_H

enum Draw_opacity
{
  DRAW_OPACITY_TRANSPARENT = 0,
  DRAW_OPACITY_LIGHT       = 1,
  DRAW_OPACITY_MEDIUM      = 2,
  DRAW_OPACITY_OPAQUE      = 3
};

enum Draw_colorcode
{
  DRAW_BLACK   = 0,
  DRAW_RED     = 1,
  DRAW_GREEN   = 2,
  DRAW_YELLOW  = 3,
  DRAW_BLUE    = 4,
  DRAW_MAGENTA = 5,
  DRAW_CYAN    = 6,
  DRAW_WHITE   = 7,
};

enum Draw_shape
{
  DRAW_CIRCLE = 0,
  DRAW_POLYGON = 1,
  DRAW_LINE = 2
};

enum Draw_status
{
  DRAW_CREATE = 0,
  DRAW_DELETE = 1
};

static inline void DRAW_set_opacity(uint8_t* color, enum Draw_opacity op) {*color = ((*color) & 0b00111111) + ((op & 0b11) << 6);}
static inline void DRAW_set_line_color(uint8_t* color, enum Draw_colorcode code) {*color = ((*color) & 0b11000111) + ((code & 0b111) << 3);}
static inline void DRAW_set_fill_color(uint8_t* color, enum Draw_colorcode code) {*color = ((*color) & 0b11111000) + (code & 0b111);}
static inline uint8_t DRAW_make_color(enum Draw_opacity op, enum Draw_colorcode line, enum Draw_colorcode fill)
{
  return ((op & 0b11) << 6) + ((line & 0b111) << 3) + (fill & 0b111);
}
static inline uint8_t DRAW_make_line(enum Draw_colorcode line)
{
  return ((line & 0b111) << 3);
}

#endif // DRAW_H