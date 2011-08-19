(*
 * $Id$
 *
 * Copyright (C) 2004 CENA/ENAC, Yann Le Fablec
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
 *)

external load_bitmap_font : string -> GlList.base
  = "_gtkgl_hack_load_bitmap_font"
external unload_bitmap_font : GlList.base -> unit
  = "_gtkgl_hack_unload_bitmap_font"
val gl_print_string : GlList.base -> string -> unit
