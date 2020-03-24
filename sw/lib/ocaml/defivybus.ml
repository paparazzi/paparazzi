(*
 * Default ivy bus address selection
 *
 * Copyright (C) 2011 Eric Parsonage
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


let default_ivy_bus = (
  try (Sys.getenv "IVY_BUS" )
  with  Not_found ->
    (if Os_calls.contains (Os_calls.os_name) "Darwin" then
        "224.255.255.255:2010"
     else
        "127.255.255.255:2010"))
