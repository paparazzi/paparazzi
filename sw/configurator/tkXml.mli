(*
 *  $Id$
 *
 * Binding of a widget to an XML file
 *  
 * Copyright (C) 2004 Pascal Brisset, Antoine Drouin
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

val create :
  'a Widget.widget ->
  (Widget.frame Widget.widget -> VarXml.xml -> unit) -> unit
(** [create parent action] Wraps the given [action] into an XML file handler.
The arguments of [action] are a graphic place and the opened XML object.
[action] may then modify the contents of the XML object and save the
modifications. Note that the interface is grabbed on the created subframe. *)

val create_section : 'a Widget.widget -> VarXml.xml -> unit
