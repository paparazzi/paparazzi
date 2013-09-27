(*
 * Support for obtaining google maps api information at runtime
 *
 * Copyright (C) 2011 Stephen Dwyer
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
let home = Env.paparazzi_home
let (//) = Filename.concat
let maps_xml_path = home // "conf" // "maps.xml"
let maps_xml_default_path = home // "conf" // "maps_example.xml"

let maps_xml = ExtXml.parse_file maps_xml_path
let maps_xml_default = ExtXml.parse_file (maps_xml_default_path)
let gv = try Some (ExtXml.int_attrib maps_xml "google_version") with _ -> None
let gv_default = try ExtXml.int_attrib maps_xml_default "google_version" with _ -> 0

let google_version = match gv with Some v -> v | None -> gv_default

