(*
 * $Id$
 *
 * Configuration handling
 *
 * Copyright (C) 2004 CENA/ENAC, Yann Le Fablec, Pascal Brisset, Antoine Drouin
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

open Printf

let (//) = Filename.concat
let make_element = fun t a c -> Xml.Element (t,a,c)

let paparazzi_src =
  try
    Sys.getenv "PAPARAZZI_SRC"
  with
    _ -> "/usr/share/paparazzi"

let paparazzi_home =
  try
    Sys.getenv "PAPARAZZI_HOME"
  with
    _ -> Filename.concat (Sys.getenv "HOME") "paparazzi"


let flight_plans_path = paparazzi_home // "conf" // "flight_plans"

let flight_plan_dtd = flight_plans_path // "flight_plan.dtd"

let icon_file = paparazzi_home // "data/pictures/penguin_icon.png"

let gconf_file = paparazzi_home // "conf" // "%gconf.xml"

let gcs_icons_path = paparazzi_home // "data" // "pictures" // "gcs_icons"

let expand_ac_xml = fun ac_conf ->
  let prefix = fun s -> sprintf "%s/conf/%s" paparazzi_home s in
  let parse = fun a ->
    let file = prefix (ExtXml.attrib ac_conf a) in
    try
      Xml.parse_file file
    with
      Xml.File_not_found _ ->
	prerr_endline (sprintf "File not found: %s" file);
	make_element "file_not_found" ["file",a] []
    | Xml.Error e ->
	let s = Xml.error e in
	prerr_endline (sprintf "Parse error in %s: %s" file s);
	make_element "cannot_parse" ["file",file;"error", s] [] in
  let fp = parse "flight_plan"
  and af = parse "airframe"
  and rc = parse "radio"
  and st = parse "settings"
  and tm = parse "telemetry" in
  let children = Xml.children ac_conf@[fp; af; rc; st; tm] in
  make_element (Xml.tag ac_conf) (Xml.attribs ac_conf) children
