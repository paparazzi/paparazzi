(*
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
let space_regexp = Str.regexp "[ \t]+"
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
let icon_gcs_file = paparazzi_home // "data/pictures/penguin_icon_gcs.png"
let icon_mes_file = paparazzi_home // "data/pictures/penguin_icon_msg.png"
let icon_rep_file = paparazzi_home // "data/pictures/penguin_icon_rep.png"

let gconf_file = paparazzi_home // "conf" // "%gconf.xml"

let gcs_icons_path = paparazzi_home // "data" // "pictures" // "gcs_icons"

let dump_fp = paparazzi_src // "sw" // "tools" // "generators" // "gen_flight_plan.out -dump"

let expand_ac_xml = fun ?(raise_exception = true) ac_conf ->
  let prefix = fun s -> sprintf "%s/conf/%s" paparazzi_home s in
  let parse_file = fun a file ->
    try
      ExtXml.parse_file file
    with
        Failure msg ->
          if raise_exception then
            failwith msg
          else begin
            prerr_endline msg;
            make_element "parse error" ["file",a; "msg", msg] []
          end in

  let parse = fun a ->
    List.map
      (fun filename -> parse_file a (prefix filename))
      (Str.split space_regexp (ExtXml.attrib ac_conf a)) in

  let parse_opt = fun a ->
    try parse a with ExtXml.Error _ -> [] in

  (* dump expanded version of flight plan before parsing *)
  let parse_fp = fun a ->
    try
      (* get full path file name *)
      let fp = prefix (ExtXml.attrib ac_conf a) in
      if Sys.is_directory fp then raise Not_found;
      (* create a temporary dump file *)
      let dump = Filename.temp_file "fp_dump" ".xml" in
      (* set command then call it *)
      let c = sprintf "%s %s > %s" dump_fp fp dump in
      if Sys.command c <> 0 then
        begin
          Sys.remove dump;
          failwith c
        end;
      (* parse temp fp file and then remove it *)
      let fp_xml = parse_file a dump in
      Sys.remove dump;
      (* return Xml list *)
      [fp_xml]
    with _ -> []
  in

  let pervasives = parse "airframe" @ parse "telemetry" @ parse "settings" in
  let optionals = parse_opt "radio" @ parse_fp "flight_plan" @ pervasives in

  let children = Xml.children ac_conf@optionals in
  make_element (Xml.tag ac_conf) (Xml.attribs ac_conf) children
