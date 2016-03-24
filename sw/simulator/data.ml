(*
 * Usefull data for simulation
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

open Printf

let (//) = Filename.concat

let user_conf_path = Env.paparazzi_home // "conf"
let user_var_path = Env.paparazzi_home // "var"

let conf_xml = ExtXml.parse_file (user_conf_path // "conf.xml")

let messages_ap =
  let xml = ExtXml.parse_file (user_var_path // "messages.xml") in
  try
    ExtXml.child xml ~select:(fun x -> Xml.attrib x "name" = "telemetry") "msg_class"
  with
    Not_found -> failwith "'telemetry' msg_class missing in messages.xml"

type aircraft = {
    name : string;
    id : int;
    airframe : Xml.xml;
    flight_plan : Xml.xml;
    radio: Xml.xml
  }


let aircraft = fun name ->
  let aircraft_xml, id =
    let rec loop = function
	[] -> failwith ("Aircraft not found : "^name)
      | x::_ when Xml.tag x = "aircraft" && Xml.attrib x "name" = name ->
	  begin
	    try
	      (x, int_of_string (Xml.attrib x "ac_id"))
	    with
	      _ ->
		failwith (sprintf "Int value expected for 'ac_id' in %s" (Xml.to_string x))
	  end
      | _x::xs -> loop xs in
    loop (Xml.children conf_xml) in

  let airframe_file = user_conf_path // ExtXml.attrib aircraft_xml "airframe" in

  { id = id; name = name;
    airframe = ExtXml.parse_file airframe_file;
    flight_plan = ExtXml.parse_file (user_conf_path // ExtXml.attrib aircraft_xml "flight_plan");
    radio = ExtXml.parse_file (user_conf_path // ExtXml.attrib aircraft_xml "radio")
  }

module type MISSION = sig val ac : aircraft end
