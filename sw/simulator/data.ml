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

(* let pprz_conf_path = Env.paparazzi_src // "conf" *)
let user_conf_path = Env.paparazzi_home // "conf"

let conf_xml = Xml.parse_file (user_conf_path // "conf.xml")


type _class = {
	class_id : int;
	class_name : string;
	class_type : string;
	class_xml : Xml.xml;
}


let messages_ap =
(*  let xml = Xml.parse_file (pprz_conf_path // "messages.xml") in *)
  let xml = Pprz.messages_xml () in
  try
  	let version_xml = Pprz.get_downlink_messages_in_one_class xml in
    ExtXml.child version_xml ~select:(fun x -> Xml.attrib x "name" = "telemetry") "class"
  with
    Not_found -> (*failwith "'telemetry' class missing in messages.xml"*) failwith "Data.messages_ap error loading messages.xml"
		| Xml.Error er -> failwith (sprintf "Data.ml XML Error (Error: %s)" (Xml.error er))
		| e -> failwith (sprintf "Data.ml Unhandled exception (Exception: %s)" (Printexc.to_string e))

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
    airframe = Xml.parse_file airframe_file;
    flight_plan = Xml.parse_file (user_conf_path // ExtXml.attrib aircraft_xml "flight_plan");
    radio = Xml.parse_file (user_conf_path // ExtXml.attrib aircraft_xml "radio")
  }

module type MISSION = sig val ac : aircraft end
