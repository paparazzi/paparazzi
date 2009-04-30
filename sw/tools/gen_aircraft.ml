(*
 * $Id$
 *
 * Call to Makefile.ac with the appropriate attributes from conf.xml
 *  
 * Copyright (C) 2003-2009 Pascal Brisset, Antoine Drouin, ENAC
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

let paparazzi_conf = Env.paparazzi_home // "conf" 
let conf_xml = paparazzi_conf // "conf.xml"

let mkdir = fun d ->
  if not (Sys.file_exists d) then
    Unix.mkdir d 0o755

let check_unique_id = fun conf ->
  let ids = Hashtbl.create 5
  and names = Hashtbl.create 5 in
  List.iter
    (fun x -> 
      if String.lowercase (Xml.tag x) = "aircraft" then 
	let id = ExtXml.attrib x "ac_id"
	and name = ExtXml.attrib x "name" in
	if Hashtbl.mem ids id then begin
	  let other_name = Hashtbl.find ids id in
	  failwith (sprintf "Error: A/C Id '%s' duplicated in %s (%s and %s)" id conf_xml name other_name)
	end;
	if Hashtbl.mem names name then begin
	  let other_id = Hashtbl.find names name in
	  failwith (sprintf "Error: A/C name '%s' duplicated in %s (ids %s and %s)" name conf_xml id other_id)
	end;
	Hashtbl.add ids id name;
	Hashtbl.add names name id)
    (Xml.children conf)



let () =
  if Array.length Sys.argv <> 2 then
    failwith (sprintf "Usage: %s <A/C ident (conf.xml)>" Sys.argv.(0));
  let aircraft = Sys.argv.(1) in
  let conf = Xml.parse_file conf_xml in
  check_unique_id conf;
  let aircraft_xml =
    try
      ExtXml.child conf ~select:(fun x -> Xml.attrib x "name" = aircraft) "aircraft"
	with
      Not_found -> failwith (sprintf "Aircraft '%s' not found in '%s'" aircraft conf_xml)

 in
  let value = ExtXml.attrib aircraft_xml in

  let aircraft_dir = Env.paparazzi_home // "var" // aircraft in
  let aircraft_conf_dir = aircraft_dir // "conf" in

  mkdir (Env.paparazzi_home // "var");
  mkdir aircraft_dir;
  mkdir (aircraft_dir // "fbw");
  mkdir (aircraft_dir // "autopilot");
  mkdir (aircraft_dir // "sim");
  mkdir aircraft_conf_dir;
  mkdir (aircraft_conf_dir // "airframes");
  mkdir (aircraft_conf_dir // "flight_plans");
  mkdir (aircraft_conf_dir // "radios");
  mkdir (aircraft_conf_dir // "settings");
  mkdir (aircraft_conf_dir // "telemetry");

  let settings = 
    try value "settings" with 
      _ -> 
	fprintf stderr "\nWARNING: No 'settings' attribute specified for A/C '%s', using 'settings/basic.xml'\n\n%!" aircraft;
	"settings/basic.xml" in

  let conf_aircraft = Env.expand_ac_xml aircraft_xml in
  let conf_aircraft_file = aircraft_conf_dir // "conf_aircraft.xml" in
  let f = open_out conf_aircraft_file in
  Printf.fprintf f "%s\n" (ExtXml.to_string_fmt conf_aircraft);
  close_out f;

  let md5sum = Digest.to_hex (Digest.file conf_aircraft_file) in
  let md5sum_file = aircraft_conf_dir // "aircraft.md5" in
  let f = open_out md5sum_file in
  Printf.fprintf f "%s\n" md5sum;
  close_out f;
  
  let make = fun target options ->
    let c = sprintf "make -f Makefile.ac AIRCRAFT=%s AC_ID=%s AIRFRAME_XML=%s TELEMETRY=%s SETTINGS=\"%s\" MD5SUM=\"%s\" %s %s" aircraft (value "ac_id") (value "airframe") (value "telemetry") settings md5sum options target in
    prerr_endline c;
    begin (** Quiet is speficied in the Makefile *)
      try if Sys.getenv "Q" <> "@" then raise Not_found with
	Not_found -> prerr_endline c
    end;
    let returned_code = Sys.command c in
    if returned_code <> 0 then
      exit returned_code in

  let make_opt = fun target var attr ->
    try
      let value = Xml.attrib aircraft_xml attr in
      make target (sprintf "%s=%s" var value)
    with
      Xml.No_attribute _ -> () in

  make "makefile_ac" "";
  make "all_ac_h" "";
  make_opt "radio_ac_h" "RADIO" "radio";
  make_opt "flight_plan_ac_h" "FLIGHT_PLAN" "flight_plan"
