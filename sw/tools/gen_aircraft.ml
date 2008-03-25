(*
 * $Id$
 *
 * Call to Makefile.ac with the appropriate attributes from conf.xml
 *  
 * Copyright (C) 2003-2008 Pascal Brisset, Antoine Drouin, ENAC
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
  let ids = Hashtbl.create 5 in
  List.iter
    (fun x -> 
      if String.lowercase (Xml.tag x) = "aircraft" then 
	let id = ExtXml.attrib x "ac_id" in
	if Hashtbl.mem ids id then
	  failwith (sprintf "Error: A/C Id '%s' duplicated in %s" id conf_xml);
	Hashtbl.add ids id ())
    (Xml.children conf)

let _ =
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
  
  let c = sprintf "make -f Makefile.ac AIRCRAFT=%s AC_ID=%s AIRFRAME_XML=%s RADIO=%s FLIGHT_PLAN=%s TELEMETRY=%s SETTINGS=\"%s\" MD5SUM=\"%s\" all_ac_h" aircraft (value "ac_id") (value "airframe") (value "radio") (value "flight_plan") (value "telemetry") settings md5sum in
  begin (** Quiet is speficied in the Makefile *)
    try if Sys.getenv "Q" <> "@" then raise Not_found with
      Not_found -> prerr_endline c
  end;
  exit (Sys.command c)
