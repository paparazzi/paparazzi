(*
 * Dump a modules' list
 *
 * Copyright (C) 2022 Gautier Hattenberger
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *)

let (//) = Filename.concat
let conf_dir = Env.paparazzi_home // "conf"
let conf_xml_file = conf_dir // "conf.xml"

let () =
  let ac_name = ref None
  and af_xml = ref None
  and fp_xml = ref None
  and output = ref None in

  let options = [
    "-ac", Arg.String (fun x -> ac_name := Some x), "Aircraft name (mandatory)";
    "-af", Arg.String (fun x -> af_xml := Some x), "Airframe XML file (optinal)";
    "-fp", Arg.String (fun x -> fp_xml := Some x), "Flight_plan XML file (optional)";
    "-o", Arg.String (fun x -> output := Some x), "Output file name (stdout if not specified)"
  ] in
  Arg.parse options (fun _ -> ()) "Usage:";

  let ac_name, xml_files = match !ac_name, !af_xml, !fp_xml with
  | None, _, _ -> failwith "Dump modules: provide aircraft name"
  | Some n, None, None -> n, None
  | Some n, Some af, Some fp -> n, Some (af, fp)
  | Some _, _, _ -> failwith "Dump modules: provide both airframe and flight plan or nothing"
  in

  let aircraft_xml = match xml_files with
  | None ->
      let conf_xml = ExtXml.parse_file conf_xml_file in
      begin try
        List.find (fun a -> ExtXml.attrib a "name" = ac_name) (Xml.children conf_xml)
      with Not_found -> failwith ("Dump modules: aircraft name not found in conf.xml") end
  | Some (airframe_xml, flight_plan_xml) ->
      Xml.Element("aircraft", ["name", ac_name; "airframe", airframe_xml; "flight_plan", flight_plan_xml], [])
  in

  let ac = Aircraft.parse_aircraft ~parse_af:true ~parse_ap:true ~parse_fp:true "" aircraft_xml in
  let modules_filenames = List.map (fun m -> m.Module.xml_filename) ac.Aircraft.all_modules in
  let modules_filenames = String.concat "\n" modules_filenames in

  match !output with
  | None -> Printf.printf "%s\n" modules_filenames
  | Some f ->
      let out = open_out f in
      Printf.fprintf out "%s\n" modules_filenames;
      close_out out

