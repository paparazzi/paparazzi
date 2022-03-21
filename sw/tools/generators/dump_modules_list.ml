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

  let ac_name, dump_out =
    if Array.length Sys.argv = 2 then
      Sys.argv.(1), None
    else if Array.length Sys.argv = 3 then
      Sys.argv.(1), Some Sys.argv.(2)
    else
      failwith "Dump modules: provide aircraft name and output file (or nothing for stdout)"
  in

  let conf_xml = ExtXml.parse_file conf_xml_file in
  let aircraft_xml = try
    List.find (fun a -> ExtXml.attrib a "name" = ac_name) (Xml.children conf_xml)
  with Not_found -> failwith ("Dump modules: aircraft name not found in conf.xml") in

  let ac = Aircraft.parse_aircraft ~parse_af:true ~parse_ap:true ~parse_fp:true "" aircraft_xml in
  let modules_filenames = List.map (fun m -> m.Module.xml_filename) ac.Aircraft.all_modules in
  let modules_filenames = String.concat "\n" modules_filenames in

  match dump_out with
  | None -> Printf.printf "%s\n" modules_filenames
  | Some f ->
      let out = open_out f in
      Printf.fprintf out "%s\n" modules_filenames;
      close_out out

