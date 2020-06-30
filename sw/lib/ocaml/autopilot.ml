(*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *                    Cyril Allignol <cyril.allignol@enac.fr>
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

(**
 * Autopilot module for parsing XML config files
 *)

module OT = Ocaml_tools

type t = {
  filename: string;
  modules: Module.config list;
  xml: Xml.xml;
}

let from_xml = function
  | Xml.Element ("autopilot", _, children) as xml ->
      let modules = List.fold_left (fun m el ->
        if Xml.tag el = "modules" then
          m @ List.map Module.config_from_xml (Xml.children el)
        else
          m
      ) [] children in
      { filename = ""; modules; xml }
  | _ -> failwith "Autopilot.from_xml: unreachable"

let from_file = fun filename ->
  let ap = from_xml (Xml.parse_file filename) in
  { ap with filename }


(* return a Settings object from flight plan *)
let get_sys_ap_settings = fun autopilots ->
  match autopilots with
  | None -> None
  | Some autopilots ->
      let dl_settings = List.fold_left (fun sl (_, autopilot) ->
        (* Filter state machines that need to be displayed *)
        let sm_filtered = List.filter (fun sm ->
          try (Compat.lowercase_ascii (Xml.attrib sm "settings_mode")) = "true" with _ -> false
          ) (Xml.children autopilot.xml) in
        if List.length sm_filtered = 0 then sl
        else
          (* Create node if there is at least one to display *)
          let dl_set = List.fold_left (fun l sm ->
            let modes = List.filter (fun m -> (Xml.tag m) = "mode") (Xml.children sm) in
            let name = Xml.attrib sm "name" in
            (* Iter on modes and store min, max and values *)
            let (_, min, max, values) = List.fold_left (fun (current, min, max, values) m ->
              let print = try Compat.lowercase_ascii (Xml.attrib m "settings") <> "hide" with _ -> true in
              let name = Xml.attrib m "name" in
              if print then begin
                let min = match min with
                | None -> Some current
                | Some x -> Some x
                in
                let max = Some current in
                let values = values @ [name] in
                (current + 1, min, max, values)
              end
              else begin
                let n = match min with None -> [] | _ -> [name] in
                (current + 1, min, max, values @ n)
              end
            ) (0, None, None, []) modes in
            (* check handler *)
            let handler = try
              let sh = Xml.attrib sm "settings_handler" in
              let s = Str.split (Str.regexp "|") sh in
              match s with
              | [header; handler] -> [("header",header); ("handler",handler)]
              | _ -> failwith "Gen_autopilot: invalid handler format"
            with _ -> [("header","autopilot_core_"^name)] in
            begin match min, max with
              | Some min_idx, Some max_idx ->
                  Xml.Element ("dl_setting", [
                        ("min", string_of_int min_idx);
                        ("max", string_of_int max_idx);
                        ("step", "1");
                        ("var", "autopilot_mode_"^name);
                        ("shortname", name);
                        ("values", (String.concat "|" values))]
                        @ handler, []) :: l
              | _, _ -> l
            end
          ) [] sm_filtered in
          dl_set @ sl
      ) [] autopilots in
      let xml = Xml.Element ("dl_settings", [("name","Autopilot")], dl_settings) in
      Some (Settings.from_xml (Xml.Element("settings",[],[xml])))

