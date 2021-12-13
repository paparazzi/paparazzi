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
 * Periodic telemetry module for parsing XML config files
 *)

type msg_period = MsgPeriod of float | MsgFreq of float

module Message = struct

  type t = {
    name: string;
    period: msg_period;
    phase: float option;
    xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("message", _, []) as xml ->
      { name = Xml.attrib xml "name";
        period = begin
          match ExtXml.attrib_opt_float xml "period",
                ExtXml.attrib_opt_float xml "freq" with
          | Some t, None -> MsgPeriod t
          | None, Some f -> MsgFreq f
          | Some _, Some _ -> failwith "Telemetry.Message.from_xml: either specify 'period' or 'freq' attribute, not both"
          | None, None -> failwith "Telemetry.Message.from_xml: specify 'period' or 'freq' attribute"
        end;
        phase = ExtXml.attrib_opt_float xml "phase";
        xml
      }
    | _ -> failwith "Telemetry.Message.from_xml: unreachable"

end

module Mode = struct

  type t = {
    name: string;
    key_press: string option;
    messages: Message.t list;
    xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("mode", _, messages) as xml ->
      { name = Xml.attrib xml "name";
        key_press = ExtXml.attrib_opt xml "key_press";
        messages = List.map Message.from_xml messages;
        xml }
    | _ -> failwith "Telemetry.Mode.from_xml: unreachable"

end

module Process = struct

  type t = {
    name: string;
    proc_type: string option;
    modes: Mode.t list;
    xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("process", attribs, modes) as xml ->
      { name = Xml.attrib xml "name";
        proc_type = ExtXml.attrib_opt xml "type";
        modes = List.map Mode.from_xml modes;
        xml }
    | _ -> failwith "Telemetry.Process.from_xml: unreachable"

end

type t = {
  filename: string;
  processes: Process.t list;
  xml: Xml.xml
}

let from_xml = function
  | Xml.Element ("telemetry", [], processes) as xml ->
    { filename = "";
      processes = List.map Process.from_xml processes;
      xml
    }
  | _ -> failwith "Telemetry.from_xml: unreachable"

let from_file = fun filename ->
  let t = from_xml (Xml.parse_file filename) in
  { t with filename }


(* return a Settings object from telemetry process and modes *)
let get_sys_telemetry_settings = fun telemetry ->
  match telemetry with
  | None -> None
  | Some telemetry ->
    (* build a XML node corresponding to the settings *)
    let tl_settings = List.fold_left (fun lp p ->
      if List.length p.Process.modes > 1 then begin
        (* only if more than one mode *)
        let p_name = p.Process.name in
        let modes = List.map (fun m -> m.Mode.name) p.Process.modes in
        let nb_modes = List.length modes in
        match nb_modes with
          | 0 | 1 -> lp (* Nothing to do if 1 or zero mode *)
          | _ -> (* add settings with all modes *)
              let (key, _) = List.fold_left (fun (lk, i) m ->
                match m.Mode.key_press with
                | None -> (lk, i+1)
                | Some k -> ((Xml.Element ("key_press", [("key", k); ("value", string_of_int i)], [])) :: lk, i+1)
              ) ([], 0) p.Process.modes in
              lp @ [Xml.Element ("dl_setting",
                    [("min", "0");
                    ("step", "1");
                    ("max", string_of_int (nb_modes-1));
                    ("var", "telemetry_mode_"^p_name);
                    ("shortname", p_name);
                    ("values", String.concat "|" modes);
                    ("header", "generated/periodic_telemetry")], key)]
      end
      else lp
    ) [] telemetry.processes in
    if List.length tl_settings > 0 then
      let xml = Xml.Element("dl_settings",[("name","Telemetry")],tl_settings) in
      Some (Settings.from_xml (Xml.Element("settings",[],[xml])))
    else
      None

