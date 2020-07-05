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
 * Airframe module for parsing XML config files
 *)

module OT = Ocaml_tools

module Autopilot = struct

  type t = { name: string; freq: string option; xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("autopilot", attrs, []) as xml ->
        { name = Xml.attrib xml "name";
          freq = ExtXml.attrib_opt xml "freq";
          xml }
    | _ -> failwith "Airframe.Autopilot.from_xml: unreachable"

end

module Include = struct

  type t = { href: string; xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("include", _, []) as xml ->
      { href = Xml.attrib xml "href"; xml }
    | _ -> failwith "Airframe.Include.from_xml: unreachable"

end

module Target = struct

  type t = { name: string;
             board: string;
             modules: Module.config list;
             autopilot: Autopilot.t option;
             configures: Module.configure list;
             defines: Module.define list;
             xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("target", _, children) as xml ->
      { name = Xml.attrib xml "name";
        board = Xml.attrib xml "board";
        modules = ExtXml.parse_children "module" Module.config_from_xml children;
        autopilot = begin
          try Some (Autopilot.from_xml (ExtXml.child xml "autopilot"))
          with _ -> None end;
        configures = ExtXml.parse_children "configure" Module.parse_configure children;
        defines = ExtXml.parse_children "define" Module.parse_define children;
        xml }
    | _ -> failwith "Airframe.Autopilot.from_xml: unreachable"

end

module Firmware = struct

  type t = { name: string;
             targets: Target.t list;
             modules: Module.config list;
             autopilot: Autopilot.t option;
             configures: Module.configure list;
             defines: Module.define list;
             xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("firmware", _, children) as xml ->
      { name = Xml.attrib xml "name";
        targets = ExtXml.parse_children "target" Target.from_xml children;
        modules = ExtXml.parse_children "module" Module.config_from_xml children;
        autopilot = begin
          try Some (Autopilot.from_xml (ExtXml.child xml "autopilot"))
          with _ -> None end;
        configures = ExtXml.parse_children "configure" Module.parse_configure children;
        defines = ExtXml.parse_children "define" Module.parse_define children;
        xml }
    | _ -> failwith "Airframe.Firmware.from_xml: unreachable"

end

module OldModules = struct

  type t = { modules: Module.config list;
             xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("modules", _, children) as xml ->
      { modules = ExtXml.parse_children "module" Module.config_from_xml children;
        xml }
    | _ -> failwith "Airframe.Modules.from_xml: unreachable"

end

type t = {
  filename: string;
  name: string;
  includes: Include.t list;
  modules: OldModules.t list; (* NOTE this is a deprecated format, should be removed *)
  firmwares: Firmware.t list;
  autopilots: Autopilot.t list;
  xml: Xml.xml
}

let from_xml = function
  | Xml.Element ("airframe", _, children) as xml ->
      if List.exists (fun c -> Xml.tag c = "modules") children then
        Printf.eprintf "\nWarning: 'modules' node is deprecated, please move modules to 'firmware' section\n%!";
      { filename = ""; name = Xml.attrib xml "name";
        includes = ExtXml.parse_children "include" Include.from_xml children;
        modules = ExtXml.parse_children "modules" OldModules.from_xml children;
        firmwares = ExtXml.parse_children "firmware" Firmware.from_xml children;
        autopilots = ExtXml.parse_children "autopilot" Autopilot.from_xml children;
        xml }
  | _ -> failwith "Airframe.from_xml: unreachable"

let from_file = fun filename ->
  let af = from_xml (Xml.parse_file filename) in
  { af with filename }


(** [expand_includes ac_id xml]
 * Get expanded xml airframe if it contains 'include' nodes
 *)
let expand_includes = fun ac_id xml ->
  match xml with
  | Xml.PCData d -> Xml.PCData d
  | Xml.Element (tag, attrs, children) ->
      Xml.Element (tag, attrs,
      List.fold_left (fun x c ->
        if Xml.tag c = "include" then begin
          let filename = Str.global_replace (Str.regexp "\\$AC_ID") ac_id (ExtXml.attrib c "href") in
          let filename =
            if Filename.is_relative filename then Filename.concat Env.paparazzi_home filename
            else filename in
          let subxml = ExtXml.parse_file filename in
          x @ (Xml.children subxml)
        end
        else x @ [c]
      ) [] children)
