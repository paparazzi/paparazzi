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
 * Settings module for parsing XML config files
 *
 * FIXME order is not preserved if dl_settings and dl_setting are in separated lists
 *)

module Dl_setting = struct

  type t = {
    var: string;
    shortname: string option;
    handler: string option;
    header: string option;
    xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("dl_setting", attribs, _) as xml ->
      { var = Xml.attrib xml "var";
        shortname = ExtXml.attrib_opt xml "shortname";
        handler = ExtXml.attrib_opt xml "handler";
        header = begin
          match ExtXml.attrib_opt xml "module",
                ExtXml.attrib_opt xml "header" with
          | Some m, _ ->
            (*Printf.eprintf "Warning: please rename 'module' attribute in settings with 'header'\n"; FIXME *)
            Some m
          | None, Some h -> Some h
          | None, None -> None
        end;
        xml;
      }
    | _ -> failwith "Settings.Dl_setting.from_xml: unreachable"

end

module Dl_settings = struct

  type t = {
    name: string option;
    dl_settings: t list;
    dl_setting: Dl_setting.t list;
    headers: string list;
    xml: Xml.xml; }

  let rec iter_xml s = function
    | Xml.Element ("dl_settings", attribs, children) as xml ->
      { name = ExtXml.attrib_opt xml "name";
        dl_settings = List.map (iter_xml s) children;
        dl_setting = [];
        headers = [];
        xml }
    | Xml.Element ("dl_setting", attribs, _) as xml ->
      { s with dl_setting = Dl_setting.from_xml xml :: s.dl_setting }
    | Xml.Element ("include", [("header", name)], _) ->
      { s with headers = name :: s.headers }
    | _ -> failwith "Settings.Dl_settings.iter_xml: unreachable"

  let from_xml = iter_xml { name = None; dl_settings = []; dl_setting = []; headers = []; xml = Xml.Element ("dl_settings", [], []) }

end

type t = {
  filename: string;
  name: string option; (* for modules' settings *)
  target: string option;
  dl_settings: Dl_settings.t list;
  xml: Xml.xml
}

let from_xml = function
  | Xml.Element ("settings", attribs, children) as xml ->
    { filename = "";
      name = ExtXml.attrib_opt xml "name";
      target = ExtXml.attrib_opt xml "target";
      dl_settings = List.map Dl_settings.from_xml children;
      xml }
  | _ -> failwith "Settings.from_xml: unreachable"

let from_file = fun filename ->
  let s = from_xml (Xml.parse_file filename) in
  { s with filename }

(**
 *  Get singletonized list of headers from a Settings.t
 *)
let get_headers = fun settings ->
  let rec iter = fun headers s ->
    let headers = List.fold_left (fun hl dl_s ->
        match dl_s.Dl_setting.header with
        | Some x -> if List.mem x hl then hl else x :: hl
        | None -> hl
      ) headers s.Dl_settings.dl_setting in
    let headers = List.fold_left (fun hl h ->
        if List.mem h hl then hl else h :: hl
      ) headers s.Dl_settings.headers in
    let headers = List.fold_left (fun hl dl_ss ->
        iter hl dl_ss
      ) headers s.Dl_settings.dl_settings in
    headers
  in
  List.fold_left iter [] settings.dl_settings


(* Get settings as a single XML node *)
let get_settings_xml = fun settings ->
  let settings_xml = List.fold_left (fun l s ->
    if List.length s.dl_settings > 0
    then (Xml.children (ExtXml.child s.xml "dl_settings")) @ l
    else l
  ) [] settings
  in
  let settings_xml = List.rev settings_xml in (* list in correct order *)
  let dl_settings = Xml.Element("dl_settings", [], settings_xml) in
  Xml.Element ("settings", [], [dl_settings])

