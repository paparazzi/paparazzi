(*
 * Commons for papgets
 *
 * Copyright (C) 2008 ENAC
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

let get_property = fun attr_name xml ->
  let attr = ExtXml.child ~select:(fun x -> ExtXml.attrib x "name" = attr_name) xml "property" in
  ExtXml.attrib attr "value"


let get_prop = fun name children default ->
  let xml = Xml.Element ("", [], children) in
  try get_property name xml with _ -> default

let property = fun name value ->
  Xml.Element("property", [ "name", name; "value", value ], [])

let xml = fun type_ display_ properties ->
  Xml.Element ("papget", ["type", type_; "display", display_],
               List.map (fun (x, y) -> property x y) properties)

let float_property = fun name value ->
  property name (string_of_float value)

let dnd_source = fun (widget:GObj.widget) papget_xml ->
  let dnd_targets = [ { Gtk.target = "STRING"; flags = []; info = 0} ] in
  widget#drag#source_set dnd_targets ~modi:[`BUTTON1] ~actions:[`COPY];
  let data_get = fun _ (sel:GObj.selection_context) ~info ~time ->
    sel#return (Xml.to_string papget_xml) in
  ignore (widget#drag#connect#data_get ~callback:data_get);
