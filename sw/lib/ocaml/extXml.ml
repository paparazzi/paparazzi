(*
 * Xml-Light extension
 *
 * Copyright (C) 2004 CENA/ENAC, Pascal Brisset
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

exception Error of string

let sep = Str.regexp "\\."

let child = fun xml ?select c ->
  let rec find = function
    | Xml.Element (tag, _attributes, _children) as elt :: elts ->
	if tag = c then
	  match select with
          | None -> elt
          | Some p -> if p elt then elt else find elts
	else find elts
    | _ :: elts -> find elts
    | [] -> raise Not_found in

  let children = Xml.children xml in

  (* Let's try with a numeric index *)
  try (Array.of_list children).(int_of_string c)
  with Failure "int_of_string" -> (* Bad luck. Go through the children *)
    find children


let get xml path =
  let p = Str.split sep path in
  let rec iter xml = function
    | [] -> failwith "ExtXml.get: empty path"
    | [x] -> begin
	try if Xml.tag xml <> x then raise Not_found else xml
	with _ -> raise Not_found end
    | x :: xs -> iter (child xml x) xs in
  iter xml p

let get_attrib = fun xml path attr -> Xml.attrib (get xml path) attr

let sprint_fields = fun () l ->
  "<"^
    List.fold_right (fun (a, b) -> (^) (Printf.sprintf "%s=\"%s\" " a b)) l ">"

let attrib = fun xml attr ->
  try Xml.attrib xml attr
  with Xml.No_attribute _ ->
    let msg = Printf.sprintf "Error: Attribute '%s' expected in <%a>"
	attr sprint_fields (Xml.attribs xml) in
    raise (Error msg)

let attrib_option = fun xml attr ->
  try Some (Xml.attrib xml attr)
  with Xml.No_attribute _ -> None

let tag_is = fun x v -> String.lowercase (Xml.tag x) = String.lowercase v

let attrib_or_default = fun x a default ->
  try Xml.attrib x a
  with Xml.No_attribute _ | Xml.Not_element _ -> default

(** Code patched from xml.ml from xml-light package: PC Data formatting removed *)
let tmp = Buffer.create 200
let buffer_attr = fun indent tab (n,v) ->
  if indent then
    Buffer.add_string tmp tab;
  Buffer.add_char tmp ' ';
  Buffer.add_string tmp n;
  Buffer.add_string tmp "=\"";
  let l = String.length v in
  for p = 0 to l-1 do
    match v.[p] with
      | '\\' -> Buffer.add_string tmp "\\\\"
      | '"' -> Buffer.add_string tmp "\\\""
    | c -> Buffer.add_char tmp c
        done;
  Buffer.add_char tmp '"';
        if indent then
          Buffer.add_char tmp '\n'

let buffer_pcdata = Buffer.add_string tmp
let my_to_string_fmt = fun tab_attribs x ->
  let rec loop ?(newl=false) tab = function
    | Xml.Element (tag,alist,[]) ->
      Buffer.add_string tmp tab;
      Buffer.add_char tmp '<';
      Buffer.add_string tmp tag;
      if tab_attribs then Buffer.add_char tmp '\n';
      List.iter (buffer_attr tab_attribs tab) alist;
      if tab_attribs then Buffer.add_string tmp tab;
      Buffer.add_string tmp "/>";
      if newl then Buffer.add_char tmp '\n';
    | Xml.Element (tag,alist,[Xml.PCData text]) ->
      Buffer.add_string tmp tab;
      Buffer.add_char tmp '<';
      Buffer.add_string tmp tag;
      List.iter (buffer_attr tab_attribs tab) alist;
      Buffer.add_string tmp ">";
      buffer_pcdata text;
      Buffer.add_string tmp "</";
      Buffer.add_string tmp tag;
      Buffer.add_char tmp '>';
      if newl then Buffer.add_char tmp '\n';
    | Xml.Element (tag,alist,l) ->
      Buffer.add_string tmp tab;
      Buffer.add_char tmp '<';
      Buffer.add_string tmp tag;
      List.iter (buffer_attr tab_attribs tab) alist;
      Buffer.add_string tmp ">\n";
      List.iter (loop ~newl:true (tab^"  ")) l;
      Buffer.add_string tmp tab;
      Buffer.add_string tmp "</";
      Buffer.add_string tmp tag;
      Buffer.add_char tmp '>';
      if newl then Buffer.add_char tmp '\n';
    | Xml.PCData text ->
      buffer_pcdata text;
      if newl then Buffer.add_char tmp '\n';
  in
  Buffer.reset tmp;
  loop "" x;
  let s = Buffer.contents tmp in
  Buffer.reset tmp;
  s



let to_string_fmt = fun ?(tab_attribs = false) xml ->
  let l = String.lowercase in
  let rec lower = function
    | Xml.PCData _ as x -> x
    | Xml.Element (t, ats, cs) ->
	Xml.Element(l t,
                    List.map (fun (a,v) -> (l a, v)) ats,
                    List.map lower cs) in
  my_to_string_fmt tab_attribs (lower xml)


let subst_attrib = fun attrib value xml ->
  let u = String.uppercase in
  let uattrib = u attrib in
  match xml with
  | Xml.Element (tag, attrs, children) ->
      let rec loop = function
          [] -> [(attrib, value)]
        | (a,_v) as c::ats ->
            if u a = uattrib then loop ats else c::loop ats in
      Xml.Element (tag,
                   loop attrs,
                   children)
  | Xml.PCData _ -> xml


let subst_child = fun ?(select= fun _ -> true) t x xml ->
  match xml with
  | Xml.Element (tag, attrs, children) ->
      let found = ref false in
      let new_children =
        List.map
          (fun xml -> if tag_is xml t && select xml then (found := true; x) else xml)
          children in
      if !found then
        Xml.Element (tag, attrs, new_children)
      else
        raise Not_found
  | Xml.PCData _ -> xml


let subst_or_add_child = fun t x xml ->
  try subst_child t x xml with Not_found ->
    match xml with
    | Xml.Element (tag, attrs, children) ->
	Xml.Element (tag, attrs, x :: children)
    |  Xml.PCData _ -> xml


let remove_child = fun ?(select= fun _ -> true) t xml ->
  match xml with
  | Xml.Element (tag, attrs, children) ->
      Xml.Element (tag,
                   attrs,
                   List.fold_right (fun xml rest -> if tag_is xml t && select xml then rest else xml::rest) children [])
  | Xml.PCData _ -> xml


let float_attrib = fun xml a ->
  let v = attrib xml a in
  try float_of_string v
  with _ -> failwith (Printf.sprintf "Error: float expected in '%s'" v)

let int_attrib = fun xml a ->
  let v = attrib xml a in
  try int_of_string v
  with _ -> failwith (Printf.sprintf "Error: integer expected in '%s'" v)

let iter_tag = fun tag f xml -> Xml.iter (fun x -> if tag_is x tag then f x) xml

let filter_tag = fun tag xml ->
  Xml.fold (fun acc x -> if tag_is x tag then x :: acc else acc) [] xml

let partition_tag = fun tag xmls ->
  List.fold_left
    (fun (yes, no) x -> if tag_is x tag then x :: yes, no else yes, x:: no
    ) ([], []) xmls

(* When an .xml is coming through http, the dtd is not available. We disable
   the DTD proving feature in this case. FIXME: We should use the resolve
   feature *)
let my_xml_parse_file =
  let parser = XmlParser.make () in
  XmlParser.prove parser false;
  fun f ->
    XmlParser.parse parser (XmlParser.SFile f)


let parse_file = fun ?(noprovedtd = false) file ->
  try (if noprovedtd then my_xml_parse_file else Xml.parse_file) file
  with
  | Xml.Error e -> failwith (sprintf "%s: %s" file (Xml.error e))
  | Xml.File_not_found f -> failwith (sprintf "File not found: %s" f)
  | Dtd.Prove_error e -> failwith (sprintf "%s: %s" file (Dtd.prove_error e))
  | Dtd.Check_error e -> failwith (sprintf "%s: %s" file (Dtd.check_error e))
  | Dtd.Parse_error e -> failwith (sprintf "%s: %s" file (Dtd.parse_error e))



let digest = fun xml -> Digest.string (Xml.to_string xml)

let predefined_general_entities =
  [ Str.regexp "&amp;", "&";
    Str.regexp "&quot;", "\"" ]

let display_entities = fun s ->
  List.fold_right
    (fun (regexp, displayed_as) r -> Str.global_replace regexp displayed_as r)
    predefined_general_entities
    s

module Gconf = struct
  let get_value = fun xml name ->
    let e = child ~select:(fun x -> Xml.attrib x "name" = name) xml "entry" in
    Xml.attrib e "value"

  let entry = fun application name value ->
    Xml.Element ("entry", ["name", name;
                           "value", value;
                           "application", application],
                 [])

  let add_entry = fun xml appli name value ->
    let entry = entry appli name value in
    let select = fun x -> Xml.attrib x "name" = name in
    let xml = remove_child ~select "entry" xml in
    Xml.Element (Xml.tag xml, Xml.attribs xml, entry::Xml.children xml)
end
