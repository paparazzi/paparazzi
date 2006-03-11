(*
 * $Id$
 *
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

exception Error of string

let sep = Str.regexp "\\."

let child xml ?select c =
  let rec find = function
      Xml.Element (tag, _attributes, _children) as elt :: elts ->
	if tag = c then
	  match select with
	    None -> elt
	  | Some p ->
	      if p elt then elt else find elts
	else
	  find elts
    | _ :: elts -> find elts
    | [] -> raise Not_found in


  let children = Xml.children xml in

  (* Let's try with a numeric index *)
  try (Array.of_list children).(int_of_string c) with
    Failure "int_of_string" -> (* Bad luck. Go through the children *)
      find children


let get xml path =
  let p = Str.split sep path in
  let rec iter xml = function
      [] -> failwith "ExtXml.get: empty path"
    | [x] -> ( try if Xml.tag xml <> x then raise Not_found else xml with _ -> raise Not_found )
    | x::xs -> iter (child xml x) xs in
  iter xml p

let get_attrib xml path attr =
  Xml.attrib (get xml path) attr

let sprint_fields = fun () l ->
  "<"^
  List.fold_right (fun (a, b) -> (^) (Printf.sprintf "%s=\"%s\" " a b)) l ">"

let attrib = fun x a ->
  try
    Xml.attrib x a
  with
    Xml.No_attribute _ ->
      raise (Error (Printf.sprintf "Error: Attribute '%s' expected in <%a>" a sprint_fields (Xml.attribs x)))

let attrib_or_default = fun x a default ->
  try Xml.attrib x a with _ -> default


let to_string_fmt = fun xml ->
  let l = String.lowercase in
  let rec lower = function
      Xml.PCData _ as x -> x
    | Xml.Element (t, ats, cs) ->
	Xml.Element(l t,
		    List.map (fun (a,v) -> (l a, v)) ats,
		    List.map lower cs) in
  Xml.to_string_fmt (lower xml)


let subst_attrib = fun attrib value xml ->
  let u = String.uppercase in
  let uattrib = u attrib in
  match xml with
    Xml.Element (tag, attrs, children) ->
      let rec loop = function
	  [] -> [(attrib, value)]
	| (a,_v) as c::ats ->
	    if u a = uattrib then loop ats else c::loop ats in
      Xml.Element (tag, 
		   loop attrs,
		   children)
  | Xml.PCData _ -> xml


let subst_child = fun t x xml ->
  let u = String.uppercase in
  match xml with
    Xml.Element (tag, attrs, children) ->
      Xml.Element (tag,
		   attrs,
		   List.map (fun xml -> if u (Xml.tag xml) = u t then x else xml) children)
  | Xml.PCData _ -> xml


let float_attrib = fun xml a ->
  let v = attrib xml a in
  try
    float_of_string v
  with
    _ -> failwith (Printf.sprintf "Error: float expected in '%s'" v)

