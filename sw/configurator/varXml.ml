(*
 *  $Id$
 *
 * Mutable XML representation based on TK Textvariable
 *  
 * Copyright (C) 2004 Pascal Brisset, Antoine Drouin
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

type xml = 
  | Element of (string * (string * Textvariable.textVariable) list * xml list)
  | PCData of string

let empty = PCData "Empty"

let variable = fun s ->
  let t = Textvariable.create () in
  Textvariable.set t s;
  t

let rec of_xml = function
  | Xml.Element (tag, attributes, children) ->
      let varattrs = List.map (fun (a, v) -> (a, variable v)) attributes in
      Element (tag, varattrs, List.map of_xml children)
  | Xml.PCData string -> PCData string

let rec to_xml = function
  | Element (tag, attributes, children) ->
      let varattrs = List.map (fun (a, v) -> (a, Textvariable.get v)) attributes in
      Xml.Element (tag, varattrs, List.map to_xml children)
  | PCData string -> Xml.PCData string

let attrib xml a =
  match xml with
  | Element (_tag, attributes, _children) ->
      List.assoc a attributes
  | _ -> failwith "VarXml.attrib"

let children xml =
  match xml with
  | Element (_tag, _attributes, children) -> children
  | _ -> failwith "VarXml.children"

let child xml ?select c =
  let rec find = function
      Element (tag, attributes, _children) as elt :: elts ->
	if tag = c then
	  match select with
	    None -> elt
	  | Some p ->
	      if p attributes then elt else find elts
	else
	  find elts
    | _ :: elts -> find elts
    | [] -> raise Not_found in
  find (children xml)



let slash = Str.regexp "/"

let get = fun path attr root ->
  let path = Str.split slash path in
  let rec find = fun path attributes xmls ->
    match path, xmls with
      [], _ ->  List.assoc attr attributes
    | tag::tags, Element (tag', attributes', children)::elements ->
	if tag = tag' then
	  find tags attributes' children
	else
	  find path attributes elements
    | tag::_, _ -> failwith ("VarXml.get "^tag) in
  find path [] [root]
	
