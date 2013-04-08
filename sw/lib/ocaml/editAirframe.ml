(*
 * Handling of an XML airframe file
 *
 * Copyright (C) 2008, Cyril Allignol, Pascal Brisset
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

type units = string option
type data = string * units * units

exception Got_it of data
exception No_param of string

let get = fun xml param ->
  let rec iter_get prefix xml =
    match xml with
        Nethtml.Element ("define", params, children)
          when List.exists (fun (p, v) -> p = "name" && (prefix^v) = param) params ->
            let old_val = snd (List.find (fun (p, v) -> p = "value") params)
            and units =
              try Some (snd (List.find (fun (p, v) -> p = "unit") params))
              with Not_found -> None
            and code_units =
              try Some (snd (List.find (fun (p, v) -> p = "code_unit") params))
              with Not_found -> None
            in
            raise (Got_it (old_val, units, code_units))
      | Nethtml.Element (block, params, children) ->
        let new_prefix =
          List.fold_left (fun acc (p, v) -> if p = "prefix" then v^acc else acc) prefix params in
        List.iter (iter_get new_prefix) children
      | Nethtml.Data s -> ()
  in
  try
    iter_get "" xml;
    raise (No_param param)
  with
      Got_it result  -> result



let set = fun xml param newval ->
  let rec iter_replace prefix xml =
    let update_param params =
      List.map (fun (p, v) -> if p = "value" then (p, newval) else (p, v)) params in
    match xml with
        Nethtml.Element ("define", params, children)
          when List.exists (fun (p, v) -> p = "name" && (prefix^v) = param) params ->
            Nethtml.Element ("define", update_param params, children)
      | Nethtml.Element (block, params, children) ->
        let new_prefix =
          List.fold_left (fun acc (p, v) -> if p = "prefix" then v^acc else acc) prefix params in
        Nethtml.Element (block, params, List.map (iter_replace new_prefix) children)
      | Nethtml.Data s -> Nethtml.Data s
  in
  iter_replace "" xml
