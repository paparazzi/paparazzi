(*
 *  $Id$
 *
 * Serial device handling
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

let buffer_len = 256

let ttys = Hashtbl.create 7
let registered = Hashtbl.create 7

let deconnect = fun tty ->
  try
    let fd = Hashtbl.find ttys tty in
    Fileevent.remove_fileinput fd;
    Unix.close fd;
    Hashtbl.remove ttys tty
  with
    Not_found -> ()

let connect = fun tty ->
  if Hashtbl.mem ttys tty then
    deconnect tty;
  let buffer = String.create buffer_len in
  let fd = Serial.opendev tty Env_conf.tty_rate in
  let log = fun () ->
    let n = Unix.read fd buffer 0 buffer_len in
    let s = String.sub buffer 0 n in
    List.iter (fun f -> f s) (Hashtbl.find_all registered tty) in
  Fileevent.add_fileinput fd log;
  Hashtbl.add ttys tty fd
    

let add_ttyinput = Hashtbl.add registered

let add_formatted_input = fun tty prefix size f ->
  if String.length prefix > size then raise (Invalid_argument "add_formatted_input");
  let buffer = String.create size
  and idx = ref 0 in
  let rec f' = fun s ->
    let n = String.length s
    and expected = size - !idx in
    Printf.printf "%d " n; flush stdout;
    let blitted = min expected n in
    String.blit s 0 buffer !idx blitted;
    idx := !idx + blitted;
    if !idx = size then begin
      if String.sub buffer 0 (String.length prefix) = prefix then begin
	f buffer;
	idx := 0
      end else begin (* Look for the  first character of the prefix *)
	try
	  let discarded = String.index_from buffer 1 prefix.[0] in
	  let kept = size - discarded in
	  String.blit buffer discarded buffer 0 kept;
	  idx := kept
	with
	  Not_found -> (* prefix.[0] not found *)
	    idx := 0
      end
    end;
    let rest = n - blitted in
    if rest > 0 then begin Printf.printf "r=%d\n" rest; flush stdout;  f' (String.sub s blitted rest) end in
  add_ttyinput tty f'

let write = fun tty s ->
  let fd = Hashtbl.find ttys tty in
  let oc = Unix.out_channel_of_descr fd in
  Printf.fprintf oc "%s" s;
  flush oc

let write_byte = fun tty b ->
  let fd = Hashtbl.find ttys tty in
  let oc = Unix.out_channel_of_descr fd in
  output_byte oc b;
  flush oc

let flush = fun tty ->
  let fd = Hashtbl.find ttys tty in
  flush (Unix.out_channel_of_descr fd)

