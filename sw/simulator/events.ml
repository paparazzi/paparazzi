(*
 *  $Id$
 *
 * High-level events handling
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

type callback = unit -> unit
type time = float (* Unix time *)
type period = float (* Seconds *)

type fd = Unix.file_descr

type timer = {
    mutable next_wake : time;
    period : period;
    cb : callback
  }


type on_input_id = Unix.file_descr
let dummy_on_input_id = Unix.stdin
let on_fds = Hashtbl.create 11
let register_on_input fd cb =
  Hashtbl.add on_fds fd cb; fd
let remove_on_input fd =
  Hashtbl.remove on_fds fd

type timer_id = callback
let timers = ref [] (* Is a priority queue really necessary ? *)
let register_timer period cb =
  timers := { next_wake = Unix.gettimeofday () +. period; period = period; cb = cb } :: !timers; cb
let remove_timer cb =
  let rec loop = function
      [] -> []
    | t::ts -> if t.cb == cb then ts else t :: loop ts in
  timers := loop !timers
  

let get_input_fds () =
  let l = ref [] in
  Hashtbl.iter (fun fd _ -> l := fd :: !l) on_fds;
  !l
let get_fd_callbacks fds =
  List.map (Hashtbl.find on_fds) fds

(** Returns next timer timeout and callback. May return a dummy callback
   if no timers are set. Wrap the update of the selected timer in the
   callback *)
let never = 2e9
let get_next_timeout () =
  let rec loop earlier_wake earlier_cb = function
      [] -> (earlier_wake, earlier_cb)
    | timer :: es ->
	if timer.next_wake < earlier_wake then
	  let t = timer.next_wake in
	  loop t (fun () -> timer.next_wake <- t +. timer.period; timer.cb ()) es
	else
	  loop earlier_wake earlier_cb es in
  loop never (fun () -> ()) !timers
	
	






let mainloop () =
  while true do
    let (next_timeout, timeout_cb) = get_next_timeout () in
    let timeout = next_timeout -. Unix.gettimeofday () in
    if timeout <= 0. then
      timeout_cb ()
    else
      let input_fds = get_input_fds () in
      let (ready_inputs, _, _) = Unix.select input_fds [] [] timeout in
      
      match ready_inputs with
	[] -> timeout_cb ()
      | _ -> 
	  let fd_callbacks = get_fd_callbacks ready_inputs in
	  List.iter (fun cb -> cb ()) fd_callbacks
  done


