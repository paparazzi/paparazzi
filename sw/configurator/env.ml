(*
 *  $Id$
 *
 * Global and default settings
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

let pprz_dir =
  try
    Sys.getenv "PAPARAZZI_DIR"
  with
    Not_found ->
      Filename.concat (Filename.dirname Sys.argv.(0)) "../.."

let abs = fun x -> pprz_dir ^ "/" ^ x

let configurator_dir = abs "sw/configurator"

let fbw_dir = abs "sw/airborne/fly_by_wire"
let ap_dir = abs "sw/airborne/autopilot"
let modem_dir = abs "sw/ground_segment/modem"

let fbw_tty = "/dev/ttyS1"
let ap_tty = "/dev/ttyS0"
let modem_tty = "/dev/ttyUSB0"

let tty_rate = Serial.B38400


(* Initialization very early for creation of Textvariables *)
let _ = Tk.openTk ()

let select_one_file = fun ?(filter="*.xml") use ->
  let action = function 
      [] -> ()
    | [f] -> use f
    | _ -> failwith "Env.select_one_file: unepected several files" in
  Fileselect.f ~title:"File Selection" ~action ~filter ~file:"" ~multi:false ~sync:false
