(*
 *  $Id$
 *
 * Hardware In The Loop
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

open Printf
open Stdlib
module LL = Latlong

module TelePprz = Pprz.Messages(struct let name = "telemetry" end)
module DatalinkPprz = Pprz.Messages(struct let name = "datalink" end)
module GroundPprz = Pprz.Messages(struct let name = "ground" end)

module Make (A:Data.MISSION) (FM: FlightModel.SIG) = struct
  let my_id = ref (-1)

  let init = fun id (_:GPack.box) ->
    my_id := id

  let boot = fun time_scale -> ()

  let scale = fun value s -> truncate (value *. s)

  open Gps

  let array_of_string = fun s ->
    Array.init (String.length s) (fun i -> Pprz.Int (Char.code s.[i]))

  let send_raw = fun m ->
    let vs = ["ac_id",Pprz.Int !my_id; "message", Pprz.String m] in
    GroundPprz.message_send "hitl" "RAW_DATALINK" vs

  let datalink_message = fun msg_name vs ->
    let (mid, m) = DatalinkPprz.message_of_name msg_name in
    DatalinkPprz.string_of_message ~sep:";" m vs

  let send_ubx_message = fun class_name msg_name vs ->
    let class_id, msg_id, msg_spec = Ubx.message class_name msg_name in
    let ubx_payload = Ubx.ubx_payload msg_spec vs in
    Debug.call 'h' (fun f -> fprintf f "ubx_payload: %d:%s\n" msg_id (Debug.xprint ubx_payload));
    let a = array_of_string ubx_payload in
    let s = Pprz.string_of_value (Pprz.Array a) in
    let vs = ["class", Pprz.Int class_id;
	      "id", Pprz.Int msg_id;
	      "ubx_payload", Pprz.String s] in
    let m = datalink_message "HITL_UBX" vs in
    send_raw m
	

  let gps = fun gps ->
    let utm = LL.utm_of LL.WGS84 gps.wgs84 in

    send_ubx_message "NAV" "POSUTM" ["EAST", scale utm.LL.utm_x 1e2;
				     "NORTH", scale utm.LL.utm_y 1e2;
				     "ALT", scale gps.alt 1e2;
				     "ZONE", utm.LL.utm_zone];
    send_ubx_message  "NAV" "STATUS" ["GPSfix", 3];
    send_ubx_message  "NAV" "VELNED" ["ITOW",scale gps.time 1e3;
				      "VEL_D", -scale gps.climb 1e2;
				      "GSpeed", scale gps.gspeed 1e2;
				      "Heading", scale (deg_of_rad gps.course) 1e5]
	
  let infrared = fun ir_left ir_front ir_top ->
    let m =
      datalink_message "HITL_INFRARED"
	["roll", Pprz.Int (truncate ir_left);
	 "pitch", Pprz.Int (truncate ir_front)] in
    send_raw m

  let sep_reg = Str.regexp Pprz.separator
  let read_commands = fun commands _sender values ->
    let s = Pprz.string_assoc "values" values in
    let vs = Array.of_list (List.map int_of_string (Str.split sep_reg s)) in
    Array.blit vs 0 commands 0 (Array.length vs)

  let commands = fun commands ->
    ignore (TelePprz.message_bind "COMMANDS" (read_commands commands))
end
