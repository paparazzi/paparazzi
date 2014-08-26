(*
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
open LL

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

  let send_ubx_message = fun class_name msg_name vs ->
    let class_id, msg_id, msg_spec = Ubx.message class_name msg_name in
    let ubx_payload = Ubx.ubx_payload msg_spec vs in
    Debug.call 'h' (fun f -> fprintf f "ubx_payload: %d:%s\n" msg_id (Debug.xprint ubx_payload));
    let a = array_of_string ubx_payload in
    let s = Pprz.string_of_value (Pprz.Array a) in
    let vs = ["ac_id",Pprz.Int !my_id;
	      "msg_class", Pprz.Int class_id;
	      "id", Pprz.Int msg_id;
	      "ubx_payload", Pprz.String s] in

    try
      DatalinkPprz.message_send "hitl" "HITL_UBX" vs
    with
      exc -> prerr_endline (Printexc.to_string exc)


  let gps = fun gps ->
    let utm = LL.utm_of LL.WGS84 gps.wgs84
    and lon = (Rad>>Deg)gps.wgs84.posn_long
    and lat = (Rad>>Deg)gps.wgs84.posn_lat in

    send_ubx_message "NAV" "POSLLH" ["ITOW", scale gps.time 1e3;
				     "LON",scale lon 1e7; (* FIXME TOO LARGE *)
				     "LAT", scale lat 1e7;
				     "HEIGHT", scale gps.alt 1e3;
				     "HMSL", scale gps.alt 1e3;
				     "Hacc", scale 3. 1e3;
				     "Vacc", scale 5. 1e3];
    send_ubx_message "NAV" "POSUTM" ["EAST", scale utm.LL.utm_x 1e2;
				     "NORTH", scale utm.LL.utm_y 1e2;
				     "ALT", scale gps.alt 1e2;
				     "ZONE", utm.LL.utm_zone];
    send_ubx_message  "NAV" "STATUS" ["GPSfix", 3];
    send_ubx_message  "NAV" "VELNED" ["ITOW",scale gps.time 1e3;
				      "VEL_D", -scale gps.climb 1e2;
				      "GSpeed", scale gps.gspeed 1e2;
				      "Heading", scale (deg_of_rad gps.course) 1e5]

  let infrared_and_airspeed = fun ir_left ir_front ir_top _air_speed ->
    try
      DatalinkPprz.message_send "hitl" "HITL_INFRARED"
	["ac_id",Pprz.Int !my_id;
	 "top", Pprz.Int (truncate ir_top);
	 "roll", Pprz.Int (truncate ir_left);
	 "pitch", Pprz.Int (truncate ir_front)]
    with
      exc -> prerr_endline (Printexc.to_string exc)

  let attitude_and_rates = fun phi theta psi p q r ->
    prerr_endline "HITL attitude sim not implemented..."

  let sep_reg = Str.regexp Pprz.separator
  let read_commands = fun commands _sender values ->
    let s = Pprz.string_assoc "values" values in
    let vs = Array.of_list (List.map int_of_string (Str.split sep_reg s)) in
    Array.blit vs 0 commands 0 (Array.length vs)

  let commands = fun commands ->
    ignore (TelePprz.message_bind "COMMANDS" (read_commands commands))
end
