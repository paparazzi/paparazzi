(*
 * Copyright (C) 2007- ENAC, Pascal Brisset, Antoine Drouin
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

(** Encode telemetry messages in an audio stream (to be mixed with a
    video stream). Listen messages from the "ground" class (a server
    must be running) and write message(s) of the "DIA" class.
*)

open Printf

let msg_period = 500 (* ms *)
let ac_id = ref 1

module Ground_Pprz = Pprz.Messages(struct let name = "ground" end)
module Sub_Pprz = Pprz.Messages(struct let name = "DIA" end)

type state = {
  mutable lat : float;
  mutable long : float;
  mutable alt : int;

  mutable course : int;
  mutable speed : int;

  mutable cam_roll : int;
  mutable cam_pitch : int;
}

let state = {
  lat = 0.; long = 0.; alt = 0;
  course = 0; speed = 0;
  cam_roll = 0; cam_pitch = 0;
}

let msg_id, _ = Sub_Pprz.message_of_name "NAV_INFO"
let send_msg = fun () ->
  let t = (Unix.gettimeofday ()) -. 1e9 in
  let vs = [
    "unix_time", Pprz.Float t;

    "lat", Pprz.Float state.lat;
    "long", Pprz.Float state.long;
    "alt", Pprz.Int state.alt;

    "course", Pprz.Int state.course;
    "speed", Pprz.Int state.speed;

    "cam_roll", Pprz.Int state.cam_roll;
    "cam_pitch", Pprz.Int state.cam_pitch
  ] in
  let s = Sub_Pprz.payload_of_values msg_id !ac_id vs in
  Debug.call 'l' (fun f ->  fprintf f "sending: %s\n" (Debug.xprint (Serial.string_of_payload s)));
  Hdlc.write_data (Serial.string_of_payload s)

let fp_msg = fun _sender vs ->
  if int_of_string (Pprz.string_assoc "ac_id" vs) = !ac_id then begin
    state.lat <- Pprz.float_assoc "lat" vs;
    state.long <- Pprz.float_assoc "long" vs;
    state.alt <- truncate (Pprz.float_assoc "alt" vs);
    state.course <- truncate (Pprz.float_assoc "course" vs);
    state.speed <- truncate (Pprz.float_assoc "speed" vs *. 100.);
    state.cam_roll <- truncate (Pprz.float_assoc "roll" vs); (* FIXME *)
    state.cam_pitch <- truncate (Pprz.float_assoc "pitch" vs); (* FIXME *)
  end





let _ =
  let ivy_bus = ref Defivybus.default_ivy_bus
  and port = ref "/dev/dsp" in
  let options = [
    "-b", Arg.Set_string ivy_bus, (sprintf "<ivy bus> Default is %s" !ivy_bus);
    "-d", Arg.Set_string port, (sprintf "<port> Default is %s" !port);
    "-ac", Arg.Set_int ac_id, (sprintf "<A/C id> Default is %d" !ac_id);
  ] in

  Arg.parse
    options
    (fun _x -> ())
    "Usage: ";

  (* Connect to Ivy bus *)
  Ivy.init "Link" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  (* Listen for telemetry messages *)
  ignore (Ground_Pprz.message_bind "FLIGHT_PARAM" fp_msg);

  (* Open the audio output and launch the periodic writings *)
  let _fd = Hdlc.init_gen "/dev/dsp" in
  ignore (Glib.Timeout.add Hdlc.write_period (fun _ -> Hdlc.write_to_dsp (); true));

  (* Periodically send messages *)
  ignore (Glib.Timeout.add msg_period (fun () -> send_msg (); true));


    (* Main Loop *)
  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do
    ignore (Glib.Main.iteration true)
  done
