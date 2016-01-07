(*
 * World environment (time, wind, ...) for multi-AC simulation
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
open Latlong

let my_id = "gaia"
let sending_period = 5000 (* ms *)

module Ground_Pprz = PprzLink.Messages(struct let name = "ground" end)

let ivy_bus = ref Defivybus.default_ivy_bus
let time_scale = ref 1.
let wind_speed = ref 0.
let wind_dir = ref 0.
let wind_up = ref 0.
let gps_off = ref false

let parse_args = fun () ->
  let options =
    [ "-b", Arg.String (fun x -> ivy_bus := x), (sprintf "Bus\tDefault is %s" !ivy_bus);
      "-t", Arg.Set_float time_scale, "Set time scale (default: 1.0)";
      "-w", Arg.Set_float wind_speed, "Set wind speed (0-30m/s)";
      "-d", Arg.Set_float wind_dir, "Set wind direction 0-359 deg";
      "-u", Arg.Set_float wind_up, "Set wind updraft (-10 to 10m/s)";
      "-g", Arg.Set gps_off, "Turn off GPS";
    ] in
  Arg.parse (options)
    (fun x -> Printf.fprintf stderr "Warning: Don't do anythig with %s\n" x)
    "Usage: "

let _ =
  parse_args ();
  let icon = GdkPixbuf.from_file Env.icon_file in
  let window = GWindow.window ~icon ~title:"Gaia" () in
  let quit = fun () -> GMain.Main.quit (); exit 0 in
  ignore (window#connect#destroy ~callback:quit);

  let time_scale_adj = GData.adjustment ~page_size:0. ~value:!time_scale ~lower:(0.5) ~upper:10. ~step_incr:0.5 () in
  let wind_dir_adj = GData.adjustment ~page_size:0. ~value:!wind_dir ~lower:(0.) ~upper:359. ~step_incr:1.0 () in
  let wind_speed_adj = GData.adjustment ~page_size:0. ~value:!wind_speed ~lower:(0.) ~upper:30. ~step_incr:0.1 () in
  let wind_up_adj = GData.adjustment ~page_size:0. ~value:!wind_up ~lower:(-10.) ~upper:10. ~step_incr:0.1 () in
  let infrared_contrast_adj = GData.adjustment ~page_size:0. ~value:266. ~lower:(0.) ~upper:1010. ~step_incr:10. () in
  let gps_sa = GButton.toggle_button ~label:"GPS OFF" ~active:!gps_off () in

  let world_values = fun asker_values ->
    let wind_speed = wind_speed_adj#value
    and wind_dir_rad = Latlong.pi /. 2. -. (Deg>>Rad) wind_dir_adj#value in

    let wind_east = -. wind_speed  *. cos wind_dir_rad
    and wind_north = -. wind_speed *. sin wind_dir_rad in
    let wind_up = wind_up_adj#value in

    [ "wind_east", PprzLink.Float wind_east;
      "wind_north", PprzLink.Float wind_north;
      "wind_up", PprzLink.Float wind_up;
      "ir_contrast", PprzLink.Float infrared_contrast_adj#value;
      "time_scale", PprzLink.Float time_scale_adj#value;
      "gps_availability", PprzLink.Int (if gps_sa#active then 0 else 1)
    ] in
  let world_send = fun () ->
    Ground_Pprz.message_send my_id "WORLD_ENV" (world_values []) in

  List.iter
    (fun (a:GData.adjustment) -> ignore (a#connect#value_changed world_send))
    [time_scale_adj; wind_dir_adj; wind_speed_adj; wind_up_adj; infrared_contrast_adj];
  ignore (gps_sa#connect#toggled world_send);

  ignore (Glib.Timeout.add sending_period (fun () -> world_send (); true));

  let vbox = GPack.vbox ~packing:window#add () in

  let hbox = GPack.hbox ~packing:vbox#pack () in
  let _ =  GMisc.label ~text:"Time scale:" ~packing:hbox#pack () in
  let _ts = GEdit.spin_button ~adjustment:time_scale_adj ~packing:hbox#add () in

  let hbox = GPack.hbox ~packing:vbox#pack () in
  ignore (GMisc.label ~text:"Wind dir:" ~packing:hbox#pack ());
  ignore (GRange.scale ~digits:0 `HORIZONTAL ~adjustment:wind_dir_adj ~packing:hbox#add ());

  let hbox = GPack.hbox ~packing:vbox#pack () in
  ignore (GMisc.label ~text:"Wind speed:" ~packing:hbox#pack ());
  ignore (GRange.scale `HORIZONTAL ~adjustment:wind_speed_adj ~packing:hbox#add ());

  let hbox = GPack.hbox ~packing:vbox#pack () in
  ignore (GMisc.label ~text:"Wind up:" ~packing:hbox#pack ());
  ignore (GRange.scale `HORIZONTAL ~adjustment:wind_up_adj ~packing:hbox#add ());

  vbox#pack gps_sa#coerce;

  Ivy.init "Paparazzi gaia" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  ignore (Ground_Pprz.message_answerer my_id "WORLD_ENV" (fun _ -> world_values));

  window#show ();
  Unix.handle_unix_error GMain.Main.main ()
