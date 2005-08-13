(*
 * $Id$
 *
 * Multi aircrafts map display
 *  
 * Copyright (C) 2004 CENA/ENAC, Pascal Brisset, Antoine Drouin
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
module Ground_Pprz = Pprz.Protocol(struct let name = "ground" end)

type color = string

let fos = float_of_string
let list_separator = Str.regexp ","

module G = MapCanvas

let home = Env.paparazzi_home
let (//) = Filename.concat
let default_path_SRTM = home // "data" // "SRTM"
let default_path_maps = home // "data" // "maps" // ""
let default_path_missions = home // "conf"

let gen_flight_plan =
  try
    Sys.getenv "PAPARAZZI_SRC" // "sw/tools/gen_flight_plan.out"
  with
    Not_found -> "/usr/share/paparazzi/bin/gen_flight_plan.out"


type aircraft = {
    track : MapTrack.track;
    color: color;
    mutable fp_group : MapWaypoints.group option
  }

let live_aircrafts = Hashtbl.create 3

let map_ref = ref None

let float_attr = fun xml a -> float_of_string (ExtXml.attrib xml a)

let load_map = fun (geomap:G.widget) xml_map ->
  let dir = Filename.dirname xml_map in
  let xml_map = Xml.parse_file xml_map in
  let image = dir // ExtXml.attrib xml_map "file"
  and scale = float_attr xml_map "scale"
  and utm_zone =
    try int_of_string (Xml.attrib xml_map "utm_zone") with
      _ -> 31 in
  geomap#set_world_unit scale;
  let one_ref = ExtXml.child xml_map "point" in
  let x = float_attr one_ref "x" and y = float_attr one_ref "y"
  and utm_x = float_attr one_ref "utm_x" and utm_y = float_attr one_ref "utm_y" in
  let utm_x0 = utm_x -. x *. scale
  and utm_y0 = utm_y +. y *. scale in

  let utm_ref =
    match !map_ref with
      None ->
	let utm0 = {utm_x = utm_x0;  utm_y = utm_y0; utm_zone = utm_zone } in
	map_ref := Some utm0;
	utm0
    | Some utm ->
	assert (utm_zone = utm.utm_zone);
	utm in

  let wgs84_of_en = fun en ->
    of_utm WGS84 {utm_x = utm_ref.utm_x +. en.G.east; utm_y = utm_ref.utm_y +. en.G.north; utm_zone = utm_zone} in

  geomap#set_wgs84_of_en wgs84_of_en;
  let en0 = {G.east=utm_x0 -. utm_ref.utm_x; north=utm_y0 -. utm_ref.utm_y} in
  ignore (geomap#display_map en0 (GdkPixbuf.from_file image));
  geomap#moveto en0


let file_of_url = fun url ->
  if String.sub url 0 7 = "file://" then
    String.sub url 7 (String.length url - 7)
  else
    let tmp_file = Filename.temp_file "fp" ".xml" in
    let c = sprintf "wget -O %s %s" tmp_file url in
    if Sys.command c = 0 then
      tmp_file
    else
      failwith c

let load_mission = fun color geomap url ->
  let file = file_of_url url in
  let xml = Xml.parse_in (Unix.open_process_in (sprintf "%s -dump %s" gen_flight_plan file)) in
  let xml = ExtXml.child xml "flight_plan" in
  let lat0 = float_attr xml "lat0"
  and lon0 = float_attr xml "lon0" in
  let utm0 = utm_of WGS84 {posn_lat = (Deg>>Rad)lat0; posn_long = (Deg>>Rad)lon0 } in
  let waypoints = ExtXml.child xml "waypoints" in
  
  let utm_ref =
    match !map_ref with
      None ->
	map_ref := Some utm0;
	utm0
    | Some utm ->
	assert (utm0.utm_zone = utm.utm_zone);
	utm in
  let en_of_xy = fun x y ->
    {G.east = x +. utm0.utm_x -. utm_ref.utm_x;
     G.north = y +. utm0.utm_y -. utm_ref.utm_y } in

  let fp = new MapWaypoints.group ~color ~editable:false geomap in
  List.iter
    (fun wp ->
      let en = en_of_xy (float_attr wp "x") (float_attr wp "y") in
      let alt = try Some (float_attr wp "alt") with _ -> None in
      ignore (MapWaypoints.waypoint fp ~name:(ExtXml.attrib wp "name") ?alt en)
    )
    (Xml.children waypoints);
  fp


let aircraft_pos_msg = fun track utm_x utm_y heading ->
  match !map_ref with
    None -> ()
  | Some utm0 ->
      let en =  {G.east = utm_x -. utm0.utm_x; north = utm_y -. utm0.utm_y } in
      track#add_point en;
      track#move_icon en heading

let carrot_pos_msg = fun track utm_x utm_y ->
  match !map_ref with
    None -> ()
  | Some utm0 ->
      let en =  {G.east = utm_x -. utm0.utm_x; north = utm_y -. utm0.utm_y } in
      track#move_carrot en

let new_color =
  let colors = ref ["red"; "blue"; "green"] in
  fun () ->
    match !colors with
      x::xs ->
	colors := xs @ [x];
	x
    | [] -> failwith "new_color"


let ask_fp = fun geomap ac ->
  let get_config = fun _sender values ->
    let file = Pprz.string_assoc "flight_plan" values in
    let ac = Hashtbl.find live_aircrafts ac in
    ac.fp_group <- Some (load_mission ac.color  geomap file) in
  Ground_Pprz.message_req "map2d" "CONFIG" ["ac_id", Pprz.String ac] get_config


let show_mission = fun geomap ac on_off ->
  if on_off then
    ask_fp geomap ac
  else
    let a = Hashtbl.find live_aircrafts ac in
    match a.fp_group with
      None -> ()
    | Some g -> 
	a.fp_group <- None;
	g#group#destroy ()

let resize_track = fun ac track ->
  match GToolbox.input_string ~text:(string_of_int track#size) ~title:ac "Track size" with
    None -> ()
  | Some s -> track#resize (int_of_string s)
	 


let live_aircrafts_msg = fun (geomap:MapCanvas.widget) acs ->
  let acs = Pprz.string_assoc "ac_list" acs in
  let acs = Str.split list_separator acs in
  List.iter
    (fun ac ->
      if not (Hashtbl.mem live_aircrafts ac) then begin
	let ac_menu = geomap#factory#add_submenu ac in
	let ac_menu_fact = new GMenu.factory ac_menu in
	let fp = ac_menu_fact#add_check_item "Fligh Plan" ~active:false in
	ignore (fp#connect#toggled (fun () -> show_mission geomap ac fp#active));
	let color = new_color () in
	let track = new MapTrack.track ~name:ac ~color:color geomap in
	ignore (ac_menu_fact#add_item "Clear Track" ~callback:(fun () -> track#clear));
	ignore (ac_menu_fact#add_item "Resize Track" ~callback:(fun () -> resize_track ac track));
	Hashtbl.add live_aircrafts ac { track = track; color = color; fp_group = None }
      end)
    acs


let listen_flight_params = fun () ->
  let get_fp = fun _sender vs ->
    let ac_id = Pprz.string_assoc "ac_id" vs in
    try
      let ac = Hashtbl.find live_aircrafts ac_id in
      let a = fun s -> Pprz.float_assoc s vs in
      aircraft_pos_msg ac.track (a "east") (a "north") (a "heading")
    with Not_found -> ()
  in
  ignore (Ground_Pprz.message_bind "FLIGHT_PARAM" get_fp);

  let get_ns = fun _sender vs ->
    let ac_id = Pprz.string_assoc "ac_id" vs in
    try
      let ac = Hashtbl.find live_aircrafts ac_id in
      let a = fun s -> Pprz.float_assoc s vs in
      carrot_pos_msg ac.track (a "target_east") (a "target_north") 
    with Not_found -> ()
  in
  ignore (Ground_Pprz.message_bind "NAV_STATUS" get_ns)

let _ =
  let ivy_bus = ref "127.255.255.255:2010"
  and map_file = ref ""
  and mission_file = ref "" in
  let options =
    [ "-b", Arg.String (fun x -> ivy_bus := x), "Bus\tDefault is 127.255.255.25:2010";
      "-m", Arg.String (fun x -> map_file := x), "Map description file"] in
  Arg.parse (options)
    (fun x -> Printf.fprintf stderr "Warning: Don't do anythig with %s\n" x)
    "Usage: ";
  (*                                 *)
  Ivy.init "Paparazzi map 2D" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  Srtm.add_path default_path_SRTM;

  let window = GWindow.window ~title: "Map2d" ~border_width:1 ~width:400 () in
  let vbox= GPack.vbox ~packing: window#add () in
  let quit = fun () -> GMain.Main.quit (); exit 0 in
  ignore (window#connect#destroy ~callback:quit);

  let geomap = new MapCanvas.widget ~height:400 () in
  let accel_group = geomap#menu_fact#accel_group in
  ignore (geomap#menu_fact#add_item "Quit" ~key:GdkKeysyms._Q ~callback:quit);

  vbox#pack ~expand:true geomap#frame#coerce;

  (* Loading an initial map *)
  if !map_file <> "" then begin
    let xml_map_file = Filename.concat default_path_maps !map_file in
    load_map geomap xml_map_file
  end;

  ignore (Ground_Pprz.message_bind "AIRCRAFTS" (fun _sender vs -> live_aircrafts_msg geomap vs));

  listen_flight_params ();

  window#add_accel_group accel_group;
  window#show ();
  GMain.Main.main ()
