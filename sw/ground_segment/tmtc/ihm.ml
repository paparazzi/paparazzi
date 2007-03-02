open Latlong

module GroundPprz = Pprz.Messages(struct let name = "ground" end)
module IhmUpPprz = Pprz.Messages(struct let name = "ihm_up" end)
module IhmDownPprz = Pprz.Messages(struct let name = "ihm_down" end)


type point = { x: int; y: int; z: int }
type pattern =
    Circle of point * int
  | Eight of point * point * int
  | Line of point * point
  | Nop


(** Hashtbl of timelines indexed by the aircraft id *)
let timelines = Hashtbl.create 3
let timeline_max_length = 8

(* FIXME (from flight_plans/ihm.xml) *)
let nop_block = 0
let circle_block = 1
let eight_block = 2
let glide_block = 3
let end_glide_block = 4

(* FIXME (from settings/ihm.xml) *)
let nav_radius_id = 0

(* FIXME *)
let utm_ref = utm_of WGS84 { posn_lat=(Deg>>Rad)43.4624; posn_long=(Deg>>Rad)1.2727 }
let geo_of = fun p -> of_utm WGS84 (utm_add utm_ref (float p.x, float p.y))

let send_circle = fun ac_id p r ->
  let wgs84 = geo_of p in
  let vs = [ "ac_id", Pprz.String ac_id;
	     "wp_id", Pprz.Int 1; (* FIXME *)
	     "alt", Pprz.Float (float p.z);
	     "lat", Pprz.Float ((Rad>>Deg)wgs84.posn_lat);
	     "long", Pprz.Float ((Rad>>Deg)wgs84.posn_long)] in
  GroundPprz.message_send "ihm" "MOVE_WAYPOINT" vs;
  
  let vs = [ "ac_id", Pprz.String ac_id;
	     "index", Pprz.Int nav_radius_id;
	     "value", Pprz.Float (float r) ] in
  GroundPprz.message_send "ihm" "DL_SETTING" vs;
  
  let vs = [ "ac_id", Pprz.String ac_id;
	     "block_id", Pprz.Int circle_block ] in
	  GroundPprz.message_send "ihm" "JUMP_TO_BLOCK" vs
    

let send_line = fun ac_id p1 p2 ->
  let wgs84_1 = geo_of p1
  and wgs84_2 = geo_of p2 in
  let vs = [ "ac_id", Pprz.String ac_id;
	     "wp_id", Pprz.Int 1; (* FIXME *)
	     "alt", Pprz.Float (float p1.z);
	     "lat", Pprz.Float ((Rad>>Deg)wgs84_1.posn_lat);
	     "long", Pprz.Float ((Rad>>Deg)wgs84_1.posn_long)] in
  GroundPprz.message_send "ihm" "MOVE_WAYPOINT" vs;

  let vs = [ "ac_id", Pprz.String ac_id;
	     "wp_id", Pprz.Int 2; (* FIXME *)
	     "alt", Pprz.Float (float p2.z);
	     "lat", Pprz.Float ((Rad>>Deg)wgs84_2.posn_lat);
	     "long", Pprz.Float ((Rad>>Deg)wgs84_2.posn_long)] in
  GroundPprz.message_send "ihm" "MOVE_WAYPOINT" vs;
  
  let vs = [ "ac_id", Pprz.String ac_id;
	     "block_id", Pprz.Int glide_block ] in
  GroundPprz.message_send "ihm" "JUMP_TO_BLOCK" vs
    

let send_eight = fun ac_id p1 p2 r ->
  let wgs84_1 = geo_of p1
  and wgs84_2 = geo_of p2 in
  let vs = [ "ac_id", Pprz.String ac_id;
	     "wp_id", Pprz.Int 1; (* FIXME *)
	     "alt", Pprz.Float (float p1.z);
	     "lat", Pprz.Float ((Rad>>Deg)wgs84_1.posn_lat);
	     "long", Pprz.Float ((Rad>>Deg)wgs84_1.posn_long)] in
  GroundPprz.message_send "ihm" "MOVE_WAYPOINT" vs;

  let vs = [ "ac_id", Pprz.String ac_id;
	     "wp_id", Pprz.Int 2; (* FIXME *)
	     "alt", Pprz.Float (float p2.z);
	     "lat", Pprz.Float ((Rad>>Deg)wgs84_2.posn_lat);
	     "long", Pprz.Float ((Rad>>Deg)wgs84_2.posn_long)] in
  GroundPprz.message_send "ihm" "MOVE_WAYPOINT" vs;
  
  let vs = [ "ac_id", Pprz.String ac_id;
	     "index", Pprz.Int nav_radius_id;
	     "value", Pprz.Float (float r) ] in
  GroundPprz.message_send "ihm" "DL_SETTING" vs;
  
  let vs = [ "ac_id", Pprz.String ac_id;
	     "block_id", Pprz.Int eight_block ] in
  GroundPprz.message_send "ihm" "JUMP_TO_BLOCK" vs
    

let send_pattern_up = fun ac_id ->
  try
    let tl = Hashtbl.find timelines ac_id in
    begin
      match tl.(0) with
	Circle (p, r) -> send_circle ac_id p r
      | Eight (p1, p2, r) -> send_eight ac_id p1 p2 r
      | Line (p1, p2) -> send_line ac_id p1 p2
      | Nop ->
	  let vs = [ "ac_id", Pprz.String ac_id;
		     "block_id", Pprz.Int nop_block ] in
	  GroundPprz.message_send "ihm" "JUMP_TO_BLOCK" vs
    end
  with
    Not_found -> failwith (Printf.sprintf "send_pattern_up: %s" ac_id)

let get_ac = fun vs ->
  let ac_id = Pprz.string_assoc "ac_id" vs in
  try
    Hashtbl.find timelines ac_id
  with
    Not_found ->
      let t = Array.create timeline_max_length Nop in
      Hashtbl.add timelines ac_id t;
      t


(** Bind to message while catching all the esceptions of the callback *)
let safe_bind = fun msg cb bind ->
  let safe_cb = fun sender vs ->
    try cb sender vs with x -> prerr_endline (Printexc.to_string x) in
  ignore (bind msg safe_cb)

let safe_bind_up = fun msg cb ->
  safe_bind msg cb IhmUpPprz.message_bind

let safe_bind_ground = fun msg cb ->
  safe_bind msg cb GroundPprz.message_bind


let point_assoc = fun x y z values ->
  { x = Pprz.int_assoc x values;
    y = Pprz.int_assoc y values;
    z = Pprz.int_assoc z values }


let ihm_circle_cb = fun _sender values ->
  let timeline = get_ac values in
  let idx = Pprz.int_assoc "idx_timeline" values
  and p = point_assoc "x" "y" "z" values
  and r = Pprz.int_assoc "r" values in
  assert(idx < timeline_max_length);
  timeline.(idx) <- Circle (p, r);
  if idx = 0 then
    send_pattern_up (Pprz.string_assoc "ac_id" values)

let ihm_line_cb = fun _sender values ->
  let timeline = get_ac values in
  let idx = Pprz.int_assoc "idx_timeline" values
  and p1 = point_assoc "x1" "y1" "z1" values
  and p2 = point_assoc "x2" "y2" "z2" values in
  assert(idx < timeline_max_length);
  timeline.(idx) <- Line (p1, p2);
  if idx = 0 then
    send_pattern_up (Pprz.string_assoc "ac_id" values)

let ihm_eight_cb = fun _sender values ->
  let timeline = get_ac values in
  let idx = Pprz.int_assoc "idx_timeline" values
  and p1 = point_assoc "x1" "y1" "z" values
  and p2 = point_assoc "x2" "y2" "z" values
  and r = Pprz.int_assoc "r" values in
  assert(idx < timeline_max_length);
  timeline.(idx) <- Eight (p1, p2, r);
  if idx = 0 then
    send_pattern_up (Pprz.string_assoc "ac_id" values)

let delete = fun timeline idx values ->
  (* Shift left *)
  for i = max 0 idx to timeline_max_length - 2 do
    timeline.(i) <- timeline.(i+1)
  done;
  if idx = 0 then
    send_pattern_up (Pprz.string_assoc "ac_id" values)

let ihm_delete_cb = fun _sender values ->
  let timeline = get_ac values in
  let idx = Pprz.int_assoc "idx_timeline" values in
  delete timeline idx values

let nav_status_cb = fun _sender values ->
  let block = Pprz.int_assoc "cur_block" values in
  if block = end_glide_block then
    let timeline = get_ac values in
    delete timeline 0 values

let listen = fun () ->
  safe_bind_up "IHM_CIRCLE" ihm_circle_cb;
  safe_bind_up "IHM_LINE" ihm_line_cb;
  safe_bind_up "IHM_EIGHT" ihm_eight_cb;
  safe_bind_up "IHM_DELETE" ihm_delete_cb;
  safe_bind_ground "NAV_STATUS" nav_status_cb



(*** Options ***)
let ivy_bus = ref "127.255.255.255:2010"
let options =
  [ "-b", Arg.String (fun x -> ivy_bus := x), (Printf.sprintf "Bus\tDefault is %s" !ivy_bus)]


(************** Main ****************)
let () = 
  Arg.parse options
    (fun x -> Printf.fprintf stderr "%s: Warning: Don't do anything with '%s' argument\n" Sys.argv.(0) x)
    "Usage: ";

  Ivy.init "Paparazzi server" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  listen ();

  (* Main loop *)
  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do ignore (Glib.Main.iteration true) done
