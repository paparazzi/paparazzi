
open Printf

module Ground_Pprz = PprzLink.Messages(struct let name = "ground" end)
module LL = Latlong
open LL

type plume = { mutable utm_x : float; mutable utm_y : float; mutable value : int; utm_zone : int }

(* NW of Muret ref *)
let muret = utm_of WGS84 {LL.posn_lat=(Deg>>Rad)43.4624; posn_long=(Deg>>Rad)1.2727}
let royan = utm_of WGS84 {LL.posn_lat=(Deg>>Rad)45.7122; posn_long=(Deg>>Rad)(-1.2037)}
let source = fun () -> { utm_x = muret.LL.utm_x -. 300.; utm_y = muret.LL.utm_y -. 300.; value = 255; utm_zone = muret.LL.utm_zone}

let available_ids = ref []
let gen_id =
  let x = ref 0 in
  fun () ->
    match !available_ids with
      [] ->
	incr x; !x
    | x::xs ->
	available_ids := xs;
	x

let dt = 1. (* s *)
let mixing_length = 5. (* m/s *)
let wind_x = ref 0.
let wind_y = ref 0.


let ivy_bus = Defivybus.default_ivy_bus

let plumes = Hashtbl.create 97
let t = ref 0

let one_step = fun () ->
  incr t;

  (* New plume *)
  if !t mod 5 = 0 then
    Hashtbl.add plumes (gen_id ()) (source ());

  (* Eddy + wind *)
  Hashtbl.iter (fun id plume ->
    let a = Random.float (2.*.LL.pi) in
    plume.utm_x <- plume.utm_x +. (mixing_length*.cos a +. !wind_x)*. dt;
    plume.utm_y <- plume.utm_y +. (mixing_length*.sin a +. !wind_y)*. dt;
    plume.value <- plume.value - 1;
    if plume.value <= 0 then begin
      Hashtbl.remove plumes id;
      available_ids := id :: !available_ids;
    end)
    plumes


let my_id = "diffusion"
let send_on_ivy = fun () ->
  if Hashtbl.length plumes > 0 then
  let ids = ref []
  and xs= ref []
  and ys = ref []
  and vs = ref [] in
  Hashtbl.iter (fun id plume ->
    ids := string_of_int id :: !ids;
    let wgs84 = LL.of_utm WGS84 {LL.utm_zone = plume.utm_zone ; utm_x = plume.utm_x; utm_y = plume.utm_y } in
    xs := sprintf "%.6f" ((Rad>>Deg)wgs84.posn_lat) :: !xs;
    ys := sprintf "%.6f" ((Rad>>Deg)wgs84.posn_long) :: !ys;
    vs := sprintf "%d" plume.value :: !vs)
    plumes ;
  let ids = Bytes.concat "," !ids
  and xs = Bytes.concat "," !xs
  and ys = Bytes.concat "," !ys
  and vs = Bytes.concat "," !vs in
  Ground_Pprz.message_send my_id "PLUMES"
    [ "ids", PprzLink.String ids;
      "lats", PprzLink.String xs;
      "longs", PprzLink.String ys;
      "values", PprzLink.String vs ]

let debug = let i = ref 0 in fun () ->
  incr i;
  let f = open_out (sprintf "plumes_%d.txt" !i) in
  Hashtbl.iter (fun id plume ->
    fprintf f "%.1f %.1f\n" plume.utm_x plume.utm_y)
    plumes;
  close_out f

let detect_distance = 20.


let flight_param_msg = fun _sender vs ->
  let lat = PprzLink.float_assoc "lat" vs
  and long = PprzLink.float_assoc "long" vs in
  let utm_ac = utm_of WGS84 {LL.posn_lat=(Deg>>Rad)lat; posn_long=(Deg>>Rad)long} in
  Hashtbl.iter (fun id plume ->
    let utm_plume = {LL.utm_zone = plume.utm_zone; utm_x = plume.utm_x; utm_y = plume.utm_y } in
    let d = utm_distance utm_ac utm_plume in
    if d < detect_distance then begin
      let ac_id = PprzLink.string_assoc "ac_id" vs in
      for i = 0 to 2 do
	Ground_Pprz.message_send my_id "DL_SETTING"
	  ["ac_id", PprzLink.String ac_id; "index", PprzLink.Int i(***FIXME***); "value", PprzLink.Float (float plume.value)]
      done
    end)
    plumes



let safe_bind = fun msg cb ->
  let safe_cb = fun sender vs ->
    try cb sender vs with x -> prerr_endline (Printexc.to_string x) in
  ignore (Ground_Pprz.message_bind msg safe_cb)

let gaia = fun time_scale _sender vs ->
  time_scale#set_value (PprzLink.float_assoc "time_scale" vs);
  wind_x := (PprzLink.float_assoc "wind_east" vs);
  wind_y := (PprzLink.float_assoc "wind_north" vs)


let _ =
  Ivy.init "Paparazzi gaia" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  let periodic = fun () ->
    one_step ();
    send_on_ivy ();
    (*** debug ();***)
    true in

  let time_scale = object val mutable v = 1. method value = v method set_value x = v <- x end in

  Stdlib.timer ~scale:time_scale dt periodic;

  safe_bind "FLIGHT_PARAM" flight_param_msg;
  safe_bind "WORLD_ENV" (gaia time_scale);

  Unix.handle_unix_error GMain.Main.main ()
