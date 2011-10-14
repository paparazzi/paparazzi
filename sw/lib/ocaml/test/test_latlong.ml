open Latlong
open Printf

let feq2 = fun x x' -> abs_float (x-.x') < 1e-2
let feq8 = fun x x' -> abs_float (x-.x') < 1e-8
let array_feq2 = fun e e' ->
  feq2 e.(0) e'.(0) && feq2 e.(1) e'.(1) && feq2 e.(2) e'.(2)
let ecef_eq = fun e e' ->
  let e = array_of_ecef e
  and e' = array_of_ecef e' in
  array_feq2 e e'
let ned_eq = fun e e' ->
  let e = array_of_ned e
  and e' = array_of_ned e' in
  array_feq2 e e'
let llh_eq = fun (ll, h) (ll', h') ->
  feq8 ll.posn_lat ll'.posn_lat &&
  feq8 ll.posn_long ll'.posn_long &&
  feq2 h h'

let () =
  let muret_geo = make_geo_deg 43.46223 1.27289
  and muret_h = 185. in

  printf "Muret LLH: %s %.2f\n%!" (string_degrees_of_geographic muret_geo) muret_h;

  let muret_ecef = ecef_of_geo WGS84 muret_geo muret_h in
  printf "Muret ECEF: %a\n%!" fprint_ecef muret_ecef;

  (* http://www.apsalin.com/convert-geodetic-to-cartesian.aspx *)
  assert (ecef_eq muret_ecef (make_ecef [|4635769.92611344; 103005.774929684; 4365044.16221717|]));

  let muret_geo', muret_h' = geo_of_ecef WGS84 muret_ecef in
  printf "Muret LLH': %s %.2f\n%!" (string_degrees_of_geographic muret_geo')muret_h';

  assert (llh_eq (muret_geo, muret_h) (muret_geo', muret_h'));

  let ecef_of_ned_muret = ecef_of_ned muret_ecef
  and ned_of_ecef_muret = ned_of_ecef muret_ecef in

  let muret_utm = utm_of WGS84 muret_geo in
  printf "Muret UTM: %a\n%!" fprint_utm muret_utm;

  for d = 0 to 5 do
    let d = 10.**float d in
    printf "d=%.0fm\n" d;
    for n = -1 to 1 do
      let n = d *. float n in
      for e = -1 to 1 do
	let e = d *. float e in
	for d = 0 to 1 do
	  let d = float (-100 * d) in
	  let plane_ned = make_ned [| n; e; d|] in

	  printf "    Plane NED: %a\n%!" fprint_ned plane_ned;
	  let plane_ecef = ecef_of_ned_muret plane_ned in
	  (* printf "Plane ECEF: %a\n%!" fprint_ecef plane_ecef; *)

	  let plane_ned' = ned_of_ecef_muret plane_ecef in
	  (* printf "Plane NED': %a\n%!" fprint_ned plane_ned'; *)
	  assert (ned_eq plane_ned plane_ned');

  (* Angles in UTM and NED *)
	  let plane_geo, plane_h = geo_of_ecef WGS84 plane_ecef in
	  printf "    Plane LLH: %s %.2f\n%!" (string_degrees_of_geographic plane_geo) plane_h;

	  let plane_utm = utm_of WGS84 plane_geo in


	  printf "    Plane UTM: %a\n%!" fprint_utm plane_utm;
	  printf "Delta UTM [%.2f %.2f]\n%!" (plane_utm.utm_x -. muret_utm.utm_x -. e) (plane_utm.utm_y -. muret_utm.utm_y -. n);
	  printf "Delta alt: %.2fm\n" (plane_h -. muret_h +. d)
	done
      done
    done
  done
