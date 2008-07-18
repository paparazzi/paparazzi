(*
 * $Id$
 *
 * Geographic conversion utilities
 *  
 * Copyright (C) 2004-2006 ENAC, Pascal Brisset, Antoine Drouin
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

module C = struct
  include Complex
  let make x y = {re = x; im = y}
  let im c = c.im
  let re c = c.re
  let scal a {re = x; im = y} = {re = x*.a; im = y*.a}
  let i {re = x; im = y} = {re = -.y; im = x}
  let sin z =
    let iz = i z in
    i (scal (-. 0.5) (sub (exp iz) (exp (scal (-1.) iz))))
end

let pi = 3.14159265358979323846;;

type degree = float
type radian = float
type semi = float
type dms = int * int * float

type semicircle = { lat : semi; long : semi }
type geographic = { posn_lat : radian ; posn_long : radian }

let norm_angle = fun long ->
  if long >= pi then long -. 2.*.pi 
  else if long < -. pi then long +. 2.*.pi
  else long

let make_geo = fun lat long ->
  { posn_long = norm_angle long; posn_lat = lat }

let valid_geo = fun {posn_long = lambda; posn_lat = phi} ->
  phi >= (-.pi/.2.) && phi <= pi/.2. && lambda >= -.pi && lambda < pi

type angle_unit = Semi | Rad | Deg | Grd

type cartesian = {x : float; y : float; z: float }

let piradian = function
  Semi -> 2. ** 31. | Rad -> pi | Deg -> 180. | Grd -> 200.
let (>>) u1 u2 x = (x *. piradian u2) /. piradian u1;;

let deg_string_of_rad = fun r -> Printf.sprintf "%.6f" ((Rad>>Deg)r)

let decimal d m s = float d +. float m /. 60. +. s /. 3600.;;
let dms = fun x ->
  let d = truncate x in
  let m = truncate ((x -. float d) *. 60.) in
  let s = 3600. *. (x -. float d -. float m /. 60.) in
  (d, m, s);;


let sprint_degree_of_radian x =
  Printf.sprintf "%.6f" ((Rad>>Deg) x)

let string_degrees_of_geographic sm =
  Printf.sprintf "%s\t%s"
    (sprint_degree_of_radian sm.posn_lat) (sprint_degree_of_radian sm.posn_long)

let string_dms_of_geographic = fun geo ->
  let hemi = if geo.posn_lat >= 0. then 'N' else 'S'
  and east = if geo.posn_long >= 0. then 'E' else 'W'
  and lat = abs_float geo.posn_lat
  and lon = abs_float geo.posn_long in
  let (lat_d, lat_m, lat_s) = dms ((Rad>>Deg) lat)
  and (lon_d, lon_m, lon_s) = dms ((Rad>>Deg) lon) in
  Printf.sprintf "%2d %02d' %02.1f\" %c\t%2d %02d' %02.1f\" %c"
    lat_d lat_m lat_s hemi lon_d lon_m lon_s east


let of_semicircle x =
  { posn_lat = (Semi>>Rad) x.lat ; posn_long = (Semi>>Rad) x.long }

let semicircle_of x =
  { lat = (Rad>>Semi) x.posn_lat ; long = (Rad>>Semi) x.posn_long }



type ellipsoid = { dx : float; dy : float; dz : float; a : float; df : float; e : float }
let ntf = { dx = -168.; dy = -60.; dz = 320. ; a = 6378249.2; df = 0.0034075495234250643; e = 0.08248325676}
let wgs84 = { dx = 0.; dy = 0.; dz = 0. ; a = 6378137.0; df = 0.0033528106647474805 ; e = 0.08181919106}
let ed50 = { dx = -87.0; dy = -98.0; dz = -121.0 ; a = 6378388.0; df = 0.003367003367003367 ; e = 0.08199188998}
let nad27 = { dx = 0.0; dy = 125.0; dz = 194.0 ; a = wgs84.a-. -.69.4; df = wgs84.df-. -0.37264639 /. 1e4 ; e = 0.08181919106(*** ??? ***)}

type geodesic = NTF | ED50 | WGS84 | NAD27
type ntf = geographic
let ellipsoid_of = function
    NTF -> ntf | ED50 -> ed50 | WGS84 -> wgs84 | NAD27 -> nad27


let latitude_isometrique phi e =
  log (tan (pi/.4. +. phi /. 2.0)) -. e /. 2.0 *. log ((1.0 +. e *. sin phi) /. (1.0 -. e *. sin phi))

let inverse_latitude_isometrique lat e epsilon =
  let exp_l = exp lat in
  let pi_2 = pi /. 2. in
  let phi0 = 2. *. atan exp_l -. pi_2 in
  let rec loop phi = 
    let sin_phi = e *. sin phi in
    let phi' = 2. *. atan (((1. +. sin_phi) /. (1. -. sin_phi))**(e/.2.) *. exp_l) -. pi_2 in
    if abs_float (phi' -. phi) < epsilon then phi' else loop phi' in
  loop phi0;;

type lambert_zone = {
    ellipsoid : ellipsoid;
    phi0 : radian;
    lphi0 : float;
    r0 : float;
    lambda0 : radian;
    y0 : int;
    x0 : int;
    ys : int;
    k0 : float (* facteur d'échelle *)
  }

type meter = int
type fmeter = float
type lambert = { lbt_x : meter; lbt_y : meter }
type utm = { utm_x : fmeter; utm_y : fmeter ; utm_zone : int }

module Ellipse = struct
  let e_square d = 2.0 *. d -. d ** 2.0;;
  let e_prime_square d = 1.0 /. (1.0 -. d) ** 2.0 -. 1.0;;
end

(* From http://www.tandt.be/wis/WiS/eqntf.html et http://www.ign.fr/MP/GEOD/geodesie/coordonnees.html *)
let lambertI = {
  ellipsoid = ntf;
  lambda0 = (Deg>>Rad) (decimal 2 20 14.025);
  phi0 = (Deg>>Rad) (decimal 49 30 0.);
  x0 = 600000;
  y0 = 200000;
  ys = 5657617;
  lphi0 = 0.991996665;
  r0 = 5457616.674;
  k0 = 0.99987734
};;

let lambertII = {
  ellipsoid = ntf;
  lambda0 = (Deg>>Rad) (decimal 2 20 14.025);
  phi0 = (Deg>>Rad) (decimal 46 48 0.);
  x0 = 600000;
  y0 = 2200000;
  ys = 6199696;
  lphi0 = 0.921557361;
  r0 = 5999695.77;
  k0 = 0.99987742};;

let lambertIIe = { lambertII with ys = 8199696 };;

let lambertIII = {
  ellipsoid = ntf;
  lambda0 = (Deg>>Rad) (decimal 2 20 14.025);
  phi0 = (Deg>>Rad) (decimal 44 6 0.);
  x0 = 600000;
  y0 = 3200000;
  ys = 6791905;
  lphi0 = 0.854591098;
  r0 = 6591905.08;
  k0 = 0.99987750};;

let lambertIV = {
  ellipsoid = ntf;
  lambda0 = (Deg>>Rad) (decimal 2 20 14.025);
  phi0 = (Deg>>Rad) (decimal 42 09 54.);
  x0 = 234;
  y0 = 4185861;
  ys = 7239162;
  lphi0 = 0.808475773;
  r0 = 7053300.18;
  k0 = 0.99994471
};;

let lambert_n l = sin l.phi0


let lambert_c l =
  let n = lambert_n l in
  l.r0 *. exp (l.lphi0 *. n)
  

let of_lambert l { lbt_x = x; lbt_y = y } =
  let c = lambert_c l and n = lambert_n l in
  let dx = float (x - l.x0) and dy = float (y - l.ys) in
  let r = sqrt (dx**2. +. dy**2.) in
  let gamma = atan2 dx (-. dy) in
  let lambda = l.lambda0 +. gamma /. n
  and ll = -. 1. /. n *. log (abs_float (r/.c)) in
  let phi = inverse_latitude_isometrique ll l.ellipsoid.e 1e-11 in
  make_geo phi lambda


let lambert_of l ({posn_long = lambda; posn_lat = phi} as pos) =
  if not (valid_geo pos) then
    invalid_arg "Latlong.lambert_of";
  let n = lambert_n l in
  let e = l.ellipsoid.e in
  let ll = latitude_isometrique phi e in
  let r = lambert_c l *. exp (-. ll *. n) in
  let gamma = (lambda -. l.lambda0) *. n in

  let x = l.x0 + truncate (r *. sin gamma) 
  and y = l.ys - truncate (r *. cos gamma) in
  { lbt_x = x; lbt_y = y };;


let serie5 cc e = 
  let ee = Array.init (Array.length cc.(0)) (fun i -> e ** (float (2*i))) in
  Array.init (Array.length cc)
    (fun i ->
      let cci = cc.(i) in
      let x = ref 0. in
      for j = 0 to Array.length cci - 1 do
	x := !x +. cci.(j) *. ee.(j)
      done;
      !x);;

let coeff_proj_mercator =
  [|[|1.; -. 1./.4.; -. 3./.64.; -.5./.256.; -.175./.16384.|];
    [|0.;1./.8.; -.1./.96.; -.9./.1024.; -.901./.184320.|];
    [|0.;0.;13./.768.;17./.5120.;-.311./.737280.|];
    [|0.;0.;0.; 61./.15360.;899./.430080.|];
    [|0.;0.;0.;0.;49561./.41287680.|]|];;

let coeff_proj_mercator_inverse =
  [|coeff_proj_mercator.(0);
    [|0.;1./.8.; 1./.48.; 7./.2048.; 1./.61440.|];
    [|0.;0.;1./.768.;3./.1280.;559./.368640.|];
    [|0.;0.;0.; 17./.30720.;283./.430080.|];
    [|0.;0.;0.;0.;4397./.41287680.|]|];;

let utm_of' = fun geo ->
  let ellipsoid =  ellipsoid_of geo in
  let k0 = 0.9996
  and xs = 500000. in
  let e = ellipsoid.e
  and n = k0 *. ellipsoid.a in
  let c = serie5 coeff_proj_mercator e in
  
  fun ({posn_long = lambda; posn_lat = phi} as pos) ->
    if not (valid_geo pos) then
      invalid_arg "Latlong.utm_of";
    let lambda_deg = truncate (floor ((Rad>>Deg)lambda)) in
    let zone = (lambda_deg + 180) / 6 + 1 in
    let lambda_c = (Deg>>Rad) (float (lambda_deg - ((lambda_deg mod 6)+6)mod 6 + 3)) in
    let ll = latitude_isometrique phi e
    and dl = lambda -. lambda_c in
    let phi' = asin (sin dl /. cosh ll) in
    let ll' = latitude_isometrique phi' 0. in
    let lambda' = atan (sinh ll /. cos dl) in
    let z = C.make lambda' ll' in
    let z' = ref (C.scal c.(0) z) in
    for k = 1 to Array.length c - 1 do
      z' := C.add !z' (C.scal c.(k) (C.sin (C.scal (float (2*k)) z)))
    done;
    z' := C.scal n !z';
    { utm_zone = zone; utm_x = xs +. C.im !z'; utm_y = C.re !z' };;
(* Benchmarks with limited bound on the for loop:
  utm_of WGS84 {posn_lat = 0.8; posn_long = 0.1 };;
 4: {utm_x = 711976.494998118491; utm_y = 5079519.01467700768; utm_zone = 31}
 3: {utm_x = 711976.494993993081; utm_y = 5079519.01467550546; utm_zone = 31}
 2: {utm_x = 711976.49488538783; utm_y = 5079519.02243153844; utm_zone = 31}
 1: {utm_x = 711977.141232272843; utm_y = 5079519.25322676543; utm_zone = 31}
 0: {utm_x = 711985.538644456305; utm_y = 5074176.83014749549; utm_zone = 31}

==> centimetric precision with 2 (i.e. using c(0), c(1) and c(2)
*)


(** Static evaluation for better performance (~50% for cputime) *)
let utm_of =
  let u_WGS84 = utm_of' WGS84
  and u_NTF = utm_of' NTF
  and u_ED50 = utm_of' ED50
  and u_NAD27 = utm_of' NAD27 in
  fun geo -> match geo with
    WGS84 -> u_WGS84
  | NTF -> u_NTF
  | ED50 -> u_ED50
  | NAD27 -> u_NAD27

let of_utm' geo  =
  let ellipsoid =  ellipsoid_of geo in
  let k0 = 0.9996
  and xs = 500000.
  and ys = 0. in
  let e = ellipsoid.e 
  and n = k0 *. ellipsoid.a in
  let c = serie5 coeff_proj_mercator_inverse e in

  fun { utm_zone = f; utm_x = x; utm_y = y } ->
    if x < 0. || x > 1e7 || y < -1e7 || y > 1e7 || f < 0 || f > 60 then
      invalid_arg "Latlong.of_utm";
    
    let lambda_c = (Deg>>Rad) (float (6 * f - 183)) in
    let z' = C.scal (1./.n/.c.(0)) (C.make (y-.ys) (x-.xs)) in
    let z = ref z' in
    for k = 1 to Array.length c - 1 do
      z := C.sub !z (C.scal c.(k) (C.sin (C.scal (float (2*k)) z')))
    done;
    let ll = C.re !z and lls = C.im !z in
    let lambda = lambda_c +. atan (sinh lls /. cos ll)
    and phi' = asin (sin ll /. cosh lls) in
    let ll = latitude_isometrique phi' 0. in
    let phi = inverse_latitude_isometrique ll e 1e-11 in
    make_geo phi lambda


(** Static evaluation for better performance (~50% for cputime) *)
let of_utm =
  let u_WGS84 = of_utm' WGS84
  and u_NTF = of_utm' NTF
  and u_ED50 = of_utm' ED50
  and u_NAD27 = of_utm' NAD27 in
  fun geo -> match geo with
    WGS84 -> u_WGS84
  | NTF -> u_NTF
  | ED50 -> u_ED50
  | NAD27 -> u_NAD27


let (<<) geo1 geo2 ({posn_long = lambda; posn_lat = phi} as pos) =
  if not (valid_geo pos) then
    invalid_arg "Latlong.(<<)";
  let elps1 = ellipsoid_of geo1
  and elps2 = ellipsoid_of geo2 in
  let d12 = sin phi
  and d13 = cos phi 
  and d14 = sin lambda
  and d15 = cos lambda in

  let d16 = Ellipse.e_square elps2.df
  and d17 = Ellipse.e_square elps1.df in
  let d18 = elps2.a /. sqrt (1.0 -. d16 *. d12 ** 2.0) in
  let d20 = d18 *. d13 *. d15 in
  let d21 = d18 *. d13 *. d14 in
  let d22 = d18 *. (1.0 -. d16) *. d12 in
  let d23 = d20 -. elps1.dx +. elps2.dx in
  let d24 = d21 -. elps1.dy +. elps2.dy in
  let d25 = d22 -. elps1.dz +. elps2.dz in
  let d26 = sqrt (d23 ** 2.0 +. d24 ** 2.0) in
  let d27 = atan2 d25 (d26 *. (1.0 -. elps1.df)) in
  let d28 = elps1.a *. (1.0 -. elps1.df) in
  let d29 = Ellipse.e_prime_square elps1.df in
  let d3 = atan2 (d25 +. d29 *. d28 *. (sin d27) ** 3.0) (d26 -. d17 *. elps1.a *. (cos d27) ** 3.0) in
  let d4 = atan2 d24 d23 in
  make_geo d3 d4

let cartesian_of ellips ({posn_long = lambda; posn_lat = phi} as pos) h =
  if not (valid_geo pos) || h < 0. then
    invalid_arg "Latlong.(<<)";
  let geo = ellipsoid_of ellips in
  let w = sqrt (1. -. geo.e**2. *. sin phi ** 2.)in
  let n = geo.a /. w in
  let x = (n+.h) *. cos phi *. cos lambda
  and y = (n+.h) *. cos phi *. sin lambda
  and z = (n*.(1.-.geo.e**2.)+.h) *. sin phi in
  { x = x; y = y; z = z}

let of_cartesian ellips {x=x;y=y;z=z} =
  let geo = ellipsoid_of ellips in
  let epsilon = 1e-11 in
  let xy = sqrt (x**2. +. y**2.)
  and r = sqrt (x**2. +. y**2. +. z**2.)
  and e2 = geo.e**2. in
  let z_xy = z /. xy in
  let lambda = 2. *. atan (y /. (x +. xy))
  and phi0 = atan (z_xy /. sqrt (1.-.geo.a*.e2/.r)) in
  let rec iter phi =
    let phi' = atan (z_xy /. (1.-.geo.a*.e2*.cos phi/.xy/.sqrt (1.-.e2*. sin phi ** 2.))) in
    if abs_float (phi -. phi') > epsilon then iter phi' else phi' in
  let phi = iter phi0 in
  let h = xy/.cos phi -. geo.a /. sqrt (1.-.e2*.sin phi ** 2.) in
  (make_geo phi lambda, h)

let utm_distance = fun utm1 utm2 ->
  if utm1.utm_zone <> utm2.utm_zone then invalid_arg "utm_distance";
  sqrt ((utm1.utm_x -. utm2.utm_x)**2. +. (utm1.utm_y -. utm2.utm_y)**2.)

let utm_add = fun u (x, y) ->
  {utm_x = u.utm_x +. x; utm_y = u.utm_y +. y; utm_zone = u.utm_zone }
  
let utm_sub = fun u1 u2 ->
  if u1.utm_zone <> u2.utm_zone then
    invalid_arg (Printf.sprintf "utm_sub: %d %d" u1.utm_zone u2.utm_zone);
  (u1.utm_x -. u2.utm_x, u1.utm_y -. u2.utm_y)

  
let of_lambertIIe = fun lbt ->
  (WGS84<<NTF)(of_lambert lambertIIe lbt)
let lambertIIe_of = fun wgs84 ->
  lambert_of lambertIIe ((NTF<<WGS84)wgs84)

let lbt_add = fun {lbt_x=x; lbt_y=y} (dx, dy) ->
  {lbt_x = x + truncate dx; lbt_y = y + truncate dy }
let lbt_sub = fun {lbt_x=x1; lbt_y=y1} {lbt_x=x2; lbt_y=y2} ->
  (float (x1-x2), float (y1-y2))

let space = Str.regexp "[ \t\'\"]+"
let fos = float_of_string
let ios = fun x -> try int_of_string x with _ -> failwith (Printf.sprintf "int_of_string: %s" x)
let rodg = fun s -> (Deg>>Rad)(fos s)
let of_string = fun s ->
  match Str.split space s with
    ["WGS84"; lat; long] ->
      make_geo (rodg lat) (rodg long)
  | ["WGS84_dms"; lat_d; lat_m; lat_s; hemi; lon_d; lon_m; lon_s; east_west] ->
      let sign_hemi =
	match hemi with
	  "N" -> 1. | "S" -> -1.
	| _ -> failwith (Printf.sprintf "N or S expected for hemispere in dms, found '%s'" hemi) in
      let sign_east =
	match east_west with
	  "E" -> 1. | "W" -> -1.
	| _ -> failwith (Printf.sprintf "E or W expected for hemispere in dms, found '%s'" east_west) in
      let lat = sign_hemi *. decimal (ios lat_d) (ios lat_m) (fos lat_s)
      and lon = sign_east *. decimal (ios lon_d) (ios lon_m) (fos lon_s) in
      make_geo ((Deg>>Rad) lat) ((Deg>>Rad) lon)
  | ["WGS84_bearing"; lat; long; dir; dist] ->
      let utm_ref = utm_of WGS84 (make_geo (rodg lat) (rodg long)) in
      let dir = rodg dir and dist = fos dist in
      let dx = dist *. sin dir
      and dy = dist *. cos dir in
      of_utm WGS84 (utm_add utm_ref (dx, dy))
  | ["UTM";x;y;zone] ->
      of_utm WGS84 { utm_x = fos x; utm_y = fos y; utm_zone = ios zone}
  | ["LBT2e";x;y] ->
      of_lambertIIe {lbt_x=ios x; lbt_y=ios y }
  | _ -> invalid_arg (Printf.sprintf "Latlong.of_string: %s" s)
let string_of = fun geo ->
  Printf.sprintf "WGS84 %s" (string_degrees_of_geographic geo)

let deg_of_string = fun s ->
  match Str.split space s with
    [lat_d; lat_m; lat_s] ->
      decimal (ios lat_d) (ios lat_m) (fos lat_s)
  | [lat_d; lat_m; lat_s; hemi] ->
      let sign =
	match hemi with
	  "N" | "E" -> 1. | "S" | "W" -> -1.
	| _ -> failwith (Printf.sprintf "N or S expected for hemispere in dms, found '%s'" hemi) in
      sign *. decimal (ios lat_d) (ios lat_m) (fos lat_s)
  | [deg] -> float_of_string deg
  | _ -> invalid_arg (Printf.sprintf "Latlong.of_string: %s" s)


let mercator_lat = fun l -> log (tan (pi/.4. +. 0.5*. l))
let inv_mercator_lat = fun l -> 2. *. atan (exp l) -. pi/.2.


let bearing = fun geo1 geo2 ->
  let utm1 = utm_of WGS84 geo1
  and utm2 = utm_of WGS84 geo2 in
  let (dx, dy) = utm_sub utm2 utm1 in
  ((Rad>>Deg)(atan2 dx dy), sqrt(dx*.dx+.dy*.dy))


let leap_seconds = 14 (* http://www.leapsecond.com/java/gpsclock.htm *)

  
let gps_tow_of_utc = fun ?wday hour min sec ->
  let wday =
    match wday with
      Some w -> w
    | None -> (Unix.gmtime (Unix.gettimeofday ())).Unix.tm_wday in
  ((wday*24 + hour)*60+min)*60+sec + leap_seconds

let get_gps_tow = fun () ->
  let utc = Unix.gmtime (Unix.gettimeofday ()) in
  gps_tow_of_utc ~wday:utc.Unix.tm_wday utc.Unix.tm_hour utc.Unix.tm_min utc.Unix.tm_sec

let unix_time_of_tow = fun tow ->
  let host_tow = get_gps_tow () in
  Unix.gettimeofday () +. float (tow - host_tow)


type coordinates_kind = 
    WGS84_dec
  | WGS84_dms
  | Bearing of geographic


let string_of_coordinates = fun kind geo ->
  match kind with
    WGS84_dec ->
      string_degrees_of_geographic geo
  | WGS84_dms ->
      string_dms_of_geographic geo
  | Bearing georef ->
      let (dx, dy) = utm_sub (utm_of WGS84 geo) (utm_of WGS84 georef) in
      let d = sqrt (dx*.dx+.dy*.dy) in
      let bearing = (int_of_float ((Rad>>Deg)(atan2 dx dy)) + 360) mod 360 in
      Printf.sprintf "%4d %4.0f" bearing d

let geographic_of_coordinates = fun kind s ->
  match kind with
    WGS84_dec -> 
      of_string ("WGS84 " ^ s)
  | WGS84_dms -> 
      of_string ("WGS84_dms " ^ s)
  | Bearing georef ->
      of_string (Printf.sprintf "WGS84_bearing %s %s" (string_degrees_of_geographic georef) s)

