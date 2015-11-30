(*
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

let (//) = Filename.concat

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

let piradian = function
Semi -> 2. ** 31. | Rad -> pi | Deg -> 180. | Grd -> 200.
let (>>) u1 u2 x = (x *. piradian u2) /. piradian u1;;

let make_geo_deg = fun lat long ->
  { posn_long = norm_angle ((Deg>>Rad)long); posn_lat = ((Deg>>Rad)lat) }


let deg_string_of_rad = fun r -> Printf.sprintf "%.7f" ((Rad>>Deg)r)

let decimal d m s = float d +. float m /. 60. +. s /. 3600.;;
let dms = fun x ->
  let d = truncate x in
  let m = truncate ((x -. float d) *. 60.) in
  let s = 3600. *. (x -. float d -. float m /. 60.) in
  (d, m, s);;


let sprint_degree_of_radian x =
  Printf.sprintf "%.7f" ((Rad>>Deg) x)

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
let grs80 = { dx = 0.; dy = 0.; dz = 0. ; a = 6378137.0; df = 0.00335281068118231879 ; e = 0.0818191910428151675}

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

(* http://professionnels.ign.fr/DISPLAY/000/526/700/5267002/transformation.pdf *)

type lambert_zone = {
  ellipsoid : ellipsoid;
  phi0 : radian;
  c : float;
  lambda0 : radian;
  y0 : int;
  x0 : int;
  ys : int;
  n : float
}

type meter = int
type fmeter = float
type lambert = { lbt_x : meter; lbt_y : meter }
type utm = { utm_x : fmeter; utm_y : fmeter ; utm_zone : int }

module Ellipse = struct
  let e_square d = 2.0 *. d -. d ** 2.0;;
  let e_prime_square d = 1.0 /. (1.0 -. d) ** 2.0 -. 1.0;;
end

(* From http://www.winnepenninckx.com/Geo/WiS/eqntf.html et http://www.ign.fr/DISPLAY/000/526/701/5267019/NTG_71.pdf *)
let lambertI =
  let phi0 = (Deg>>Rad) (decimal 49 30 0.) in
  { ellipsoid = ntf;
    lambda0 = (Deg>>Rad) (decimal 2 20 14.025);
    phi0 = phi0;
    x0 = 600000;
    y0 = 200000;
    ys = 5657617;
    c = 11603796.98;
    n = sin phi0 (* tangent projection *)
  };;

let lambertII =
  let phi0 = (Deg>>Rad) (decimal 46 48 0.) in
  { ellipsoid = ntf;
    lambda0 = (Deg>>Rad) (decimal 2 20 14.025);
    phi0 = phi0;
    x0 = 600000;
    y0 = 2200000;
    ys = 6199696;
    c = 11745793.39;
    n = sin phi0 (* tangent projection *)
  };;

let lambertIIe = { lambertII with ys = 8199696 };;

let lambertIII =
  let phi0 = (Deg>>Rad) (decimal 44 6 0.) in
  { ellipsoid = ntf;
    lambda0 = (Deg>>Rad) (decimal 2 20 14.025);
    phi0 = phi0;
    x0 = 600000;
    y0 = 3200000;
    ys = 6791905;
    c = 11947992.52;
    n = sin phi0 (* tangent projection *)
  };;

let lambertIV =
  let phi0 = (Deg>>Rad) (decimal 42 09 54.) in
  {
    ellipsoid = ntf;
    lambda0 = (Deg>>Rad) (decimal 2 20 14.025);
    phi0 = phi0;
    x0 = 234;
    y0 = 4185861;
    ys = 7239162;
    c = 12136281.99;
    n = sin phi0;    (* tangent projection *)
  };;

let lambert93 = {
  ellipsoid = grs80;
  lambda0 = (Deg>>Rad) (decimal 3 0 0.0);
  phi0 = (Deg>>Rad) (decimal 46 30 0.);
  x0 = 700000;
  y0 = 6600000;
  ys = 12655612;
  c = 11754255.426;
  n = 0.725607765053268738 (* Secant projection *)
};;

let lambert_n l = l.n

let lambert_c = fun l -> l.c


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

let utm_of' = fun ?zone geo ->
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
    let zone, lambda_c =
      match zone with
      | None ->
          (lambda_deg + 180) / 6 + 1,
          (Deg>>Rad) (float (lambda_deg - ((lambda_deg mod 6)+6)mod 6 + 3))
      | Some z -> z, (Deg>>Rad) (float ((z - 1)*6 - 180 + 3))
    in
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
let utm_of = fun ?zone ->
  let u_WGS84 = utm_of' ?zone WGS84
  and u_NTF = utm_of' ?zone NTF
  and u_ED50 = utm_of' ?zone ED50
  and u_NAD27 = utm_of' ?zone NAD27 in
  fun geo -> match geo with
      WGS84 -> u_WGS84
    | NTF -> u_NTF
    | ED50 -> u_ED50
    | NAD27 -> u_NAD27

let of_utm' geo  =
  let ellipsoid = ellipsoid_of geo in
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


(** Offset between GPS and UTC times in seconds.
 * Update when a new leap second is inserted and be careful about times in the
 * past when this offset was different.
 * Last leap second was inserted on June 30, 2012 at 23:59:60 UTC
 * http://www.leapsecond.com/java/gpsclock.htm
 *)
let leap_seconds = 16

(** leap seconds in GPS time.
 * There have been 16 leap seconds so far, with the last one at
 * June 30, 2012 at 23:59:60 UTC which equals 1025136015 in GPS seconds
 * http://www.leapsecond.com/java/gpsclock.htm
 * http://www.andrews.edu/~tzs/timeconv/timealgorithm.html
 *)
let leap_seconds_list = [46828800.; 78364801.; 109900802.; 173059203.; 252028804.; 315187205.; 346723206.; 393984007.; 425520008.; 457056009.; 504489610.; 551750411.; 599184012.; 820108813.; 914803214.; 1025136015.]

(** Count number of leap seconds when converting gps to unix time *)
let gps_count_leaps = fun gps_time ->
  let rec loop = fun l s ->
    match l with
    | [] -> s
    | x::xs -> if gps_time >= x then loop xs (s+1) else s
  in
  loop leap_seconds_list 0

(** Unix timestamp of the GPS epoch 1980-01-06 00:00:00 UTC *)
let gps_epoch = 315964800.


let gps_tow_of_utc = fun ?wday hour min sec ->
  let wday =
    match wday with
        Some w -> w
      | None -> (Unix.gmtime (Unix.gettimeofday ())).Unix.tm_wday in
  ((wday*24 + hour)*60+min)*60+sec + leap_seconds

let get_gps_tow = fun () ->
  let utc = Unix.gmtime (Unix.gettimeofday ()) in
  gps_tow_of_utc ~wday:utc.Unix.tm_wday utc.Unix.tm_hour utc.Unix.tm_min utc.Unix.tm_sec


let unix_time_of_tow = fun ?week tow ->
  match week with
      None ->
        let host_tow = get_gps_tow ()
        and unix_now = Unix.gettimeofday () in
        unix_now +. float (tow - host_tow)
    | Some w ->
      let gps_seconds = gps_epoch
        +. float w *. 60. *. 60. *. 24. *. 7.
        +. float tow in
      gps_seconds -. float (gps_count_leaps gps_seconds)



type coordinates_kind =
    WGS84_dec
  | WGS84_dms
  | LBT2e
  | Bearing of < pos : geographic>


      let string_of_coordinates = fun kind geo ->
        match kind with
            WGS84_dec ->
              string_degrees_of_geographic geo
          | WGS84_dms ->
            string_dms_of_geographic geo
          | LBT2e ->
            let l = lambertIIe_of geo in
            Printf.sprintf "%d %d" l.lbt_x l.lbt_y
          | Bearing georef ->
            try
              let (dx, dy) = utm_sub (utm_of WGS84 geo) (utm_of WGS84 georef#pos) in
              let d = sqrt (dx*.dx+.dy*.dy) in
              let bearing = (int_of_float ((Rad>>Deg)(atan2 dx dy)) + 360) mod 360 in
              Printf.sprintf "%4d %4.0f" bearing d
            with _ -> "Dist across diff utm zones unsupported"

let geographic_of_coordinates = fun kind s ->
  match kind with
      WGS84_dec ->
        of_string ("WGS84 " ^ s)
    | WGS84_dms ->
      of_string ("WGS84_dms " ^ s)
    | LBT2e ->
      of_string ("LBT2e " ^ s)
    | Bearing georef ->
      of_string (Printf.sprintf "WGS84_bearing %s %s" (string_degrees_of_geographic georef#pos) s)


let fprint_utm = fun c utm ->
  Printf.fprintf c "[%.2f %.2f %d]" utm.utm_x utm.utm_y utm.utm_zone

type ecef = float array
type ned = float array

let make_ecef = fun x -> x
let make_ned = fun x -> x
let array_of_ecef = fun x -> x
let array_of_ned = fun x -> x

let fprint_ecef = fun c x -> Printf.fprintf c "[%.2f %.2f %.2f]" x.(0) x.(1) x.(2)
let fprint_ned = fprint_ecef

let ecef_distance = fun e1 e2 ->
  sqrt ((e1.(0)-.e2.(0))**2. +. (e1.(1)-.e2.(1))**2. +. (e1.(2)-.e2.(2))**2.)

let ecef_of_geo = fun geo ->
  let elps = ellipsoid_of geo in
  let e2 = 2.*.elps.df -. elps.df*.elps.df in
  fun {posn_lat=lat; posn_long=long} h ->
    let sin_lat = sin lat
    and cos_lat = cos lat
    and cos_long = cos long in

    let chi = sqrt (1. -. e2*.sin_lat*.sin_lat) in
    let a_chi = elps.a /. chi in
    let x = (a_chi +.h)*.cos_lat*.cos_long
    and y = (a_chi +.h)*.cos_lat*.sin long
    and z = (a_chi*.(1.-.e2) +. h)*.sin_lat in
    [|x; y; z|]

let geo_of_ecef = fun geo ->
  let elps = ellipsoid_of geo in

  let e2 = 2.*.elps.df -. elps.df*.elps.df
  and ep2 = elps.df*.(2.-.elps.df)/.((1.-.elps.df)**2.)
  and b = elps.a*.(1.-.elps.df) in

  fun ecef ->
    let x = ecef.(0) and y = ecef.(1) and z = ecef.(2) in
    let z2 = z**2.
    and r2 = x**2. +. y**2. in
    let r = sqrt r2
    and _E2 = elps.a**2. -. b**2.
    and _F = 54.*.b**2.*.z2 in
    let _G = r2 +. (1.-.e2)*.z2 -. e2*._E2 in
    let _C = (e2*.e2*._F*.r2)/.(_G**3.) in
    let _S = ( 1. +. _C +. sqrt (_C**2. +. 2.*._C))**(1./.3.) in
    let _P = _F/.(3.*.(_S +. 1./._S +. 1.)**2. *. _G**2.) in
    let _Q = sqrt (1.+.2.*.e2*.e2*._P) in
    let r0 = -. (e2*._P*.r)/.(1.+._Q) +. sqrt ((elps.a**2./.2.)*.(1. +. 1./._Q) -. ((1.-.e2)*._P*.z2)/.(_Q*.(1.+._Q)) -. _P*.r2/.2.) in
    let tmp = (r -. e2*.r0)**2. in
    let _U = sqrt (tmp +. z2)
    and _V = sqrt (tmp +. (1.-.e2)*.z2) in
    let z0 = (b**2.*.z)/.(elps.a *. _V) in

    let h = _U*.(1. -. b**2./.(elps.a *. _V))
    and phi = atan ((z +. ep2*.z0)/.r )
    and lambda = atan2 y x in

    ({posn_lat = phi; posn_long = lambda}, h)

(* http://en.wikipedia.org/wiki/Geodetic_system
   Approximation using the geocentric latitude instead of the geodetic one *)
let ned_of_ecef = fun r ->
  let wgs84, _h = geo_of_ecef WGS84 r in
  let sin_lambda = sin wgs84.posn_long
  and cos_lambda = cos wgs84.posn_long
  and sin_phiP = sin wgs84.posn_lat
  and cos_phiP = cos wgs84.posn_lat in

  fun p ->
    let x_xr = p.(0) -. r.(0)
    and y_yr = p.(1) -. r.(1)
    and z_zr = p.(2) -. r.(2) in

    let e = -.sin_lambda*.x_xr +. cos_lambda*.y_yr
    and n = -.sin_phiP*.cos_lambda*.x_xr -. sin_phiP*.sin_lambda*.y_yr +. cos_phiP*.z_zr
    and u = cos_phiP*.cos_lambda*.x_xr +. cos_phiP*.sin_lambda*.y_yr +. sin_phiP*.z_zr in
    [|n; e; -.u|]

let ecef_of_ned = fun r ->
  let wgs84, _h = geo_of_ecef WGS84 r in
  let sin_lambda = sin wgs84.posn_long
  and cos_lambda = cos wgs84.posn_long
  and sin_phiP = sin wgs84.posn_lat
  and cos_phiP = cos wgs84.posn_lat in
  fun ned ->
    let n = ned.(0) and e = ned.(1) and u = -.ned.(2) in
    let x = -.sin_lambda*.e -. cos_lambda*.sin_phiP*.n +. cos_lambda*.cos_phiP*.u +. r.(0)
    and y =  cos_lambda*.e -. sin_lambda*.sin_phiP*.n +. cos_phiP*.sin_lambda*.u +. r.(1)
    and z = cos_phiP*.n +. sin_phiP*.u +. r.(2) in
    [|x; y; z|]


(** From gpsd geoid.c *)
let bilinear = fun x1 y1 x2 y2 x y z11 z12 z21 z22 ->
  match x1 = x2, y1 = y2 with (* Check for exact grid points *)
      true, true   -> z11
    | true, false  -> (z22*.(x-.x1)+.z11*.(x2-.x))/.(x2-.x1)
    | false, true  -> (z22*.(y-.y1)+.z11*.(y2-.y))/.(y2-.y1)
    | false, false ->
      let delta = (y2-.y1)*.(x2-.x1) in
      (z22*.(y-.y1)*.(x-.x1)+.z12*.(y2-.y)*.(x-.x1)+.z21*.(y-.y1)*.(x2-.x)+.z11*.(y2-.y)*.(x2-.x))/.delta




(** From gpsd geoid.c
    return geoid separtion (MSL-WGS84) in meters,given geographic coordinates*)
let geoid_data =
  [|    (* 90S *) [|-30;-30;-30;-30;-30;-30;-30;-30;-30;-30;-30;-30;-30;-30;-30;-30;-30;-30;-30;-30;-30;-30;-30;-30;-30;-30; -30;-30;-30;-30;-30;-30;-30;-30;-30;-30;-30|];
    (* 80S *) [|-53;-54;-55;-52;-48;-42;-38;-38;-29;-26;-26;-24;-23;-21;-19;-16;-12; -8; -4; -1;  1;  4;  4;  6;  5;  4;   2; -6;-15;-24;-33;-40;-48;-50;-53;-52;-53|];
    (* 70S *) [|-61;-60;-61;-55;-49;-44;-38;-31;-25;-16; -6;  1;  4;  5;  4;  2;  6; 12; 16; 16; 17; 21; 20; 26; 26; 22;  16; 10; -1;-16;-29;-36;-46;-55;-54;-59;-61|];
    (* 60S *) [|-45;-43;-37;-32;-30;-26;-23;-22;-16;-10; -2; 10; 20; 20; 21; 24; 22; 17; 16; 19; 25; 30; 35; 35; 33; 30;  27; 10; -2;-14;-23;-30;-33;-29;-35;-43;-45|];
    (* 50S *) [|-15;-18;-18;-16;-17;-15;-10;-10; -8; -2;  6; 14; 13;  3;  3; 10; 20; 27; 25; 26; 34; 39; 45; 45; 38; 39;  28; 13; -1;-15;-22;-22;-18;-15;-14;-10;-15|];
    (* 40S *) [| 21;  6;  1; -7;-12;-12;-12;-10; -7; -1;  8; 23; 15; -2; -6;  6; 21; 24; 18; 26; 31; 33; 39; 41; 30; 24;  13; -2;-20;-32;-33;-27;-14; -2;  5; 20; 21|];
    (* 30S *) [| 46; 22;  5; -2; -8;-13;-10; -7; -4;  1;  9; 32; 16;  4; -8;  4; 12; 15; 22; 27; 34; 29; 14; 15; 15;  7;  -9;-25;-37;-39;-23;-14; 15; 33; 34; 45; 46|];
    (* 20S *) [| 51; 27; 10;  0; -9;-11; -5; -2; -3; -1;  9; 35; 20; -5; -6; -5;  0; 13; 17; 23; 21;  8; -9;-10;-11;-20; -40;-47;-45;-25;  5; 23; 45; 58; 57; 63; 51|];
    (* 10S *) [| 36; 22; 11;  6; -1; -8;-10; -8;-11; -9;  1; 32;  4;-18;-13; -9;  4; 14; 12; 13; -2;-14;-25;-32;-38;-60; -75;-63;-26;  0; 35; 52; 68; 76; 64; 52; 36|];
    (* 00N *) [| 22; 16; 17; 13;  1;-12;-23;-20;-14; -3; 14; 10;-15;-27;-18;  3; 12; 20; 18; 12;-13; -9;-28;-49;-62;-89;-102;-63; -9; 33; 58; 73; 74; 63; 50; 32; 22|];
    (* 10N *) [| 13; 12; 11;  2;-11;-28;-38;-29;-10;  3;  1;-11;-41;-42;-16;  3; 17; 33; 22; 23;  2; -3; -7;-36;-59;-90; -95;-63;-24; 12; 53; 60; 58; 46; 36; 26; 13|];
    (* 20N *) [|  5; 10;  7; -7;-23;-39;-47;-34; -9;-10;-20;-45;-48;-32; -9; 17; 25; 31; 31; 26; 15;  6;  1;-29;-44;-61; -67;-59;-36;-11; 21; 39; 49; 39; 22; 10;  5|];
    (* 30N *) [| -7; -5; -8;-15;-28;-40;-42;-29;-22;-26;-32;-51;-40;-17; 17; 31; 34; 44; 36; 28; 29; 17; 12;-20;-15;-40; -33;-34;-34;-28;  7; 29; 43; 20;  4; -6; -7|];
    (* 40N *) [|-12;-10;-13;-20;-31;-34;-21;-16;-26;-34;-33;-35;-26;  2; 33; 59; 52; 51; 52; 48; 35; 40; 33; -9;-28;-39; -48;-59;-50;-28;  3; 23; 37; 18; -1;-11;-12|];
    (* 50N *) [| -8;  8;  8;  1;-11;-19;-16;-18;-22;-35;-40;-26;-12; 24; 45; 63; 62; 59; 47; 48; 42; 28; 12;-10;-19;-33; -43;-42;-43;-29; -2; 17; 23; 22;  6;  2; -8|];
    (* 60N *) [|  2;  9; 17; 10; 13;  1;-14;-30;-39;-46;-42;-21;  6; 29; 49; 65; 60; 57; 47; 41; 21; 18; 14;  7; -3;-22; -29;-32;-32;-26;-15; -2; 13; 17; 19;  6;  2|];
    (* 70N *) [|  2;  2;  1; -1; -3; -7;-14;-24;-27;-25;-19;  3; 24; 37; 47; 60; 61; 58; 51; 43; 29; 20; 12;  5; -2;-10; -14;-12;-10;-14;-12; -6; -2;  3;  6;  4;  2|];
    (* 80N *) [|  3;  1; -2; -3; -3; -3; -1;  3;  1;  5;  9; 11; 19; 27; 31; 34; 33; 34; 33; 34; 28; 23; 17; 13;  9;  4;   4;  1; -2; -2;  0;  2;  3;  2;  1;  1;  3|];
    (* 90N *) [| 13; 13; 13; 13; 13; 13; 13; 13; 13; 13; 13; 13; 13; 13; 13; 13; 13; 13; 13; 13; 13; 13; 13; 13; 13; 13;  13; 13; 13; 13; 13; 13; 13; 13; 13; 13; 13|]|]


(* Online geoid calculator :
   http://earth-info.nga.mil/GandG/wgs84/gravitymod/egm96/intpt.html

   lat lon  EGM96   this function
   41. 1.   49.83   51.15
   35 -140  -32.53  -29.5
   56 6     41.82   45.08
   77 13    37.71   34.18
   -35 144  8.15    6.5
*)
let wgs84_hmsl = fun geo ->
  let n_rows = Array.length geoid_data
  and n_cols = Array.length geoid_data.(0) in

  let lat = truncate ((Rad>>Deg) (norm_angle geo.posn_lat))
  and lon = truncate ((Rad>>Deg) (norm_angle geo.posn_long)) in

  let ilat = (lat + 90) / 10
  and ilon = (lon + 180) / 10 in

  assert(ilat < n_rows-1);
  assert(ilon < n_cols-1);

  let ilat1 = ilat and ilon1 = ilon
  and ilat2 = ilat+1 and ilon2 = ilon+1 in

  bilinear
    (float (ilon1*10-180)) (float (ilat1*10-90))
    (float (ilon2*10-180)) (float (ilat2*10-90))
    (float lon)                    (float lat)
    (float geoid_data.(ilat1).(ilon1))
    (float geoid_data.(ilat1).(ilon2))
    (float geoid_data.(ilat2).(ilon1))
    (float geoid_data.(ilat2).(ilon2))

let wgs84_distance = fun geo1 geo2 ->
  let e1 = ecef_of_geo WGS84 geo1 0.
  and e2 = ecef_of_geo WGS84 geo1 0. in
  ecef_distance e1 e2

