(*
 * $Id$
 *
 * Geographic conversion utilities
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

type degree = float
type radian = float
type semi = float
type dms = int * int * float

type semicircle = { lat : semi; long : semi }
type geographic = { posn_lat : radian ; posn_long : radian }

type angle_unit = Semi | Rad | Deg | Grd

type cartesian = {x : float; y : float; z: float }

let pi = 3.14159265358979323846;;

let piradian = function
  Semi -> 2. ** 31. | Rad -> pi | Deg -> 180. | Grd -> 200.
let (>>) u1 u2 x = (x *. piradian u2) /. piradian u1;;

let deg_string_of_rad = fun r -> Printf.sprintf "%.6f" ((Rad>>Deg)r)

let sprint_degree_of_radian x =
  Printf.sprintf "%.6f" ((Rad>>Deg) x)

let string_degrees_of_geographic sm =
  Printf.sprintf "%s\t%s"
    (sprint_degree_of_radian sm.posn_lat) (sprint_degree_of_radian sm.posn_long)


let of_semicircle x =
  { posn_lat = (Semi>>Rad) x.lat ; posn_long = (Semi>>Rad) x.long }

let semicircle_of x =
  { lat = (Rad>>Semi) x.posn_lat ; long = (Rad>>Semi) x.posn_long }

let decimal d m s = float d +. float m /. 60. +. s /. 3600.;;
let dms x =
  let d = truncate x in
  let m = truncate ((x -. float d) *. 60.) in
  let s = 3600. *. (x -. float d -. float m /. 60.) in
  (d, m, s);;



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
  
let lambert = function
    1 -> lambertI |  2 -> lambertII |  3 -> lambertIII |  4 -> lambertIV | _ -> failwith "lambert";;


let of_lambert l { lbt_x = x; lbt_y = y } =
  let c = lambert_c l and n = lambert_n l in
  let dx = float (x - l.x0) and dy = float (y - l.ys) in
  let r = sqrt (dx**2. +. dy**2.) in
  let gamma = atan2 dx (-. dy) in
  let lambda = l.lambda0 +. gamma /. n
  and ll = -. 1. /. n *. log (abs_float (r/.c)) in
  let phi = inverse_latitude_isometrique ll l.ellipsoid.e 1e-11 in
  {posn_long = lambda; posn_lat = phi};;


let lambert_of l {posn_long = lambda; posn_lat = phi} =
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

let utm_of geo {posn_long = lambda; posn_lat = phi} =
  let ellipsoid =  ellipsoid_of geo in
  let k0 = 0.9996
  and xs = 500000.
  and ys = if phi > 0. then 0. else 10000000. in
  let lambda_deg = truncate (floor ((Rad>>Deg)lambda)) in
  let zone = (lambda_deg + 180) / 6 + 1 in
  let lambda_c = (Deg>>Rad) (float (lambda_deg - ((lambda_deg mod 6)+6)mod 6 + 3)) in
  let e = ellipsoid.e
  and n = k0 *. ellipsoid.a in
  let ll = latitude_isometrique phi e
  and dl = lambda -. lambda_c in
  let phi' = asin (sin dl /. cosh ll) in
  let ll' = latitude_isometrique phi' 0. in
  let lambda' = atan (sinh ll /. cos dl) in
  let z = C.make lambda' ll'
  and c = serie5 coeff_proj_mercator e in
  let z' = ref (C.scal c.(0) z) in
  for k = 1 to Array.length c - 1 do
    z' := C.add !z' (C.scal c.(k) (C.sin (C.scal (float (2*k)) z)))
  done;
  z' := C.scal n !z';
  { utm_zone = zone; utm_x = xs +. C.im !z'; utm_y = ys +. C.re !z' };;

let of_utm geo { utm_zone = f; utm_x = x; utm_y = y } =
  let ellipsoid =  ellipsoid_of geo in
  let k0 = 0.9996
  and xs = 500000.
  and ys = 0. in
  let e = ellipsoid.e 
  and n = k0 *. ellipsoid.a in
  let c = serie5 coeff_proj_mercator_inverse e in
  
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
  {posn_long = lambda; posn_lat = phi};;


let (<<) geo1 geo2 {posn_long = lambda; posn_lat = phi} =
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
  {posn_long = d4; posn_lat = d3};;

let cartesian_of ellips {posn_long = lambda; posn_lat = phi} h =
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
  ({posn_long = lambda; posn_lat = phi}, h)

let utm_distance = fun utm1 utm2 ->
  if utm1.utm_zone <> utm2.utm_zone then invalid_arg "utm_distance";
  sqrt ((utm1.utm_x -. utm2.utm_x)**2. +. (utm1.utm_y -. utm2.utm_y)**2.)

let utm_add = fun u x y ->
  {utm_x = u.utm_x +. x; utm_y = u.utm_y +. y; utm_zone = u.utm_zone }
  
let wgs84_of_lambertIIe = fun x y -> (WGS84<<NTF)(of_lambert lambertIIe {lbt_x = x; lbt_y = y})

let space = Str.regexp "[ \t]+"
let fos = float_of_string
let ios = int_of_string
let rodg = fun s -> (Deg>>Rad)(fos s)
let of_string = fun s ->
  match Str.split space s with
    ["WGS84"; lat ; long] ->
      {posn_lat = rodg lat; posn_long = rodg long}
  | ["UTM";x;y;zone] ->
      of_utm WGS84 { utm_x = fos x; utm_y = fos y; utm_zone = ios zone}
  | ["LBT2e";x;y] ->
      wgs84_of_lambertIIe (ios x) (ios y)
  | _ -> invalid_arg (Printf.sprintf "Latlong.of_string: %s" s)



let (/.=) r x = r := !r /. x
let (+.=) r x = r := !r +. x
let (-.=) r x = r := !r -. x

(** Returns a keyhole string for a longitude (x), latitude (y), and zoom 
   for Google Maps (http://www.ponies.me.uk/maps/GoogleTileUtils.java) *)
let gm_tile_string = fun wgs84 zoom ->
  let zoom = 18 - zoom in
  
  (* first convert the lat lon to transverse mercator coordintes.*)
  let lon = (Rad>>Deg)wgs84.posn_long in
  let lon = if lon > 180. then lon -. 180. else lon in
  let lon = lon /. 180. in

  (*  convert latitude to a range -1..+1 *)
  let lat = log (tan (pi/.4. +. 0.5*. wgs84.posn_lat)) /. pi in

  let tLat = ref (-1.)
  and tLon      = ref (-1.)
  and lonWidth  = ref 2.
  and latHeight = ref 2. (** Always identical to lonWidth !!! *)
  and keyholeString = Buffer.create 3 in
  Buffer.add_char keyholeString 't';

  for i = 0 to zoom - 1 do
    lonWidth /.= 2.;
    latHeight /.= 2.;

    if !tLat +. !latHeight > lat then
      if ((!tLon +. !lonWidth) > lon) then begin
        Buffer.add_char keyholeString 't';
      end else begin
        tLon +.= !lonWidth;
        Buffer.add_char keyholeString 's';
      end
    else begin
      tLat +.= !latHeight;

      if ((!tLon +. !lonWidth) > lon) then begin
        Buffer.add_char keyholeString 'q';
      end
      else begin  
        tLon +.= !lonWidth;
        Buffer.add_char keyholeString 'r';
      end
    end
  done;
  let tmp_lat = fun l -> 2. *. atan (exp (l *. pi)) -. pi/.2. in
  let bl_lat = tmp_lat !tLat in
  let tr_lat = tmp_lat (!tLat +. !latHeight) in
  let bottom_left = {posn_lat = bl_lat ; posn_long = !tLon *. pi} in
  let top_right = {posn_lat = tr_lat;
		   posn_long = bottom_left.posn_long +. !lonWidth *. pi } in

  let utm_bottom_left = utm_of WGS84 bottom_left
  and utm_top_right = utm_of WGS84 top_right in
  Printf.fprintf stderr "%fx%f\n" (utm_bottom_left.utm_x -. utm_top_right.utm_x) (utm_bottom_left.utm_y -. utm_top_right.utm_y);
  let scale = utm_distance utm_bottom_left utm_top_right  /. sqrt (2. *. 256.*.256.) in
  (Buffer.contents keyholeString, bottom_left, scale)


let gm_lat_long_of_tile = fun keyholeStr ->
  assert(keyholeStr.[0] = 't');
  
  let lon  = ref (-180.)
  and lonWidth = ref 360. 
  and  lat       = ref (-1.) 
  and latHeight = ref 2. in
  
  for i = 1 to String.length keyholeStr - 1  do
    lonWidth /.= 2.;
    latHeight /.= 2.;

    match keyholeStr.[i] with
      's' -> lon +.= !lonWidth
    | 'r' -> 
        lat +.= !latHeight;
        lon +.= !lonWidth
    | 'q' -> lat +.= !latHeight
    | 't' -> ()
    | _ -> invalid_arg ("gm_get_lat_long " ^ keyholeStr)
  done;

  latHeight +.= !lat;
  latHeight := (2. *. atan (exp (pi *. !latHeight))) -. (pi /. 2.);

  lat := (2. *. atan (exp (pi *. !lat))) -. (pi /. 2.);

  latHeight -.= !lat;
  lon := (Deg>>Rad)!lon;
  { posn_lat = !lat; posn_long = !lon }
