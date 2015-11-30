(*
 * Geographic conversion utilities
 *
 * Copyright (C) 2004 CENA/ENAC, Pascal Brisset
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

(** Representing and converting geographic positions *)

(** {b Angles} *)

val pi : float

type degree = float
and radian = float
and semi = float
and dms = int * int * float
(** Angle units. In [semi], -pi is coded by -2^31 and pi by 2^31 (used in
Garmin GPS protocol). *)

val deg_string_of_rad  : radian -> string

type angle_unit = Semi | Rad | Deg | Grd
val ( >> ) : angle_unit -> angle_unit -> float -> float
(** [(Unit1>>Unit2) a] converts angle [a] expressed in [Unit1] in [Unit2] *)

val decimal : int -> int -> float -> float
val dms : float -> dms
(** Conversions between decimal degrees and degree, minutes and seconds *)

(** {b Geodesic datum} *)

type geodesic = NTF | ED50 | WGS84 | NAD27
(** Geodesic referential *)

type lambert_zone
val lambertI : lambert_zone
val lambertII : lambert_zone
val lambertIIe : lambert_zone
val lambertIII : lambert_zone
val lambertIV : lambert_zone
val lambert93 : lambert_zone
(** French lambert zones *)


(** {b Positions} *)

type semicircle = { lat : semi; long : semi; }
type geographic = { posn_lat : radian; posn_long : radian; }
type meter = int
type fmeter = float
type lambert = { lbt_x : meter; lbt_y : meter; }
type utm = { utm_x : fmeter; utm_y : fmeter; utm_zone : int; }
(** Position units. *)

val norm_angle : float -> float
(** [norm_angle rad] Returns an angle -pi<= < pi *)

val valid_geo : geographic -> bool

val make_geo : radian -> radian -> geographic
(** [make_geo lat long] *)

val make_geo_deg : degree -> degree -> geographic
(** [make_geo_deg lat long] *)

val string_degrees_of_geographic : geographic -> string
val string_dms_of_geographic : geographic -> string
val fprint_utm : out_channel -> utm -> unit
(** Pretty printing *)


(** {b Conversions} *)

val ( << ) : geodesic -> geodesic -> geographic -> geographic
(** [(Geo1<<Geo2) g] converts [g] expressed in [Geo2] referential to [Geo1]
referential *)

val of_semicircle : semicircle -> geographic
val semicircle_of : geographic -> semicircle

val of_lambert : lambert_zone -> lambert -> geographic
val lambert_of : lambert_zone -> geographic -> lambert
(** Conversions between geographic (in NTF or GRS80) and lambert *)

val utm_of' : ?zone:int -> geodesic -> geographic -> utm
val utm_of : ?zone:int -> geodesic -> geographic -> utm
val of_utm : geodesic -> utm -> geographic
(** Conversions between geographic and UTM. UTM zone can be forced with a specific value (use with caution).
 *  May raise Invalid_argument. *)

val utm_distance : utm -> utm -> fmeter

val utm_add : utm -> (fmeter * fmeter) -> utm
(** [add_utm utm (east,north)] *)

val utm_sub : utm -> utm -> (fmeter * fmeter)
(** [utm_sub u1 u2] Raises Invalid_arg if [u1] and [u2] are not in the same
UTM zone *)

val of_lambertIIe : lambert -> geographic
val lambertIIe_of : geographic -> lambert
val lbt_add : lambert -> (fmeter * fmeter) -> lambert
(** [add_lbt utm (east,north)] *)

val lbt_sub : lambert -> lambert -> (fmeter * fmeter)
(** Returns (east,north) *)


val of_string : string -> geographic
(** [of_string pos] Parses [pos] as "WGS84 45.678 1.2345", "UTM 500123 4500300 31" or "LBT2e 544945 1755355" *)
val string_of : geographic -> string
(** Returns a "WGS84" annotated string *)
val deg_of_string : string -> float
(** Parse a deg min sec or a decimal angle *)

val mercator_lat : float -> float
(** wgs84 -> [-pi; pi] *)

val inv_mercator_lat : float -> float
(** [-pi; pi] -> wgs84 *)

val bearing : geographic -> geographic -> float * float
(** [bearing from to] returns (degrees CW/north, m) *)

val gps_tow_of_utc : ?wday:int -> int -> int -> int -> int
(** [gps_tow_of_utc ?wday hour min sec] Returns the GPS time of week
    in seconds, taking into acount the leap seconds. Default [wday] (week
    day) is the current day (Sunday is 0). *)

val get_gps_tow : unit -> int
(** Returns the current GPS time of week in seconds *)

val unix_time_of_tow : ?week:int -> int -> float
(** If week if not supplied, current one is assumed *)

type coordinates_kind =
    WGS84_dec
  | WGS84_dms
  | LBT2e
  | Bearing of < pos : geographic >

val string_of_coordinates : coordinates_kind -> geographic -> string
val geographic_of_coordinates : coordinates_kind -> string -> geographic


type ecef
type ned

val array_of_ecef : ecef -> float array
val make_ecef : float array -> ecef

val array_of_ned : ned -> float array
val make_ned : float array -> ned

val fprint_ecef : out_channel -> ecef -> unit
val fprint_ned : out_channel -> ned -> unit

val ecef_distance : ecef -> ecef -> fmeter

val ned_of_ecef : ecef -> ecef -> ned
(** [ned_of_ecef r p] Returns the coordinates of [p] in a local NED (north east down)
    located in [r] *)

val ecef_of_ned : ecef -> ned -> ecef
(** [ecef_of_ned r ned] *)

val ecef_of_geo : geodesic -> geographic -> float -> ecef
(** [ecef_of_geo ellipsoid geo h] *)

val geo_of_ecef : geodesic -> ecef -> geographic * float
(** [geo_of_ecef ellipsoid geo ecef] Returns llh *)

val wgs84_hmsl : geographic -> fmeter

val wgs84_distance : geographic -> geographic -> fmeter
(** Distance over the WGS84 ellipsoid *)
