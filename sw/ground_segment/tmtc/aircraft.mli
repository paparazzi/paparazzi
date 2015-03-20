(*
 * Copyright (C) ENAC
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

(** State of an A/C handled by the server *)

type ac_cam = {
    mutable phi : float; (* Rad, right = >0 *)
    mutable theta : float; (* Rad, front = >0 *)
    mutable target : (float * float) (* meter*meter relative *)
  }

type inflight_calib = {
    mutable if_mode : int;
    mutable if_val1 : float;
    mutable if_val2 : float;
  }

type rc_status = string
type rc_mode = string
type fbw = { mutable rc_status : rc_status; mutable rc_mode : rc_mode; mutable rc_rate : int; mutable pprz_mode_msgs_since_last_fbw_status_msg : int; }
val gps_nb_channels : int
type svinfo = {
    svid : int;
    flags : int;
    qi : int;
    cno : int;
    elev : int;
    azim : int;
    mutable age : int
  }
val svinfo_init : unit -> svinfo

type datalink_status = {
    mutable uplink_lost_time : int;
    mutable uplink_msgs : int;
    mutable downlink_msgs : int;
    mutable downlink_rate : int;
 }
type link_status = {
    rx_lost_time : int;
    rx_bytes : int;
    rx_msgs : int;
    rx_bytes_rate : float;
    tx_msgs : int;
    ping_time : float
 }
val datalink_status_init : unit -> datalink_status
val link_status_init : unit -> link_status

type horiz_mode =
    Circle of Latlong.geographic * int
  | Segment of Latlong.geographic * Latlong.geographic
  | UnknownHorizMode

type nav_ref =
    Geo of Latlong.geographic
  | Utm of Latlong.utm
  | Ltp of Latlong.ecef

type vehicle_type =
    FixedWing
  | Rotorcraft
  | UnknownVehicleType

val add_pos_to_nav_ref : nav_ref -> ?z:float -> (float * float) -> Latlong.geographic

type waypoint = { altitude : float; wp_geo : Latlong.geographic }

type aircraft = {
    mutable vehicle_type : vehicle_type;
    id : string;
    name : string;
    flight_plan : Xml.xml;
    airframe : Xml.xml;
    mutable pos : Latlong.geographic;
    mutable unix_time : float;
    mutable itow : int64;
    mutable roll : float;
    mutable pitch : float;
    mutable heading : float; (* rad *)
    mutable gspeed : float; (* m/s *)
    mutable airspeed : float; (* m/s *)
    mutable course : float; (* rad *)
    mutable alt : float;
    mutable agl : float; (* m *)
    mutable climb : float;
    mutable nav_ref : nav_ref option;
    mutable d_hmsl : float; (* difference between geoid and ellipsoid *)
    mutable ground_alt : float; (* ground alt ref if no SRTM data *)
    mutable desired_pos : Latlong.geographic;
    mutable desired_altitude : float;
    mutable desired_course : float;
    mutable desired_climb : float;
    mutable cur_block : int;
    mutable cur_stage : int;
    mutable throttle : float;
    mutable kill_mode : bool;
    mutable throttle_accu : float;
    mutable rpm : float;
    mutable temp : float;
    mutable bat : float;
    mutable amp : float;
    mutable energy : int;
    mutable ap_mode : int;
    mutable gaz_mode : int;
    mutable lateral_mode : int;
    mutable horizontal_mode : int;
    mutable periodic_callbacks : Glib.Timeout.id list;
    cam : ac_cam;
    mutable gps_mode : int;
    mutable gps_Pacc : int;
    mutable state_filter_mode : int;
    fbw : fbw;
    svinfo : svinfo array;
    waypoints : (int, waypoint) Hashtbl.t;
    mutable flight_time : int;
    mutable stage_time : int;
    mutable block_time : int;
    mutable horiz_mode : horiz_mode;
    dl_setting_values : float option array;
    mutable nb_dl_setting_values : int;
    mutable survey : (Latlong.geographic * Latlong.geographic) option;
    datalink_status : datalink_status;
    link_status : (int, link_status) Hashtbl.t;
    mutable last_msg_date : float;
    mutable time_since_last_survey_msg : float;
    mutable dist_to_wp : float;
    inflight_calib : inflight_calib
}

val new_aircraft : string -> string -> Xml.xml -> Xml.xml -> aircraft
val max_nb_dl_setting_values : int
