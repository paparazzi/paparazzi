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

open Latlong

type ac_cam = {
  mutable phi : float; (* Rad, right = >0 *)
  mutable theta : float; (* Rad, front = >0 *)
  mutable target : (float * float) (* meter*meter relative *)
}

type rc_status = string (** OK, LOST, REALLY_LOST *)
type rc_mode = string (** MANUAL, AUTO, FAILSAFE *)
type fbw = {
  mutable fbw_bat : float;
  mutable rc_status : rc_status;
  mutable rc_mode : rc_mode;
  mutable rc_rate : int;
  mutable pprz_mode_msgs_since_last_fbw_status_msg : int;
}

let gps_nb_channels = 16
type svinfo = {
  svid : int;
  flags : int;
  qi : int;
  cno : int;
  elev : int;
  azim : int;
  mutable age : int
}

let svinfo_init = fun () ->
  {
    svid = 0 ;
    flags = 0;
    qi = 0;
    cno = 0;
    elev = 0;
    azim = 0;
    age = 0
  }

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

let datalink_status_init = fun () ->
  {
    uplink_lost_time = 9999;
    uplink_msgs = 0;
    downlink_msgs = 0;
    downlink_rate = 0;
  }
let link_status_init = fun () ->
  {
    rx_lost_time = 9999;
    rx_bytes = 0;
    rx_msgs = 0;
    rx_bytes_rate = 0.;
    tx_msgs = 0;
    ping_time = 9999.
  }

type inflight_calib = {
  mutable if_mode : int; (* DOWN|OFF|UP *)
  mutable if_val1 : float;
  mutable if_val2 : float
}

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

let add_pos_to_nav_ref = fun nav_ref  ?(z = 0.) (x, y) ->
  let rec lat_of_xy = fun lat last geo (_x, _y) n e ->
    if n > 0 && abs_float (lat -. last) > e then
      lat_of_xy (asin ( (_y +. (sin geo.posn_lat)*.(cos lat)*.(cos (asin (_x /. cos lat)))) /. cos geo.posn_lat)) lat geo (_x, _y) (n-1) e
    else
      lat
  in
  match nav_ref with
      Geo geo ->
        let m_to_rad = 0.0005399568034557235 *. 0.00029088820866572159 in
        let lat = lat_of_xy (geo.posn_lat +. asin (y*.m_to_rad)) 0. geo (x*.m_to_rad, y *.m_to_rad) 10 1.e-7 in
        Latlong.make_geo lat (geo.posn_long +. asin (x*.m_to_rad /. cos lat))
    | Utm utm ->
      Latlong.of_utm Latlong.WGS84 (Latlong.utm_add utm (x, y))
    | Ltp ecef ->
      let ned = Latlong.make_ned [| y; x; 0. |] in (* FIXME z=0 *)
      let (geo, _) = Latlong.geo_of_ecef Latlong.WGS84 (Latlong.ecef_of_ned ecef ned) in
      geo

type waypoint = { altitude : float; wp_geo : Latlong.geographic }

type aircraft = {
  mutable vehicle_type : vehicle_type;
  id : string;
  name : string;
  flight_plan : Xml.xml;
  airframe : Xml.xml;
  mutable pos : Latlong.geographic;
  mutable unix_time : float;
  mutable itow : int64; (* ms *)
  mutable roll    : float;
  mutable pitch   : float;
  mutable heading  : float; (* rad, CW 0=N *)
  mutable gspeed  : float; (* m/s *)
  mutable airspeed : float; (* m/s *)
  mutable course : float; (* rad *)
  mutable alt     : float;
  mutable agl     : float;
  mutable climb   : float;
  mutable nav_ref : nav_ref option;
  mutable d_hmsl : float;
  mutable ground_alt : float; (* ground alt ref if no SRTM data *)
  mutable desired_pos    : Latlong.geographic;
  mutable desired_altitude    : float;
  mutable desired_course : float;
  mutable desired_climb : float;
  mutable cur_block : int;
  mutable cur_stage : int;
  mutable throttle : float;
  mutable kill_mode : bool;
  mutable throttle_accu : float;
  mutable rpm  : float;
  mutable temp : float;
  mutable bat  : float;
  mutable amp : float;
  mutable energy  : int;
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

let max_nb_dl_setting_values = 256 (** indexed iwth an uint8 (messages.xml)  *)

let new_aircraft = fun id name fp airframe ->
  let svsinfo_init = Array.init gps_nb_channels (fun _ -> svinfo_init ()) in
  { vehicle_type = UnknownVehicleType; id = id; name = name; flight_plan = fp; airframe = airframe;
    pos = { Latlong.posn_lat = 0.; posn_long = 0. };
    unix_time = 0.; itow = Int64.of_int 0;
    roll = 0.; pitch = 0.;
    gspeed=0.; airspeed= -1.; course = 0.; heading = 0.; alt=0.; climb=0.; agl = 0.;
    nav_ref = None; d_hmsl = 0.; ground_alt = 0.;
    desired_pos = { Latlong.posn_lat = 0.; posn_long = 0. };
    desired_course = 0.; desired_altitude = 0.; desired_climb = 0.;
    cur_block=0; cur_stage=0;
    flight_time = 0; stage_time = 0; block_time = 0;
    throttle = 0.; throttle_accu = 0.; rpm = 0.; temp = 0.; bat = 0.; amp = 0.; energy = 0; ap_mode= -1;
    kill_mode = false;
    gaz_mode= -1; lateral_mode= -1;
    gps_mode = 0; gps_Pacc = 0; periodic_callbacks = [];
    state_filter_mode = 0;
    cam = { phi = 0.; theta = 0. ; target=(0.,0.)};
    fbw = { rc_status = "???"; rc_mode = "???"; rc_rate=0; fbw_bat=0.; pprz_mode_msgs_since_last_fbw_status_msg=0 };
    svinfo = svsinfo_init;
    dl_setting_values = Array.make max_nb_dl_setting_values None;
    nb_dl_setting_values = 0;
    horiz_mode = UnknownHorizMode;
    horizontal_mode = 0;
    waypoints = Hashtbl.create 3; survey = None; last_msg_date = 0.; dist_to_wp = 0.;
    datalink_status = datalink_status_init (); link_status = Hashtbl.create 1;
    time_since_last_survey_msg = 1729.;
    inflight_calib = { if_mode = 1 ; if_val1 = 0.; if_val2 = 0.}
  }
