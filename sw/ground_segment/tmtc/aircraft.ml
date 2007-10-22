(*
 * $Id$
 *
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

type ac_cam = {
    mutable phi : float; (* Rad, right = >0 *)
    mutable theta : float; (* Rad, front = >0 *)
    mutable target : (float * float) (* meter*meter relative *)
  }

type inflight_calib = {
    mutable if_mode : int; (* DOWN|OFF|UP *)
    mutable if_val1 : float;
    mutable if_val2 : float
  }

type contrast_status = string (** DEFAULT|WAITING|SET *)
type infrared = {
    mutable gps_hybrid_mode : int;
    mutable gps_hybrid_factor : float;
    mutable contrast_status : contrast_status;
    mutable contrast_value : int
  }
let new_infrared =  fun () ->
  { gps_hybrid_mode = 0; gps_hybrid_factor = 0. ;
    contrast_status = "DEFAULT"; contrast_value = 0}

type rc_status = string (** OK, LOST, REALLY_LOST *)
type rc_mode = string (** MANUAL, AUTO, FAILSAFE *)
type fbw = {
    mutable rc_status : rc_status;
    mutable rc_mode : rc_mode;
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

type horiz_mode = 
    Circle of Latlong.utm * int
  | Segment of Latlong.utm * Latlong.utm
  | UnknownHorizMode

type waypoint = { altitude : float; wp_utm : Latlong.utm }

type aircraft = { 
    id : string;
    name : string;
    flight_plan : Xml.xml;
    mutable pos : Latlong.utm;
    mutable roll    : float;
    mutable pitch   : float;
    mutable nav_ref    : Latlong.utm option;
    mutable desired_east    : float;
    mutable desired_north    : float;
    mutable desired_altitude    : float;
    mutable desired_course : float;
    mutable desired_climb : float;
    mutable gspeed  : float; (* m/s *)
    mutable course  : float; (* rad *)
    mutable alt     : float;
    mutable agl     : float;
    mutable climb   : float;
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
    inflight_calib : inflight_calib;
    infrared : infrared;
    fbw : fbw;
    svinfo : svinfo array;
    waypoints : (int, waypoint) Hashtbl.t;
    mutable flight_time : int;
    mutable stage_time : int;
    mutable block_time : int;
    mutable horiz_mode : horiz_mode;
    dl_setting_values : float array;
    mutable nb_dl_setting_values : int;
    mutable survey : (Latlong.geographic * Latlong.geographic) option;
    mutable last_bat_msg_date : float;
    mutable time_since_last_survey_msg : float;
    mutable dist_to_wp : float
  }

let max_nb_dl_setting_values = 256 (** indexed iwth an uint8 (messages.xml)  *)

let new_aircraft = fun id name fp ->
  let svsinfo_init = Array.init gps_nb_channels (fun _ -> svinfo_init ()) in
  { id = id ; name = name; roll = 0.; pitch = 0.; desired_east = 0.; desired_north = 0.; flight_plan = fp; dist_to_wp = 0.;
    desired_course = 0.;
    gspeed=0.; course = 0.; alt=0.; climb=0.; cur_block=0; cur_stage=0;
    throttle = 0.; throttle_accu = 0.; rpm = 0.; temp = 0.; bat = 42.; amp = 0.; energy = 0; ap_mode= -1; agl = 0.;
    gaz_mode= -1; lateral_mode= -1;
    gps_mode =0; gps_Pacc = 0; periodic_callbacks = [];
    desired_altitude = 0.;
    desired_climb = 0.;
    pos = { Latlong.utm_x = 0.; utm_y = 0.; utm_zone = 0 };
    nav_ref = None;
    cam = { phi = 0.; theta = 0. ; target=(0.,0.)};
    inflight_calib = { if_mode = 1 ; if_val1 = 0.; if_val2 = 0.};
    infrared = new_infrared (); kill_mode = false;
    fbw = { rc_status = "???"; rc_mode = "???" };
    svinfo = svsinfo_init;
    dl_setting_values = Array.create max_nb_dl_setting_values 42.;
    nb_dl_setting_values = 0;
    flight_time = 0; stage_time = 0; block_time = 0;
    horiz_mode = UnknownHorizMode;
    horizontal_mode = 0;
    waypoints = Hashtbl.create 3; survey = None; last_bat_msg_date = 0.;
    time_since_last_survey_msg = 1729.
  }
