(*
* $Id$
*
* Real time handling of flying A/Cs
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


type color = string
type aircraft = {
    ac_name : string;
    config : Pprz.values;
    track : MapTrack.track;
    color: color;
    fp_group : MapFP.flight_plan;
    fp : Xml.xml;
    blocks : (int * string) list;
    mutable last_ap_mode : string;
    mutable last_stage : int * int;
    ir_page : Pages.infrared;
    gps_page : Pages.gps;
    pfd_page : Horizon.pfd;
    misc_page : Pages.misc;
    dl_settings_page : Pages.settings option;
    rc_settings_page : Pages.rc_settings option;
    pages : GObj.widget;
    notebook_label : GMisc.label;
    strip : Strip.t;
    mutable first_pos : bool;
    mutable last_block_name : string;
    mutable in_kill_mode : bool;
    mutable speed : float;
    mutable alt : float;
    mutable target_alt : float;
    mutable flight_time : int;
    mutable wind_speed : float;
    mutable wind_dir : float; (* Rad *)
    mutable ground_prox : bool;
  }

val aircrafts : (string, aircraft) Hashtbl.t


val safe_bind : string -> (string -> Pprz.values -> unit) -> unit

val track_size : int ref
(** Default length for A/C tracks on the 2D view *)

val listen_acs_and_msgs : MapCanvas.widget -> GPack.notebook -> Pages.alert -> bool -> unit
(** [listen_acs_and_msgs geomap aircraft_notebook alert_page auto_center_new_ac] *)
