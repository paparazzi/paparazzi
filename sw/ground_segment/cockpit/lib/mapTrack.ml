(*
 * Track objects
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

open Printf
module G2d = Geometry_2d
module LL = Latlong

module G = MapCanvas

module CL = ContrastLabel
module ACI = AcIcon

let affine_pos_and_angle z xw yw angle =
  let rad_angle = angle /. 180. *. acos(-1.) in
  let cos_a = cos rad_angle in
  let sin_a = sin rad_angle in
  [| cos_a /. z ; sin_a /. z ; ~-. sin_a /. z; cos_a /. z; xw ; yw |]

let rec norm_angle_360 = fun alpha ->
  if alpha > 360.0 then norm_angle_360 (alpha -. 360.0)
  else if alpha < 0.0 then norm_angle_360 (alpha +. 360.0)
  else alpha


(** variables used for handling cam moves: *)

let cam_half_aperture = LL.pi /. 6.0
let half_pi = LL.pi /. 2.0

type desired =
    NoDesired
  | DesiredCircle of LL.geographic*float*GnoCanvas.ellipse
  | DesiredSegment of LL.geographic*LL.geographic*GnoCanvas.line

class track = fun ?(name="Noname") ?(icon="fixedwing") ?(size = 500) ?(color="red") ?(show_carrot=true) (ac_id:string) (geomap:MapCanvas.widget) ->
  let group = GnoCanvas.group geomap#canvas#root in
  let empty = ({LL.posn_lat=0.; LL.posn_long=0.},  GnoCanvas.line group) in
  let v_empty = ({LL.posn_lat=0.; LL.posn_long=0.},  0.0) in

  let aircraft = GnoCanvas.group group
  and track = GnoCanvas.group group in
  let icon_template = match icon with
  | "home"          -> ACI.icon_home_template
  | "rotorcraft"    -> ACI.icon_rotorcraft_template
  | "quadrotor"     -> ACI.icon_quadrotor_template
  | "hexarotor"     -> ACI.icon_hexarotor_template
  | "octorotor"     -> ACI.icon_octorotor_template
  | "quadrotor_x"   -> ACI.icon_quadrotor_x_template
  | "hexarotor_x"   -> ACI.icon_hexarotor_x_template
  | "octorotor_x"   -> ACI.icon_octorotor_x_template
  | "quadrotor_xi"  -> ACI.icon_quadrotor_xi_template
  | "flyingwing"    -> ACI.icon_flyingwing_template
  | "intruder"      -> ACI.icon_intruder_template
  | "fixedwing" | _ -> ACI.icon_fixedwing_template
  in
  let _ac_icon = new ACI.widget ~color ~icon_template aircraft in
  let ac_label = new CL.widget ~name ~color 25. 25. group in

  let carrot = GnoCanvas.group group in
  let _ac_carrot =
    if show_carrot then
      ignore (GnoCanvas.polygon ~points:[|0.;0.;-5.;-10.;5.;-10.|] ~props:[`WIDTH_UNITS 1.;`FILL_COLOR "orange"; `OUTLINE_COLOR "orange"; `FILL_STIPPLE (Gdk.Bitmap.create_from_data ~width:2 ~height:2 "\002\001")] carrot)
    else ()
  in

  let cam = GnoCanvas.group group in

  (** rectangle representing the field covered by the cam *)
  let _ac_cam_targeted =
    ignore ( GnoCanvas.ellipse ~x1: (-. 2.5) ~y1: (-. 2.5 ) ~x2: 2.5 ~y2: 2.5 ~fill_color:color ~props:[`WIDTH_UNITS 1.; `OUTLINE_COLOR color; `FILL_STIPPLE (Gdk.Bitmap.create_from_data ~width:2 ~height:2 "\002\001")] cam) in
  let _ = cam#hide () in

  let mission_target = GnoCanvas.group group in

  (** red circle : target of the mission *)
  let _ac_mission_target =
    ignore ( GnoCanvas.ellipse ~x1: (-5.) ~y1: (-5.) ~x2: 5. ~y2: 5. ~fill_color:"red" ~props:[`WIDTH_UNITS 1.; `OUTLINE_COLOR "red"; `FILL_STIPPLE (Gdk.Bitmap.create_from_data ~width:2 ~height:2 "\002\001")] mission_target) in
  let _ = mission_target#hide () in

  let _desired_circle = GnoCanvas.ellipse group
  and _desired_segment = GnoCanvas.line group in

  let _ = aircraft#raise_to_top () in

object (self)
  val mutable top = 0
  val mutable color = color
  val mutable segments = Array.make size empty
  val mutable v_segments = Array.make size empty
  val mutable v_top = 0
  val mutable v_path = Array.make 10 v_empty
  val mutable last = None
  val mutable last_heading = 0.0
  val mutable last_altitude = 0.0
  val mutable last_speed = 0.0
  val mutable last_climb = 0.0
  val mutable last_flight_time = 0.0
  val mutable last_x_val = 0.0
  val mutable cam_on = false
  val mutable params_on = false
  val mutable v_params_on = false
  val mutable desired_track = NoDesired
  val zone = GnoCanvas.rect group
  val mutable ac_cam_cover = GnoCanvas.polygon ~fill_color:"grey" ~props:[`WIDTH_PIXELS 1 ; `FILL_STIPPLE (Gdk.Bitmap.create_from_data ~width:2 ~height:2 "\002\001")] cam
  val mutable event_cb = None
  val mutable destroyed = false
  method color = color
  method set_color c = color <- c
  method track = track
  method v_path = v_path
  method aircraft = aircraft
  method id = ac_id
  method name = name
  method set_label = fun s ->
          ac_label#set_name s
  method clear_one = fun i ->
    if segments.(i) != empty then begin
      (snd segments.(i))#destroy ();
      segments.(i) <- empty
    end
  method incr = fun seg ->
    let s = Array.length seg in
    top <- (top + 1) mod s
  method v_incr = fun path ->
    let s = Array.length path in
    v_top <- (v_top + 1) mod s
  method clear = fun () ->
    for i = 0 to Array.length segments - 1 do
      self#clear_one i
    done;
    top <- 0
  method set_cam_state = fun b ->
    cam_on <- b;
    if b then begin
      cam#show ();
      mission_target#show ()
    end else begin
      cam#hide ();
      mission_target#hide ()
    end

  method update_ap_status = fun time ->
    last_flight_time <- time
  method set_params_state = fun b ->
    params_on <- b;
    if not b then (* Reset to the default simple label *)
      ac_label#set_name name;
      ac_label#set_y 25.
  method set_v_params_state = fun b -> v_params_on <- b
  method set_last = fun x -> last <- x
  method last = last
  method pos = match last with Some pos -> pos | None -> failwith "No pos"
  method last_heading = last_heading
  method last_altitude = last_altitude
  method last_speed = last_speed
  method last_climb = last_climb

  method height = fun () ->
    match last with
        None -> last_altitude
      | Some wgs84 ->
        let h = try float (Srtm.of_wgs84 wgs84) with _ -> 0. in
        last_altitude -. h

    (** add track points on map2D, according to the
        track parameter and store altitude for the vertical path *)
  method add_point = fun geo alt ->
    self#clear_one top;
    let last_geo =
      match last with
          None -> geo
        | Some last_geo -> last_geo in
    segments.(top) <- (geo, geomap#segment ~group ~fill_color:color last_geo geo);
    self#incr segments;
    self#set_last (Some geo);
    v_path.(v_top) <- (geo, alt);
    self#v_incr v_path

  method clear_map2D = self#clear ()

  method move_icon = fun wgs84 heading altitude speed climb ->
    let (xw,yw) = geomap#world_of wgs84 in
    aircraft#affine_absolute (affine_pos_and_angle geomap#zoom_adj#value xw yw heading);
    last_heading <- heading;
    last_altitude <- altitude;
    last_speed <- speed ;
    last_climb <- climb;

    if params_on then begin
      let last_height = self#height () in
      ac_label#set_name (sprintf "%s\n%+.0f m\n%.1f m/s" name last_height last_speed);
      ac_label#set_y 70.
    end;

    ac_label#affine_absolute (affine_pos_and_angle geomap#zoom_adj#value xw yw 0.);
    self#add_point wgs84 altitude;

  method move_carrot = fun wgs84 ->
    let (xw,yw) = geomap#world_of wgs84 in
    carrot#affine_absolute (affine_pos_and_angle geomap#zoom_adj#value xw yw 0.);

    (** draws the circular path to be followed by the aircraft in circle mode *)
  method draw_circle = fun en radius ->
    let create = fun () ->
      desired_track <- DesiredCircle (en, radius, geomap#circle ~color:"#00ff00" en radius) in
    match desired_track with
        DesiredCircle (c, r, circle) ->
          if c <> en || r <> radius then begin
            circle#destroy ();
            create ()
          end
      | DesiredSegment (p1,p2,s) ->
        s#destroy ();
        create ()
      | NoDesired ->
        create ()

    (** draws the linear path to be followed by the aircraft between two waypoints *)
  method draw_segment = fun en1 en2 ->
    let create = fun () ->
      desired_track <- DesiredSegment (en1, en2, geomap#segment ~fill_color:"#00ff00" en1 en2) in
    match desired_track with
        DesiredCircle (c, r, circle) ->
          circle#destroy ();
          create ()
      | DesiredSegment (p1,p2,s) ->
        if p1 <> en1 || p2 <> en2 then begin
          s#destroy ();
          create ()
        end
      | NoDesired ->
        create ()

  method delete_desired_track = fun () ->
    begin
      match desired_track with
          DesiredCircle (c, r, circle) ->
            circle#destroy ()
        | DesiredSegment (p1,p2,s) ->
          s#destroy ();
        | NoDesired ->
          ()
    end;
    desired_track <- NoDesired

  method draw_zone = fun geo1 geo2 ->
    let (x1, y1) = geomap#world_of geo1
    and (x2, y2) = geomap#world_of geo2 in
    zone#set [`X1 x1; `Y1 y1; `X2 x2; `Y2 y2; `OUTLINE_COLOR "#ffc0c0"; `WIDTH_PIXELS 2]

    (** moves the rectangle representing the field covered by the camera *)
  method move_cam = fun positions mission_target_wgs84 ->
    match last, cam_on with
        Some last_ac, true ->
          let points = geomap#convert_positions_to_points positions in
          ac_cam_cover#set [`POINTS points;
                            `OUTLINE_COLOR color];
          let (mission_target_xw, mission_target_yw) = geomap#world_of mission_target_wgs84 in
          mission_target#affine_absolute (affine_pos_and_angle geomap#zoom_adj#value mission_target_xw mission_target_yw 0.0)
      | _ -> ()
  method zoom = fun z ->
    let a = aircraft#i2w_affine in
    let z' = sqrt (a.(0)*.a.(0)+.a.(1)*.a.(1)) in
    for i = 0 to 3 do a.(i) <- a.(i) /. z' *. 1./.z done;
    aircraft#affine_absolute a

  method resize =  fun new_size ->
    let a = Array.make new_size empty in
    let size =  Array.length segments in
    let m = min new_size size in
    let j = ref ((top - m + size) mod size) in
    for i = 0 to m - 1 do
      a.(i) <- segments.(!j);
      j := (!j + 1) mod size
    done;
    for i = 1 to size - new_size do (* Never done if new_size > size *)
      self#clear_one !j;
      j := (!j + 1) mod size
    done;
    top <- m mod new_size;
    segments <- a

  method size = Array.length segments

  method event (ev : GnoCanvas.item_event) =
    begin
      match ev with
        | `BUTTON_PRESS ev ->
          begin
            match GdkEvent.Button.button ev with
              | 1 ->
                  begin
                    match event_cb with
                    | Some cb -> cb ac_id
                    | None -> ()
                  end
              | _ -> ()
          end
        | _ -> ()
    end;
    true
  initializer ignore(aircraft#connect#event self#event)

  method set_event_cb = fun (cb: string -> unit) -> event_cb <- Some cb

  initializer
    (* could not properly disconnect adjustment signal, so only calling zoom method if group is still displayed *)
    ignore(geomap#zoom_adj#connect#value_changed (fun () -> if not destroyed then self#zoom geomap#zoom_adj#value));
    ignore(group#connect#destroy (fun () -> destroyed <- true))

  (* destroy method *)
  method destroy = fun () -> group#destroy ()

  initializer
    Gc.finalise (fun self -> self#destroy ()) self
end
