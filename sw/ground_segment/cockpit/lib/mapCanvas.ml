(*
 * Geographic display
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

module LL = Latlong
open LL
module G2D = Geometry_2d
open Printf
let (//) = Filename.concat

type world = float * float

let zoom_factor = 1.5 (* Mouse wheel zoom action *)
let max_zoom = 40.
let pan_step = 50 (* Pan keys speed *)
let pan_arrow_size = 40.

let grid_color = "#29d3f8"
let size_utm_grid = 10 (* half the horiz/vert size in km *)

let align = fun x a ->
  float (truncate (x /. float a) * a)

type meter = float

let distance = fun (x1,y1) (x2,y2) -> sqrt ((x1-.x2)**2.+.(y1-.y2)**2.)

let affine_pos_and_angle ?(z = 1.) xw yw angle =
  let cos_a = cos angle in
  let sin_a = sin angle in
  [| cos_a /. z; sin_a /. z; ~-. sin_a /. z; cos_a /. z; xw ; yw |]

type projection =
    Mercator (* 1e-6 = 1 world unit, y axis reversed *)
  | UTM (* 1m = 1 world unit, y axis reversed *)
  | LambertIIe (* 1m = 1 world unit, y axis reversed *)

let string_of_projection = function
UTM -> "UTM"
  | Mercator -> "Mercator"
  | LambertIIe -> "LBT2e"

let mercator_coeff = 5e6

let my_check_menu_item = fun label ~active ~callback ~packing () ->
  let mi = GMenu.check_menu_item ~label ~active ~packing () in
  ignore (mi#connect#toggled ~callback:(fun () -> callback mi#active));
  mi

let my_menu_item = fun label ~callback ~packing () ->
  let mi = GMenu.menu_item ~label ~packing () in
  ignore (mi#connect#activate ~callback)

let my_menu_item_insert = fun label ~menu ~pos ~callback  ->
  let mi = GMenu.menu_item ~label () in
  menu#insert mi ~pos ;
  ignore (mi#connect#activate ~callback)

let set_opacity = fun pixbuf opacity ->
  let pixbuf = GdkPixbuf.add_alpha pixbuf in

  let region = GdkPixbuf.get_pixels pixbuf
  and w = GdkPixbuf.get_width pixbuf
  and h = GdkPixbuf.get_height pixbuf in
  let n_channels = GdkPixbuf.get_n_channels pixbuf in
  assert(n_channels = 4);
  for i = 0 to h - 1 do
    for j = 0 to w - 1 do
      let pos = n_channels* (i*w + j) + 3 in
      Gpointer.set_byte region ~pos opacity
    done
  done;
  pixbuf


type drawing =
    NotDrawing
  | Rectangle of float*float
  | Panning of float*float


let float_array_of_points = fun l ->
  Array.of_list (List.fold_right (fun (x,y) r -> x::y::r) l [])

let pvect (x1,y1) (x2,y2) = x1*.y2-.y1*.x2

let rec convexify = fun l ->
  match l with
      [] | [_] | [_;_] -> l
    | (x1,y1)::(x2,y2)::(x3,y3)::l ->
      if pvect (x3-.x2, y3-.y2) (x1-.x2,y1-.y2) < 0.
      then convexify ((x1,y1)::(x3,y3)::l)
      else (x1,y1)::convexify ((x2,y2)::(x3,y3)::l)

let convex = fun l ->
  match convexify l with
      [] -> []
    | (x3,y3)::ps ->
      (** Remove last bad points *)
      let rec loop = fun l ->
        match l with
            [] | [_] -> l
          | (x2,y2)::(x1,y1)::pts ->
            if pvect (x3-.x2, y3-.y2) (x1-.x2,y1-.y2) < 0.
            then loop ((x1,y1)::pts)
            else l in
      (x3,y3)::List.rev (loop (List.rev ps))

(** Setting Opacity for bitmap *)
let stipple_opacity = fun opacity ->
  match opacity with
    | 0 -> (1,"\002\001")
    | 1 -> (3,"\002\001")
    | 2 -> (2,"\002\001")
    | 3 -> (1,"\003\001")
    | _ -> (1,"\002\001")

class type geographic = object
  method pos : Latlong.geographic
end


(** basic canvas with menubar ************************************************)

(* world_unit: m:pixel at scale 1. *)
class basic_widget = fun ?(height=800) ?width ?(projection = Mercator) ?georef () ->
  let frame = GPack.vbox ~height ?width () in

  let top_bar = GPack.hbox ~packing:frame#pack () in

  let menubar = GMenu.menu_bar ~packing:(top_bar#pack ~expand:true) () in
  let file_menu_item = GMenu.menu_item ~label:"Nav" ~packing:menubar#append () in
  let file_menu = GMenu.menu () in
  let _ = file_menu_item#set_submenu file_menu in
  let factory = new GMenu.factory menubar in

  let toolbar = GPack.hbox ~packing:top_bar#pack () in

  let info = GPack.hbox ~packing:top_bar#pack () in

  let spin_button = GEdit.spin_button  ~rate:0. ~digits:2 ~width:50 (*** ~height:20 ***) ~packing:top_bar#pack () in

  let canvas = GnoCanvas.canvas ~packing:(frame#pack ~expand:true) () in
  let background = GnoCanvas.group canvas#root
  and still = GnoCanvas.group canvas#root in
  (* set a black rectangle as background to catch mouse event even without maps loaded *)
  let _ = GnoCanvas.rect ~props:[`X1 (-25000000.); `Y1 (-25000000.); `X2 25000000.; `Y2 25000000.; `FILL_COLOR "black"] background in
  (* create several layers of canvas group to display the map in correct order *)
  let maps = Array.init (Gm.zoom_max - Gm.zoom_min + 1) (fun _ -> GnoCanvas.group background) in
  let view_cbs = Hashtbl.create 3 in (* Store for view event callback *)
  let region_rectangle = GnoCanvas.rect canvas#root ~props:[`WIDTH_PIXELS 2; `OUTLINE_COLOR "red"] in

  (* Pan arrows *)
  let s = pan_arrow_size in
  let s2 = s/.2. and s4=s/.4. in
  let points = [|0.;0.; s2;s2; s4;s2; s4;s; -.s4;s; -.s4;s2; -.s2;s2|] in
  let props = [`FILL_COLOR "#a0a0ff"; `FILL_STIPPLE (Gdk.Bitmap.create_from_data ~width:2 ~height:2 "\002\001")] in
  let arrow = fun x y angle ->
    let a = GnoCanvas.polygon still ~points ~props in
    a#affine_relative (affine_pos_and_angle x y angle);
    a in
  let north_arrow = arrow (1.5*.s) 0. 0.
  and south_arrow = arrow (1.5*.s) (3.*.s) LL.pi
  and west_arrow = arrow 0. (1.5*.s) (-.LL.pi/.2.)
  and east_arrow = arrow (3.*.s) (1.5*.s) (LL.pi/.2.) in

  (* Biroute *)
  let wind_sock = new Wind_sock.item 4. still in
  let _ = wind_sock#item#affine_relative (affine_pos_and_angle 60. 60. 0.) in
  (* Time *)
  let utc_time = GnoCanvas.text ~x:0. ~y:0. ~props:[`TEXT "00:00:00"; `FILL_COLOR "#00ff00"; `ANCHOR `NW] still in

object (self)

  (** GUI attributes *)

  val background = background
  val toolbar = toolbar
  method toolbar = toolbar
  method background = background
  method still = still
  method maps = maps
  method top_still = 3.5*.s
  method utc_time = utc_time
  method set_utc_time = fun h m s ->
    let string = sprintf "%02d:%02d:%02d" h m s in
    utc_time#set [`TEXT string]

  method wind_sock = wind_sock
  method set_wind_sock = fun angle_deg string  ->
    let angle_rad = (Deg>>Rad) (90. +. angle_deg) in
    wind_sock#item#affine_absolute (affine_pos_and_angle 60. 60. angle_rad);
    wind_sock#label#set [`TEXT string]

  val adj = GData.adjustment
    ~value:1. ~lower:0.005 ~upper:max_zoom
    ~step_incr:0.25 ~page_incr:1.0 ~page_size:0. ()

  method info = info

  (** other attributes *)

  val mutable projection = projection
  val mutable georef = georef
  val mutable dragging = None
  val mutable drawing  = NotDrawing
  val mutable region = None (* Rectangle selected region *)
  val mutable last_mouse_x = 0
  val mutable last_mouse_y = 0
  val mutable zoom_level = 1.;

  val mutable fitted_objects = ([] : geographic list)

  method region = region
  method register_to_fit = fun o -> fitted_objects <- o :: fitted_objects

  method fit_to_window () =
    let min_lat, max_lat, min_long, max_long =
      List.fold_right
        (fun p (min_lat, max_lat, min_long, max_long) ->
          let pos = p#pos in
          let lat = pos.LL.posn_lat
          and long = pos.LL.posn_long in
          (* Processing over positive longitudes *)
          let long = if long < 0. then long +. 2. *. pi else long in
          (min min_lat lat, max max_lat lat,
           min min_long long, max max_long long))
        fitted_objects
        (max_float, -.max_float, max_float, -. max_float) in

    (* Over 0° ? *)
    let min_long, max_long =
      if max_long -. min_long > pi
      then (max_long -. 2. *. pi, min_long)
      else (min_long, max_long) in

    (* Longitude is renormalized here *)
    let c = LL.make_geo ((min_lat+.max_lat)/.2.) ((min_long+.max_long)/.2.)
    and nw_xw, nw_yw = self#world_of (LL.make_geo max_lat min_long)
    and se_xw, se_yw = self#world_of (LL.make_geo min_lat max_long) in
    let width, height = Gdk.Drawable.get_size canvas#misc#window in
    let margin = 10 in
    let width = width - 2*margin and height = height - 2*margin in
    let zoom = min (float width/.(se_xw-.nw_xw)) (float height/.(se_yw-.nw_yw)) in
    self#zoom zoom;
    self#center c

  (** initialization of instance attributes *)

  initializer (

    spin_button#set_adjustment adj;
    spin_button#set_value zoom_level; (* this should be done by set_adjustment but seems to fail on ubuntu 13.10 (at least) *)

    utc_time#hide ();

    ignore (GMisc.separator ~packing:(toolbar#pack ~from:`END) `VERTICAL ());

    (** callback bindings *)

    canvas#coerce#misc#modify_bg [`NORMAL, `BLACK];
    ignore (background#connect#event ~callback:self#background_event);

    ignore (canvas#event#connect#motion_notify ~callback:self#mouse_motion);
    ignore (canvas#event#connect#after#key_press ~callback:self#key_press) ;
    ignore (canvas#event#connect#enter_notify ~callback:(fun _ -> self#canvas#misc#grab_focus () ; false));
    ignore (canvas#event#connect#any ~callback:self#any_event);
    ignore (adj#connect#value_changed ~callback:(fun () -> if abs_float (adj#value -. zoom_level) >= 0.01 then self#zoom_in_center adj#value));

    canvas#set_center_scroll_region false ;
    canvas#set_scroll_region ~x1:(-25000000.) ~y1:(-25000000.) ~x2:25000000. ~y2:25000000.;

  )



  (** methods *)

  (** accessors to instance variables *)
  method current_zoom = zoom_level
  method canvas = canvas
  method frame = frame
  method factory = factory
  method menubar = menubar
  method file_menu = file_menu
  method window_to_world = canvas#window_to_world
  method root = canvas#root
  method zoom_adj = adj

  (** following display functions can be redefined by subclasses.
      they do nothing in the basic_widget *)
  method display_geo = fun _s -> ()
  method display_alt = fun _wgs84 -> ()
  method display_group = fun _s -> ()

  method georef = georef
  method set_georef = fun wgs84 -> georef <- Some wgs84

  method projection = string_of_projection projection


  method world_of = fun wgs84 ->
    assert (LL.valid_geo wgs84);
    match georef with
        Some georef -> begin
          match projection with
              UTM ->
                let utmref = LL.utm_of LL.WGS84 georef
                and utm = LL.utm_of LL.WGS84 wgs84 in
                let (wx, y) = LL.utm_sub utm utmref in
                (wx, -.y)
            | Mercator ->
              let mlref = LL.mercator_lat georef.LL.posn_lat
              and ml = LL.mercator_lat wgs84.LL.posn_lat in
              let dl = LL.norm_angle (wgs84.LL.posn_long -. georef.LL.posn_long) in
              let xw = dl *. mercator_coeff
              and yw = -. (ml -. mlref) *. mercator_coeff in
              (xw, yw)
            | LambertIIe ->
              let lbtref = LL.lambertIIe_of georef
              and lbt = LL.lambertIIe_of wgs84 in
              let (wx, y) = LL.lbt_sub lbt lbtref in
              (wx, -.y)
        end
      | None -> failwith "#world_of : no georef"

  method pt2D_of = fun wgs84 ->
    let (x, y) = self#world_of wgs84 in
    {G2D.x2D = x; y2D = y}

  method of_world = fun (wx, wy) ->
    match georef with
        Some georef -> begin
          match projection with
              UTM ->
                let utmref = LL.utm_of LL.WGS84 georef in
                LL.of_utm LL.WGS84 (LL.utm_add utmref (wx, -.wy))
            | LambertIIe ->
              let utmref = LL.lambertIIe_of georef in
              LL.of_lambertIIe (LL.lbt_add utmref (wx, -.wy))
            | Mercator ->
              let mlref = LL.mercator_lat georef.LL.posn_lat in
              let ml = mlref -. wy /. mercator_coeff in
              let lat = LL.inv_mercator_lat ml
              and long = wx /. mercator_coeff +. georef.LL.posn_long in
              LL.make_geo lat long
        end
      | None -> failwith "#of_world : no georef"


  method convert_positions_to_points = fun geo_arr ->
    let getx = fun (x1, y1) -> x1 in
    let gety = fun (x1, y1) -> y1 in
    let arrlen = (Array.length geo_arr) in
    let points = Array.make (arrlen*2) (getx (self#world_of geo_arr.(0)))  in
    for i = 0 to arrlen - 1 do
      points.(i*2) <- getx (self#world_of geo_arr.(i));
      points.((i*2)+1) <- gety (self#world_of geo_arr.(i));
    done;
    points

  method move_item = fun ?(z = 1.) (item:GnomeCanvas.re_p GnoCanvas.item) wgs84 ->
    let (xw,yw) = self#world_of wgs84 in
    item#affine_absolute (affine_pos_and_angle ~z xw yw 0.);

  method moveto = fun wgs84 ->
    let (xw, yw) = self#world_of wgs84 in
    let (xc, yc) = canvas#world_to_window ~wox:xw ~woy:yw in
    canvas#scroll_to ~x:(truncate xc) ~y:(truncate yc)

  method center = fun wgs84 ->
    let (xw, yw) = self#world_of wgs84 in
    let (xc, yc) = canvas#world_to_window ~wox:xw ~woy:yw in
    let (xt, yt) = ((truncate xc), (truncate yc)) in
    let sx_w, sy_w = Gdk.Drawable.get_size canvas#misc#window in
    canvas#scroll_to ~x:(xt-sx_w/2) ~y:(yt-sy_w/2)

  method get_center = fun () ->
    let (x, y) = canvas#get_scroll_offsets
    and (sx_w, sy_w) = Gdk.Drawable.get_size canvas#misc#window in
    let xc = x + sx_w/2 and yc = y + sy_w/2 in
    let (xw, yw) = canvas#window_to_world  ~winx:(float xc) ~winy:(float yc) in
    self#of_world (xw, yw)


  method display_pixbuf = fun ?opacity ?level ((x1,y1), geo1) ((x2,y2), geo2) image ->
    let x1 = float x1 and x2 = float x2
    and y1 = float y1 and y2 = float y2 in
    let image =
      match opacity with
          None -> image
        | Some o -> set_opacity image o in
    let map_layer = match level with
      | None -> 0
      | Some l ->
        if l > Gm.zoom_max then
          Array.length maps - 1
        else if l < Gm.zoom_min then
          0
        else l - Gm.zoom_min
    in
    let pix = GnoCanvas.pixbuf ~x:(-.x1) ~y:(-.y1) ~pixbuf:image ~props:[`ANCHOR `NW] maps.(map_layer) in
    let xw1, yw1 = self#world_of geo1
    and xw2, yw2 = self#world_of geo2 in

    let scale = distance (xw1, yw1) (xw2, yw2) /. distance (x1,y1) (x2,y2) in
    let a = atan2 (yw2-.yw1) (xw2-.xw1) -. atan2 (y2-.y1) (x2-.x1) in
    let cos_a = cos a *. scale and sin_a = sin a *. scale in
    pix#move ~x:xw1 ~y:yw1;
    pix#affine_relative [| cos_a; sin_a; -. sin_a; cos_a; 0.;0.|];
    pix

  method fix_bg_coords (xw, yw) = (** FIXME: how to do it properly ? *)
    ((xw +. 25000000.) *. zoom_level, (yw +. 25000000.) *. zoom_level)

  method zoom = fun value ->
    let value = min max_zoom value in
    zoom_level <- value;  (* must set this before changing adj so that another zoom is not triggered *)
    adj#set_value value;
    canvas#set_pixels_per_unit value

  (**  events *******************************************)
  method background_event = fun ev ->
    match ev with
      | `BUTTON_PRESS ev when GdkEvent.Button.button ev = 1 ->
        begin
          let xc = GdkEvent.Button.x ev
          and yc = GdkEvent.Button.y ev
          and state = GdkEvent.Button.state ev in
          let (xw,yw) = self#window_to_world ~winx:xc ~winy:yc in
          let (xw, yw) = self#fix_bg_coords (xw, yw) in
          if Gdk.Convert.test_modifier `SHIFT state then begin
            drawing <- Rectangle (xw,yw);
            region_rectangle#set [`X1 xw; `Y1 yw; `X2 xw; `Y2 yw];
            region_rectangle#raise_to_top ()
          end else begin (* panning *)
            drawing <- Panning (xc, yc);
            let curs = Gdk.Cursor.create `FLEUR in
            background#grab [`POINTER_MOTION; `BUTTON_RELEASE] curs
              (GdkEvent.Button.time ev)
          end;
          true
        end
      | `MOTION_NOTIFY ev ->
        begin
          let xc = GdkEvent.Motion.x ev
          and yc = GdkEvent.Motion.y ev in
          let (xw, yw) = self#window_to_world ~winx:xc ~winy:yc in
          let (xw, yw) = self#fix_bg_coords (xw, yw) in
          match drawing with
              Rectangle (x1,y1) ->
                let starting_point = self#of_world (x1,y1) in
                let starting_point = LL.utm_of LL.WGS84 starting_point in
                let current_point = LL.utm_of LL.WGS84 (self#of_world (xw, yw)) in
                let (east, north) = LL.utm_sub current_point starting_point in
                region_rectangle#set [`X2 xw; `Y2 yw];
                self#display_group (sprintf "[%.0fm %.0fm]" east north)
            | Panning (x0, y0) ->
              let xc = GdkEvent.Motion.x ev
              and yc = GdkEvent.Motion.y ev in
              let dx = zoom_level *. (xc -. x0)
              and dy = zoom_level *. (yc -. y0) in
              let (x, y) = canvas#get_scroll_offsets in
              canvas#scroll_to ~x:(x-truncate dx) ~y:(y-truncate dy)
            | _ -> ()
        end;
        false
      | `BUTTON_RELEASE ev when GdkEvent.Button.button ev = 1 ->
        begin
          let xc = GdkEvent.Button.x ev in
          let yc = GdkEvent.Button.y ev in
          let current_point = self#window_to_world ~winx:xc ~winy:yc in
          let current_point = self#fix_bg_coords  current_point in
          match drawing with
              Rectangle (x1,y1) ->
                region <- Some ((x1,y1), current_point);
                self#display_group "";
                drawing <- NotDrawing;
                true
            | Panning _ ->
              drawing <- NotDrawing;
              background#ungrab (GdkEvent.Button.time ev);
              true
            | _ -> false
        end
      | _ -> false


  method mouse_motion = fun ev ->
    if georef <> None then begin
      let xc = GdkEvent.Motion.x ev
      and yc = GdkEvent.Motion.y ev in
      let (xw, yw) = self#window_to_world ~winx:xc ~winy:yc in
      self#display_geo (self#of_world (xw,yw));
      self#display_alt (self#of_world (xw,yw));
      let (x, y) = canvas#get_scroll_offsets in
      last_mouse_x <- truncate xc - x;
      last_mouse_y <- truncate yc - y
    end;
    false

  method switch_background = fun x -> if x then Array.iter (fun m -> m#show ()) maps else Array.iter (fun m -> m#hide ()) maps


  method key_press = fun ev ->
    let (x, y) = canvas#get_scroll_offsets in
    match GdkEvent.Key.keyval ev with
      | k when k = GdkKeysyms._Up -> canvas#scroll_to ~x ~y:(y-pan_step) ; true
      | k when k = GdkKeysyms._Down -> canvas#scroll_to ~x ~y:(y+pan_step) ; true
      | k when k = GdkKeysyms._Left -> canvas#scroll_to ~x:(x-pan_step) ~y ; true
      | k when k = GdkKeysyms._Right -> canvas#scroll_to ~x:(x+pan_step) ~y ; true
      | k when k = GdkKeysyms._f -> self#fit_to_window () ; true
      | k when k = GdkKeysyms._Page_Up ->
        self#zoom_up ();
        true
      | k when k = GdkKeysyms._Page_Down ->
        self#zoom_down ();
        true
      | _ -> false

  method connect_view = fun cb ->
    Hashtbl.add view_cbs cb ()

  (* zoom keeping the center *)
  method zoom_in_center = fun z ->
    let c = self#get_center () in
    self#zoom z;
    self#center c

  (* zoom keeping the area under the mouse pointer *)
  method zoom_in_place = fun z ->
    let (x, y) = canvas#get_scroll_offsets in
    canvas#scroll_to ~x:(x+last_mouse_x) ~y:(y+last_mouse_y);

    self#zoom z;

    let (x, y) = canvas#get_scroll_offsets in
    canvas#scroll_to ~x:(x-last_mouse_x) ~y:(y-last_mouse_y)



  method zoom_up () =
    self#zoom_in_place (zoom_level*.zoom_factor);
  method zoom_down () =
    self#zoom_in_place (zoom_level/.zoom_factor);

  method any_event =
    let rec last_view = ref (0,0,0,0) in
    fun ev ->
      let width_c, height_c = Gdk.Drawable.get_size canvas#misc#window
      and (xc0, yc0) = canvas#get_scroll_offsets in
      let view = (xc0, yc0, width_c, height_c) in
      (** View has changed ? *)
      if view <> !last_view then begin
        last_view := view;
        Hashtbl.iter (fun cb _ -> cb ()) view_cbs
      end;
      try
        match GdkEvent.get_type ev with
          | `SCROLL when not (Gdk.Convert.test_modifier `SHIFT (GdkEvent.Scroll.state (GdkEvent.Scroll.cast ev))) -> begin
            let scroll_event = GdkEvent.Scroll.cast ev in
            match GdkEvent.Scroll.direction scroll_event with
                `UP   -> self#zoom_up (); true
              | `DOWN -> self#zoom_down (); true
              | _     -> false
          end
          | _ ->    false
      with
          Invalid_argument _ -> (* Raised GdkEvent.get_type *)
            false



  method segment = fun ?(group = canvas#root) ?(width=1) ?fill_color geo1 geo2 ->
    let (x1, y1) = self#world_of geo1
    and (x2, y2) = self#world_of geo2 in
    let l = GnoCanvas.line ?fill_color ~props:[`WIDTH_PIXELS width] ~points:[|x1;y1;x2;y2|] group in
    l#show ();
    l

  method arc = fun ?(nb_points=5) ?(width=1) ?fill_color (xw,yw) r a1 a2 ->
    let c = {G2D.x2D = xw; y2D = yw } in
    let pts = G2D.arc ~nb_points c r a1 a2 in
    let points = Array.init (2*nb_points)
      (fun j ->
        let i = j / 2 in
        if j = i * 2 then pts.(i).G2D.x2D else pts.(i).G2D.y2D) in
    let _p = points in
    let l = GnoCanvas.line ?fill_color ~props:[`WIDTH_PIXELS width] ~points canvas#root in
    l#show ();
    l


  method circle = fun ?(group = canvas#root) ?(width=1) ?fill_color ?(opacity=0) ?(color="black") geo radius ->
    let (x, y) = self#world_of geo in
    let (stpwidth, stpstr) = stipple_opacity opacity in
    (** Compute the actual radius in a UTM projection *)
    let utm = LL.utm_of LL.WGS84 geo in
    let geo_east = LL.of_utm LL.WGS84 (LL.utm_add utm (radius, 0.)) in
    let (xe, _) = self#world_of geo_east in
    let rad = xe -. x in
    let l = GnoCanvas.ellipse ?fill_color ~props:[`WIDTH_PIXELS width; `OUTLINE_COLOR color; `FILL_STIPPLE (Gdk.Bitmap.create_from_data ~width:stpwidth ~height:stpwidth stpstr)] ~x1:(x-.rad) ~y1:(y -.rad) ~x2:(x +.rad) ~y2:(y+.rad) group in
    l#show ();
    l

  method polygon = fun ?(group = canvas#root) ?(width=1) ?fill_color ?(opacity=0) ?(color="black") geo_arr ->
    (*setting opacity from 0-4 *)
    let (stpwidth, stpstr) = stipple_opacity opacity in
    let points = self#convert_positions_to_points geo_arr in
    let l = GnoCanvas.polygon ?fill_color ~props:[`WIDTH_PIXELS width; `OUTLINE_COLOR color; `FILL_STIPPLE (Gdk.Bitmap.create_from_data ~width:stpwidth ~height:stpwidth stpstr)] ~points group in
    l#show ();
    l

  method photoprojection = fun ?(group = canvas#root) ?(width=1) ?fill_color ?(color="black") ?(number="1") geo radius ->
    let (x, y) = self#world_of geo in

    (** Compute the actual radius in a UTM projection *)
    let utm = LL.utm_of LL.WGS84 geo in
    let geo_east = LL.of_utm LL.WGS84 (LL.utm_add utm (radius, 0.)) in
    let (xe, _) = self#world_of geo_east in
    let rad = xe -. x in
    let l1 = GnoCanvas.ellipse ?fill_color ~props:[`WIDTH_PIXELS width; `OUTLINE_COLOR color] ~x1:(x-.rad) ~y1:(y -.rad) ~x2:(x +.rad) ~y2:(y+.rad) group in
    let l2 = GnoCanvas.text ~x:(x) ~y:(y) ~text:number ~props:[`FILL_COLOR color; `X_OFFSET 0.0; `Y_OFFSET 0.0] group in
    l1#show ();
    l2#show ();
    l2

  method text = fun ?(group = canvas#root) ?(fill_color = "blue") ?(x_offset = 0.0) ?(y_offset = 0.0) geo text ->
    let (x1, y1) = self#world_of geo in
    let t = GnoCanvas.text ~x:x1 ~y:y1 ~text:text ~props:[`FILL_COLOR fill_color; `X_OFFSET x_offset; `Y_OFFSET y_offset] group in
    t#show ();
    t

  initializer
  let replace_still = fun _ ->
    let (x, y) = canvas#get_scroll_offsets in
    let (xc, yc) = canvas#window_to_world ~winx:(float x) ~winy:(float y) in
    let z = 1./.zoom_level in
    still#affine_absolute [|z;0.;0.;z;xc;yc|]
  in
  self#connect_view replace_still;
  let move_timer = ref (Glib.Timeout.add ~ms:0 ~callback:(fun _ -> false)) in
  let move dx dy = function
  `BUTTON_PRESS _ ->
    let scroll = fun _ ->
      let (x, y) = canvas#get_scroll_offsets in
      canvas#scroll_to ~x:(x+dx) ~y:(y+dy) ; true in
    move_timer := Glib.Timeout.add ~ms:50 ~callback:scroll;
    true
    | `BUTTON_RELEASE _ ->
      Glib.Timeout.remove !move_timer;
      true
    | _ -> false in
  let up = move 0 (-pan_step)
  and down = move 0 pan_step
      and left = move (-pan_step) 0
      and right = move pan_step 0 in
      ignore (north_arrow#connect#event ~callback:up);
      ignore (south_arrow#connect#event ~callback:down);
      ignore (west_arrow#connect#event ~callback:left);
      ignore (east_arrow#connect#event ~callback:right)
  end


(** canvas which inherits from basic_widget  ********************
 * - labels for displaying mouse coordinates on the map         *
 * - background switching                                       *
 ****************************************************************)


class widget =  fun ?(height=800) ?(srtm=false) ?width ?projection ?georef () ->
  let srtm = GMenu.check_menu_item ~label:"display SRTM alt" ~active:srtm () in
  let lbl_xy = GMisc.label ()
  and lbl_geo = GMisc.label ()
  and lbl_alt =  GMisc.label ()
  and lbl_group = GMisc.label () in

  let georef_menu = GMenu.menu ()
  and optmenu = GMenu.option_menu () in

  object(self)
    inherit (basic_widget ~height ?width ?projection ?georef ())

    val mutable utm_grid_group = None
    val mutable georefs = []
    val mutable selected_georef = WGS84_dec

    method pack_labels =
      self#info#pack lbl_xy#coerce;
      self#info#pack optmenu#coerce;
      self#info#pack lbl_geo#coerce;
      self#info#pack lbl_alt#coerce;
      self#info#pack lbl_group#coerce;

    initializer (
      self#pack_labels;
      self#file_menu#append (srtm :> GMenu.menu_item);
      ignore (my_check_menu_item "UTM Grid" ~active:false ~callback:self#switch_utm_grid ~packing:self#file_menu#append ());
      ignore (my_check_menu_item "UTC Time" ~active:false ~callback:self#switch_utc_time ~packing:self#file_menu#append ());
      let bg_menu = my_check_menu_item "Background" ~active:true ~callback:self#switch_background ~packing:self#file_menu#append () in

      let tooltips = GData.tooltips () in
      tooltips#set_tip srtm#coerce ~text:"Display SRTM alt at pointer position (will request for download if not available)";

      let b = GButton.button ~packing:toolbar#add () in
      ignore (b#connect#clicked ~callback:(fun _ -> bg_menu#activate ()));
      let pixbuf = GdkPixbuf.from_file (Env.gcs_icons_path // "switch_background.png") in
      ignore (GMisc.image ~pixbuf ~packing:b#add ());
      tooltips#set_tip b#coerce ~text:"Toggle background";

      my_menu_item "Goto" ~callback:self#goto ~packing:self#file_menu#append ();

      let callback = self#fit_to_window in
      my_menu_item "Fit to window (f)" ~callback ~packing:self#file_menu#append ();
      let b = GButton.button ~packing:toolbar#add () in
      ignore (b#connect#clicked ~callback);
      ignore (GMisc.image ~stock:(`STOCK "gtk-zoom-fit") ~packing:b#add ());
      tooltips#set_tip b#coerce ~text:"Fit to window";

      let set = fun x () -> selected_georef <- x in
      my_menu_item "WGS84" ~packing:georef_menu#append ~callback:(set WGS84_dec) ();
      my_menu_item "WGS84_dms" ~packing:georef_menu#append ~callback:(set WGS84_dms) ();
      my_menu_item "LambertIIe" ~packing:georef_menu#append ~callback:(set LBT2e) ();
      optmenu#set_menu georef_menu
     )

    method switch_utc_time = fun flag ->
      if flag then self#utc_time#show () else self#utc_time#hide ()

    method switch_utm_grid = fun flag ->
      match georef with
    None -> ()
      | Some georef ->
      match utm_grid_group with
        None ->
          if flag then (** Create and show *)
        let g = GnoCanvas.group self#canvas#root in
        let u0 = LL.utm_of LL.WGS84 (self#get_center ()) in
        let u0 = { LL.utm_x = align u0.LL.utm_x 1000;
               LL.utm_zone = u0.LL.utm_zone;
               LL.utm_y = align u0.LL.utm_y 1000 } in
        for i = -size_utm_grid to size_utm_grid do
          let h = Array.make (2*(2*size_utm_grid+1)) 0.
          and v = Array.make (2*(2*size_utm_grid+1)) 0. in
          for j = -size_utm_grid to size_utm_grid do
            let k = 2*(j+size_utm_grid) in
            let p = fun i j ->
              let u = LL.utm_add u0 (float (i*1000), float (j*1000)) in
              let wgs84 = LL.of_utm LL.WGS84 u in
              self#world_of wgs84 in
            let (xw,yw) = p i j in
            h.(k) <- xw; h.(k+1) <- yw;
            let (xw,yw) = p j i in
            v.(k) <- xw; v.(k+1) <- yw
          done;
          let h = GnoCanvas.line ~fill_color:grid_color ~props:[`WIDTH_PIXELS 1] ~points:h g
          and v = GnoCanvas.line ~fill_color:grid_color ~props:[`WIDTH_PIXELS 1] ~points:v g in
          h#show (); v#show ()
        done;
        utm_grid_group <- Some g
      | Some g -> if flag then g#show () else g#hide ()

    (** ground altitude extraction from srtm data *)
    method altitude = fun wgs84 ->
      try
        Srtm.of_wgs84 wgs84
      with
        Srtm.Tile_not_found x ->
          srtm#set_active false;
          (*GToolbox.message_box "SRTM" (sprintf "SRTM tile %s not found: %s ?" x (Srtm.error x));*)
          let msg = (sprintf "Oups, I can't find SRTM tile %s.\nCan I try to donwload it ?\n(%s)" x (Srtm.error x)) in
          match GToolbox.question_box ~title:"SRTM" ~buttons:["Download"; "Cancel"] msg with
          | 1 ->
              begin try
                let tile_zip = x^".SRTMGL1.hgt.zip" in
                let url = Srtm.srtm_url // tile_zip in
                let dest = Env.paparazzi_home // "data" // "srtm" // tile_zip in
                let tmp_dest = Env.paparazzi_home // "var" // tile_zip in
                ignore(Http.file_of_url ~dest:tmp_dest url);
                Sys.rename tmp_dest dest;
                srtm#set_active true;
                self#altitude wgs84
              with
              | Http.Failure _ | Srtm.Tile_not_found _ ->
                  GToolbox.message_box ~title:"SRTM" ("Sorry, tile "^x^" couldn't be downloaded");
                  0
              end
          | _ -> 0

    method georefs = georefs

    method add_info_georef = fun name geo ->
      (* add to the end so georefs has same order as waypoint list *)
      georefs <- List.append georefs [(name, geo)];
      let callback = fun () -> selected_georef <- Bearing geo in
      my_menu_item name ~packing:georef_menu#append ~callback ();

    (* change wp name *)
    method edit_georef_name = fun oldname newname ->
      (* get offset between WP list and menu list *)
      let extraitems = (List.length georef_menu#children) - (List.length georefs) in
      if newname <> oldname then
        georefs <- List.fold_left (fun l (label, geo) ->
          if label = oldname then
            begin
              let callback = fun () -> selected_georef <- Bearing geo in
              let pos = List.length l + extraitems in
              (* remove item and readd with new name *)
              georef_menu#remove (List.nth georef_menu#children pos);
              (*my_menu_item_insert newname ~menu:georef_menu ~pos:menupos ~callback;*)
              my_menu_item newname ~packing:(georef_menu#insert ~pos) ~callback ();
              if selected_georef = (Bearing geo) then optmenu#set_history pos;
              List.append l [(newname, geo)]
            end
          else
            List.append l [(label, geo)]
        ) [] georefs

    (* Delete item from georefs and from menu *)
    method delete_georef = fun name ->
      (* get offset between WP list and menu list *)
      let extraitems = (List.length georef_menu#children) - (List.length georefs) in
      georefs <- List.fold_left (fun l (label, geo) ->
        if label = name then
          begin
            let menupos = List.length l + extraitems in
            (* remove item *)
            georef_menu#remove (List.nth georef_menu#children menupos);
            if selected_georef = (Bearing geo) then
              begin
                (List.nth georef_menu#children 0)#activate ();
                optmenu#set_history 0;
              end;
            l
          end
        else
          List.append l [(label, geo)]
      ) [] georefs

    (* delete all wp, including fitted objects *)
    method clear_georefs = fun () ->
      (* get offset between WP list and menu list *)
      let extraitems = (List.length georef_menu#children) - (List.length georefs) in
      (* delete items from georefs and from menu *)
      ignore (List.fold_left (fun i v ->
        if i >= extraitems then georef_menu#remove v;
        i + 1
      ) 0 georef_menu#children);
      (List.nth georef_menu#children 0)#activate ();
      optmenu#set_history 0;
      georefs <- [];
      (* finally delete all fitted objects *)
      fitted_objects <- [];


    (** display methods *)
    method display_xy = fun s ->  lbl_xy#set_text s
    method display_geo = fun geo ->
      lbl_geo#set_text (LL.string_of_coordinates selected_georef geo)


    method display_alt = fun wgs84 ->
      if srtm#active then
        lbl_alt#set_text (sprintf "  SRTM:%dm"(self#altitude wgs84))
      else if not (Srtm.available wgs84) then
        lbl_alt#set_text (sprintf "  SRTM: N/A")

    method display_group = fun s ->  lbl_group#set_text s

    method goto = fun () ->
      match GToolbox.input_string ~title:"Geo ref" ~text:"WGS84 " "Geo ref" with
    Some s ->
      let wgs84 = Latlong.of_string s in
      if georef = None then
        self#set_georef wgs84;
      self#moveto wgs84
      | None -> ()

end
