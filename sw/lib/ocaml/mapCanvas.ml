(*
 * $Id$
 *
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
module G2D = Geometry_2d
open Printf

let zoom_factor = 1.5 (* Mouse wheel zoom action *)
let pan_step = 50 (* Pan keys speed *)

let grid_color = "#29d3f8"
let size_utm_grid = 10 (* half the horiz/vert size in km *)

let align = fun x a ->
  float (truncate (x /. float a) * a)

type meter = float

let distance = fun (x1,y1) (x2,y2) -> sqrt ((x1-.x2)**2.+.(y1-.y2)**2.)
      
let _ = Srtm.add_path "SRTM"

let affine_pos_and_angle xw yw angle =
  let cos_a = cos angle in
  let sin_a = sin angle in
  [| cos_a ; sin_a ; ~-. sin_a; cos_a; xw ; yw |]

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
  ignore (mi#connect#toggled ~callback:(fun () -> callback mi#active))

let my_menu_item = fun label ~callback ~packing () ->
  let mi = GMenu.check_menu_item ~label ~packing () in
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


type drawing = NotDrawing | Rectangle of float*float | Polygon of (float*float) list


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
	    


(** basic canvas with menubar **************************************
 * (the vertical display in map2.ml is an instance of basic_widget)*
 *******************************************************************)
    
(* world_unit: m:pixel at scale 1. *)
class basic_widget = fun ?(height=800) ?width ?(projection = Mercator) ?georef () -> 
  let frame = GPack.vbox ~height ?width () in

  let top_bar = GPack.hbox ~packing:frame#pack () in

  let menubar = GMenu.menu_bar ~packing:(top_bar#pack ~expand:true) () in
  let file_menu_item = GMenu.menu_item ~label:"Nav" ~packing:menubar#append () in
  let file_menu = GMenu.menu () in
  let _ = file_menu_item#set_submenu file_menu in
  let factory = new GMenu.factory menubar in

  let info = GPack.hbox ~packing:top_bar#pack () in

  let spin_button = GEdit.spin_button  ~rate:0. ~digits:2 ~width:50 (*** ~height:20 ***) ~packing:top_bar#pack () in

  let canvas = GnoCanvas.canvas ~packing:(frame#pack ~expand:true) () in
  let background = GnoCanvas.group canvas#root
  and still = GnoCanvas.group canvas#root in
  let view_cbs = Hashtbl.create 3 in (* Store for view event callback *)
  let region_rectangle = GnoCanvas.rect canvas#root ~props:[`WIDTH_PIXELS 2; `OUTLINE_COLOR "red"] in
  let region_polygon = GnoCanvas.polygon canvas#root ~props:[`WIDTH_PIXELS 2; `OUTLINE_COLOR "red"] in

  let s = 50. in
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

  object (self)
   
(** GUI attributes *)

    val background = background
    method background = background
    method still = still
    method top_still = 3.5*.s
	
    val adj = GData.adjustment 
	~value:1. ~lower:0.05 ~upper:10. 
	~step_incr:0.25 ~page_incr:1.0 ~page_size:1.0 ()
  
    val info = info
	
	

(** other attributes *)

    val mutable projection = projection	
    val mutable georef = georef
    val mutable dragging = None
    val mutable drawing  = NotDrawing
    val mutable region = None (* Rectangle selected region *)
    val mutable polygon = None (* Polygon selected region *)

    method region = region
    method polygon = polygon

(** initialization of instance attributes *)

    initializer (

      spin_button#set_adjustment adj;

(** callback bindings *)

(***      canvas#coerce#misc#modify_bg [`NORMAL, `NAME "black"];***)
      ignore (background#connect#event self#background_event);

      ignore (canvas#event#connect#motion_notify self#mouse_motion);
      ignore (canvas#event#connect#after#key_press self#key_press) ;
      ignore (canvas#event#connect#enter_notify (fun _ -> self#canvas#misc#grab_focus () ; false));
      ignore (canvas#event#connect#any self#any_event);
      ignore (adj#connect#value_changed (fun () -> canvas#set_pixels_per_unit adj#value));

      canvas#set_center_scroll_region false ;
      canvas#set_scroll_region (-25000000.) (-25000000.) 25000000. 25000000.;
      ignore (GnoCanvas.rect ~props:[`X1 (-25000000.); `Y1 (-25000000.); `X2 25000000.; `Y2 25000000.; `FILL_COLOR "black"] background);

     )
 


(** methods *)

(** accessors to instance variables *)	
    method current_zoom = adj#value
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
	      let xw = (wgs84.LL.posn_long -. georef.LL.posn_long) *. mercator_coeff
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
	      { LL.posn_lat = lat; posn_long = long }
	end
      | None -> failwith "#of_world : no georef"

		
    method geo_string = fun wgs84 ->
      LL.string_degrees_of_geographic wgs84

    	    
    method moveto = fun wgs84 ->
      let (xw, yw) = self#world_of wgs84 in
      let (xc, yc) = canvas#world_to_window xw yw in
      canvas#scroll_to (truncate xc) (truncate yc)

    method center = fun wgs84 ->
      self#moveto wgs84;
      let sx_w, sy_w = Gdk.Drawable.get_size canvas#misc#window
      and (x, y) = canvas#get_scroll_offsets in
      canvas#scroll_to (x-sx_w/2) (y-sy_w/2)
			
    method display_pixbuf = fun ?opacity ((x1,y1), geo1) ((x2,y2), geo2) image ->
      let x1 = float x1 and x2 = float x2
      and y1 = float y1 and y2 = float y2 in
      let image = 
	match opacity with
	  None -> image
	| Some o -> set_opacity image o in
      let pix = GnoCanvas.pixbuf ~x:(-.x1) ~y:(-.y1)~pixbuf:image ~props:[`ANCHOR `NW] background in
      let xw1, yw1 = self#world_of geo1
      and xw2, yw2 = self#world_of geo2 in
      let scale = distance (xw1, yw1) (xw2, yw2) /. distance (x1,y1) (x2,y2) in
      let a = atan2 (yw2-.yw1) (xw2-.xw1) -. atan2 (y2-.y1) (x2-.x1) in
      let cos_a = cos a *. scale and sin_a = sin a *. scale in
      pix#move xw1 yw1;
      pix#affine_relative [| cos_a; sin_a; -. sin_a; cos_a; 0.;0.|];
      pix

    method fix_bg_coords (xw, yw) = (** FIXME: how to do it properly ? *)
      let z = self#current_zoom in
      ((xw +. 25000000.) *. z, (yw +. 25000000.) *. z)
	
    method zoom = fun value ->
      adj#set_value value

   (**  events *******************************************)
    method background_event = fun ev ->
      match ev with
      | `BUTTON_PRESS ev when GdkEvent.Button.button ev = 1 ->
	  begin
	    let xc = GdkEvent.Button.x ev
	    and yc = GdkEvent.Button.y ev
	    and state = GdkEvent.Button.state ev in
	    let (xw,yw) = self#window_to_world xc yc in
	    let (xw, yw) = self#fix_bg_coords (xw, yw) in
	    match Gdk.Convert.modifier state with
	      [] ->
		drawing <- Rectangle (xw,yw);
		region_rectangle#set [`X1 xw; `Y1 yw; `X2 xw; `Y2 yw];
		region_rectangle#raise_to_top ();
		true
	    | [`SHIFT] ->
		drawing <- Polygon [(xw, yw)];
		true;
	    | _ -> false
	  end
      | `MOTION_NOTIFY ev ->
	  begin
	    let xc = GdkEvent.Motion.x ev 
	    and yc = GdkEvent.Motion.y ev in
	    let (xw, yw) = self#window_to_world xc yc in
	    let (xw, yw) = self#fix_bg_coords (xw, yw) in
	    match drawing with
	      Rectangle (x1,y1) ->
		let starting_point = self#of_world (x1,y1) in
		let starting_point = LL.utm_of LL.WGS84 starting_point in
		let current_point = LL.utm_of LL.WGS84 (self#of_world (xw, yw)) in
		let (east, north) = LL.utm_sub current_point starting_point in
		region_rectangle#set [`X2 xw; `Y2 yw];
		self#display_group (sprintf "[%.1fkm %.1fkm]" (east/.1000.) (north/.1000.))
	    | Polygon ps ->
		let points = convex ((xw,yw)::ps) in
		drawing <- Polygon points;
		region_polygon#set [`POINTS (float_array_of_points points)]
	    | _ -> ()
	  end;
	  false
      | `BUTTON_RELEASE ev when GdkEvent.Button.button ev = 1 ->
	  begin
	    let xc = GdkEvent.Button.x ev in
	    let yc = GdkEvent.Button.y ev in
	    let current_point = self#window_to_world xc yc in
	    let current_point = self#fix_bg_coords  current_point in
	    match drawing with
	      Rectangle (x1,y1) ->
		    region <- Some ((x1,y1), current_point);
		self#display_group "";
		drawing <- NotDrawing;
		true
	    | Polygon points ->
		    drawing <- NotDrawing;
		polygon <- Some points;
		true
	    | _ -> false
	  end
      | _ -> false
		
		
    method mouse_motion = fun ev ->
      if georef <> None then begin
	let xc = GdkEvent.Motion.x ev 
	and yc = GdkEvent.Motion.y ev in
	let (xw, yw) = self#window_to_world xc yc in
	self#display_geo (self#geo_string (self#of_world (xw,yw)));
	self#display_alt (self#of_world (xw,yw));
      end;
      false
	  
   method key_press = fun ev ->
      let (x, y) = canvas#get_scroll_offsets in
      match GdkEvent.Key.keyval ev with
      | k when k = GdkKeysyms._Up -> canvas#scroll_to x (y-pan_step) ; true
      | k when k = GdkKeysyms._Down -> canvas#scroll_to x (y+pan_step) ; true
      | k when k = GdkKeysyms._Left -> canvas#scroll_to (x-pan_step) y ; true
      | k when k = GdkKeysyms._Right -> canvas#scroll_to (x+pan_step) y ; true
      | k when k = GdkKeysyms._Page_Up -> adj#set_value (adj#value+.adj#step_increment) ; true
      | k when k = GdkKeysyms._Page_Down -> adj#set_value (adj#value-.adj#step_increment) ; true
      | _ -> false

    method connect_view = fun cb ->
      Hashtbl.add view_cbs cb ()

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
	
	match GdkEvent.get_type ev with
	| `SCROLL when not (Gdk.Convert.test_modifier `SHIFT (GdkEvent.Scroll.state (GdkEvent.Scroll.cast ev))) -> begin
	    let scroll_event = GdkEvent.Scroll.cast ev in
	    let (x, y) = canvas#get_scroll_offsets in
	    let xr = GdkEvent.Scroll.x_root scroll_event in
	    let yr = GdkEvent.Scroll.y_root scroll_event -. 50. in
	    match GdkEvent.Scroll.direction scroll_event with
	      `UP    ->
		canvas#scroll_to (x+truncate xr) (y+truncate yr);
		
		adj#set_value (adj#value*.zoom_factor);
		
		let (x, y) = canvas#get_scroll_offsets in
		canvas#scroll_to (x-truncate (xr)) (y-truncate (yr));
		true
	    | `DOWN  ->
		canvas#scroll_to (x+truncate xr) (y+truncate yr);
		
		adj#set_value (adj#value/.zoom_factor);
		
		let (x, y) = canvas#get_scroll_offsets in
		canvas#scroll_to (x-truncate (xr)) (y-truncate (yr));
		true
	    | _  -> false
	end
	| _ -> false
	    
	    
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

	
    method circle = fun ?(group = canvas#root) ?(width=1) ?fill_color ?(color="black") geo radius ->
      let (x, y) = self#world_of geo in

      (** Compute the actual radius in a UTM projection *)
      let utm = LL.utm_of LL.WGS84 geo in
      let geo_east = LL.of_utm LL.WGS84 (LL.utm_add utm (radius, 0.)) in
      let (xe, _) = self#world_of geo_east in
      let rad = xe -. x in      

      let l = GnoCanvas.ellipse ?fill_color ~props:[`WIDTH_PIXELS width; `OUTLINE_COLOR color] ~x1:(x-.rad) ~y1:(y -.rad) ~x2:(x +.rad) ~y2:(y+.rad) group in
      l#show ();
      l

    method text = fun ?(group = canvas#root) ?(fill_color = "blue") ?(x_offset = 0.0) ?(y_offset = 0.0) geo text ->
      let (x1, y1) = self#world_of geo in
      let t = GnoCanvas.text ~x:x1 ~y:y1 ~text:text ~props:[`FILL_COLOR fill_color; `X_OFFSET x_offset; `Y_OFFSET y_offset] group in
      t#show ();
      t

    initializer
      let replace_still = fun _ ->
	let (x, y) = canvas#get_scroll_offsets in
	let (xc, yc) = canvas#window_to_world (float x) (float y) in
	let z = 1./.self#current_zoom in
	still#affine_absolute [|z;0.;0.;z;xc;yc|]
      in
      self#connect_view replace_still;
      let move_timer = ref (Glib.Timeout.add 0 (fun _ -> false)) in
      let move dx dy = function
	  `BUTTON_PRESS _ ->
	    let scroll = fun _ ->
	      let (x, y) = canvas#get_scroll_offsets in
	      canvas#scroll_to (x+dx) (y+dy) ; true in
	    move_timer := Glib.Timeout.add 50 scroll;
	    true
	| `BUTTON_RELEASE _ ->
	    Glib.Timeout.remove !move_timer;
	    true
	| _ -> false in
      let up = move 0 (-pan_step)
      and down = move 0 pan_step
      and left = move (-pan_step) 0
      and right = move pan_step 0 in
      ignore (north_arrow#connect#event up);
      ignore (south_arrow#connect#event down);
      ignore (west_arrow#connect#event left);
      ignore (east_arrow#connect#event right)
  end
    

(** canvas which inherits from basic_widget  ********************
 * - labels for displaying mouse coordinates on the map         *
 * - background switching                                       *
 *(the horizontal map in map2.ml is an instance of basic_widget)*
 ****************************************************************)

    
class widget =  fun ?(height=800) ?width ?projection ?georef () ->
  let srtm = GMenu.check_menu_item ~label:"SRTM" ~active:false () in
  object(self)
    inherit (basic_widget ~height ?width ?projection ?georef ())

    val mutable lbl_xy = GMisc.label ()
    val mutable lbl_geo = GMisc.label  ()
    val mutable lbl_alt =  GMisc.label ()
    val mutable lbl_group = GMisc.label ()   
    val mutable utm_grid_group = None

    method pack_labels =
      info#pack lbl_xy#coerce;
      info#pack lbl_geo#coerce;
      info#pack lbl_alt#coerce;
      info#pack lbl_group#coerce;
      
    initializer (
      self#pack_labels;
      self#file_menu#append (srtm :> GMenu.menu_item);
      my_check_menu_item "UTM Grid" ~active:false ~callback:self#switch_utm_grid ~packing:self#file_menu#append ();
      my_check_menu_item "Background" ~active:true ~callback:self#switch_background ~packing:self#file_menu#append ();
      my_menu_item "Goto" ~callback:self#goto ~packing:self#file_menu#append ();
     )

    method switch_utm_grid = fun flag ->
      match georef with
	None -> ()
      |	Some georef ->
	  match utm_grid_group with
	    None ->
	      if flag then (** Create and show *)
		let g = GnoCanvas.group self#canvas#root in
		let u0 = LL.utm_of LL.WGS84 georef in
		let u0 = { LL.utm_x = align u0.LL.utm_x 1000; 
			   LL.utm_zone = u0.LL.utm_zone;
			   LL.utm_y = align u0.LL.utm_y 1000 } in
		for i = -size_utm_grid to size_utm_grid do
		  let h = Array.create (2*(2*size_utm_grid+1)) 0.
		  and v = Array.create (2*(2*size_utm_grid+1)) 0. in
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
	  GToolbox.message_box "SRTM" (sprintf "SRTM tile %s not found: %s ?" x (Srtm.error x));
	  0

    (** display methods *)
	   
    method display_xy = fun s ->  lbl_xy#set_text s
    method display_geo = fun s ->  lbl_geo#set_text s
    method display_alt = fun wgs84 ->
      if srtm#active then
	lbl_alt#set_text (sprintf "\t%dm"(self#altitude wgs84))
	
    method display_group = fun s ->  lbl_group#set_text s
	

    method switch_background = fun x -> if x then background#show () else background#hide ()

    method goto = fun () ->
      match GToolbox.input_string ~title:"Geo ref" ~text:"WGS84 " "Geo ref" with
	Some s ->
	  let wgs84 = Latlong.of_string s in
	  self#moveto wgs84
      | None -> ()
	
end
