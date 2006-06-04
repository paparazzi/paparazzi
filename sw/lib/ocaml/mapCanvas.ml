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


(** basic canvas with menubar **************************************
 * (the vertical display in map2.ml is an instance of basic_widget)*
 *******************************************************************)
    
(* world_unit: m:pixel at scale 1. *)
class basic_widget = fun ?(height=800) ?width ?(projection = Mercator) ?georef () ->  
  let canvas = GnoCanvas.canvas () in
  let background = GnoCanvas.group canvas#root in
  let view_cbs = Hashtbl.create 3 in (* Store for view event callback *)
  let region_rectangle = GnoCanvas.rect canvas#root ~props:[`WIDTH_PIXELS 2; `OUTLINE_COLOR "red"] in

  let menubar = GMenu.menu_bar () in
  let file_menu_item = GMenu.menu_item ~label:"Nav" ~packing:menubar#append () in
  let file_menu = GMenu.menu () in
  let _ = file_menu_item#set_submenu file_menu in

  let factory = new GMenu.factory menubar in

  object (self)
   
(** GUI attributes *)

    val background = background
	
    val frame = GPack.vbox ~height ?width ()
	
    val adj = GData.adjustment 
	~value:1. ~lower:0.05 ~upper:10. 
	~step_incr:0.25 ~page_incr:1.0 ~page_size:1.0 ()
  
    val bottom = GPack.hbox ~height:30 ()
	
    val _w = GEdit.spin_button  ~rate:0. ~digits:2 ~width:50 ~height:20 ()
	
    val mutable lbl_x_axis = GMisc.label ~height:50 ()

	
(** other attributes *)

    val mutable projection = projection	
    val mutable georef = georef
    val mutable dragging = None
    val mutable grouping = None
    val mutable region = None (* Rectangle selected region *)


    method region = region

    method pack = 
      frame#pack menubar#coerce;
      frame#pack ~expand:true canvas#coerce;
      frame#pack bottom#coerce;
      bottom#pack _w#coerce;
      bottom#pack lbl_x_axis#coerce;

(** initialization of instance attributes *)

    initializer (

      self#pack;

      _w#set_adjustment adj;

(** callback bindings *)

      canvas#coerce#misc#modify_bg [`NORMAL, `NAME "black"];
      ignore (canvas#event#connect#motion_notify (self#mouse_motion));
      ignore (canvas#event#connect#button_press (self#button_press));
      ignore (canvas#event#connect#button_release self#button_release);
      ignore (canvas#event#connect#after#key_press self#key_press) ;
      ignore (canvas#event#connect#enter_notify (fun _ -> self#canvas#misc#grab_focus () ; false));
      ignore (canvas#event#connect#any self#any_event);
      ignore (adj#connect#value_changed (fun () -> canvas#set_pixels_per_unit adj#value));

      canvas#set_center_scroll_region false ;
      canvas#set_scroll_region (-2500000.) (-2500000.) 2500000. 2500000.;

     )
 


(** methods *)

    method set_lbl_x_axis = fun s -> lbl_x_axis#set_text s
	
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
	  try
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
	  with
	    _ -> (0., 0.) (** Don't want to break everything with bad coordinates *)
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
	
    method zoom = fun value ->
      adj#set_value value
  	  
	(** Mouse button events *******************************************)
     method button_press = fun ev ->
      let xc = GdkEvent.Button.x ev in
      let yc = GdkEvent.Button.y ev in  
      match GdkEvent.Button.button ev with
	1 when Gdk.Convert.test_modifier `SHIFT (GdkEvent.Button.state ev) && not (Gdk.Convert.test_modifier `CONTROL (GdkEvent.Button.state ev)) ->
	  let (x1,y1) = self#window_to_world xc yc in
	  grouping <- Some (x1,y1);
	  region_rectangle#set [`X1 x1; `Y1 y1; `X2 x1; `Y2 y1];
	  true
      | 2 when Gdk.Convert.test_modifier `SHIFT (GdkEvent.Button.state ev) ->
	  dragging <- Some (xc, yc);
	  true
      |	_ -> false
	    
    method mouse_motion = fun ev ->
      if georef <> None then begin
	let xc = GdkEvent.Motion.x ev 
	and yc = GdkEvent.Motion.y ev in
	let (xw, yw) = self#window_to_world xc yc in
	self#display_geo (self#geo_string (self#of_world (xw,yw)));
	self#display_alt (self#of_world (xw,yw));
	begin
	  match dragging with
	    Some (x0, y0 ) -> 
	      let (x, y) = self#canvas#get_scroll_offsets in
	      self#canvas#scroll_to (x+truncate (x0-.xc)) (y+truncate (y0-.yc))
	  | None -> ()
	end;
	begin
	  match grouping with
	    Some starting_point ->
	      let starting_point = self#of_world starting_point in
	      let starting_point = LL.utm_of LL.WGS84 starting_point in
	      let current_point = LL.utm_of LL.WGS84 (self#of_world (xw, yw)) in
	      let (east, north) = LL.utm_sub current_point starting_point in
	      region_rectangle#set [`X2 xw; `Y2 yw];
	      self#display_group (sprintf "[%.1fkm %.1fkm]" (east/.1000.) (north/.1000.))
	  | None -> ()
	end
      end;
      false
	  
    method button_release = fun ev ->
      match GdkEvent.Button.button ev, grouping with
	2, _ ->
	  dragging <- None; false
      | 1, Some starting_point ->
	  let xc = GdkEvent.Button.x ev in
	  let yc = GdkEvent.Button.y ev in  
	  let current_point = self#window_to_world xc yc in
	  region <- Some (starting_point, current_point);
	  self#display_group "";
	  grouping <- None;
	  false
      | _ -> false
	    
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
      (** View has changed ? *)
      let width_c, height_c = Gdk.Drawable.get_size canvas#misc#window
      and (xc0, yc0) = canvas#get_scroll_offsets in
      let view = (xc0, yc0, width_c, height_c) in
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
      let p = points in
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

    val mutable lbl_xy = GMisc.label  ~height:50 ()
    val mutable lbl_geo = GMisc.label  ~height:50 ()
    val mutable lbl_alt =  GMisc.label  ~height:50 ()
    val mutable lbl_group = GMisc.label  ~height:50 ()   
    val mutable utm_grid_group = None

    method pack_labels =
      bottom#pack lbl_xy#coerce;
      bottom#pack lbl_geo#coerce;
      bottom#pack lbl_alt#coerce;
      bottom#pack lbl_group#coerce;
      
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
