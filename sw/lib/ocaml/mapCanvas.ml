open Latlong
open Printf

let pan_step = 50

type meter = float
type en = { east : meter; north : meter }

let _ = Srtm.add_path "SRTM"

(* world_unit: m:pixel at scale 1. *)
class widget = fun  ?(height=800) ?width ?wgs84_of_en () ->

  let frame = GPack.vbox ~height ?width () in

  let menubar = GMenu.menu_bar ~packing:frame#pack () in

  let adj = GData.adjustment 
      ~value:1. ~lower:0.25 ~upper:10. 
      ~step_incr:0.25 ~page_incr:1.0 ~page_size:1.0 () in

  let canvas = GnoCanvas.canvas ~height ~packing:(frame#pack ~expand:true) () in

  let bottom = GPack.hbox ~height:30 ~packing:frame#pack () in
  let _w = GEdit.spin_button ~adjustment:adj ~rate:0. ~digits:2 ~width:50 ~height:20 ~packing:bottom#pack () in


  let lbl_xy = GMisc.label ~height:50 ~packing:bottom#pack () in
  let lbl_geo = GMisc.label ~height:50 ~packing:bottom#pack () in
  let lbl_alt = GMisc.label ~height:50 ~packing:bottom#pack () in
  let lbl_group = GMisc.label ~height:50 ~packing:bottom#pack () in

  let factory = new GMenu.factory menubar in
  let file_menu = factory#add_submenu "Nav" in
  let menu_fact = new GMenu.factory file_menu in

  let srtm = menu_fact#add_check_item "SRTM" ~active:false in

 object (self)

   initializer ignore (menu_fact#add_item "Goto" ~callback:self#goto)
  initializer ignore (canvas#event#connect#motion_notify (self#mouse_motion));
  initializer ignore (canvas#event#connect#button_press (self#button_press));
  initializer ignore (canvas#event#connect#button_release self#button_release);
  initializer ignore (canvas#event#connect#after#key_press self#key_press) ;
  initializer ignore (canvas#event#connect#enter_notify (fun _ -> self#canvas#misc#grab_focus () ; false));
  initializer ignore (canvas#event#connect#any self#any_event);

  initializer ignore (adj#connect#value_changed (fun () -> self#zoom adj#value));
  initializer canvas#set_center_scroll_region false ;
  initializer canvas#set_scroll_region (-2500000.) (-2500000.) 2500000. 2500000.;


  val mutable current_zoom = 1.
  val mutable dragging = None
  val mutable grouping = None
   val mutable rectangle = None
   val mutable world_unit = 1.
   val mutable wgs84_of_en = wgs84_of_en

   method set_wgs84_of_en = fun x -> wgs84_of_en <- Some x

   method set_world_unit = fun x -> world_unit <- x

  method current_zoom = current_zoom

  method canvas = canvas
  method frame = frame
  method menu_fact = menu_fact
  method window_to_world = canvas#window_to_world
  method root = canvas#root
   method zoom_adj = adj
   method factory = factory
     

  method world_of_en = fun en -> en.east /. world_unit, -. en.north /. world_unit
  method en_of_world = fun wx wy -> { east = wx *. world_unit; north = -. wy *. world_unit } 

  method geo_string = fun en ->
    match wgs84_of_en with
      None -> ""
    | Some f -> string_degrees_of_geographic (f en)

   method altitude = fun wgs84 ->
     try
       Srtm.of_wgs84 wgs84
     with
       Srtm.Tile_not_found x ->
	 srtm#set_active false;
	 GToolbox.message_box "SRTM" (sprintf "SRTM tile %s not found: %s ?" x (Srtm.error x));
	 0

  method moveto = fun en ->
    let (xw, yw) = self#world_of_en en in
    let (xc, yc) = canvas#world_to_window xw yw in
    canvas#scroll_to (truncate xc) (truncate yc)

  method goto = fun () ->
    let dialog = GWindow.window ~border_width:10 ~title:"Geo ref" () in
    let dvbx = GPack.box `VERTICAL ~packing:dialog#add () in
    let lat  = GEdit.entry ~packing:dvbx#add () in
    let lon  = GEdit.entry ~packing:dvbx#add () in
    let cancel = GButton.button ~label:"Cancel" ~packing: dvbx#add () in 
    let ok = GButton.button ~label:"OK" ~packing: dvbx#add () in
    ignore(cancel#connect#clicked ~callback:dialog#destroy);
    ignore(ok#connect#clicked ~callback:
	     begin fun _ ->
	       let x = float_of_string lat#text in
	       let y = float_of_string lon#text in
	       self#moveto {east=x; north=y};
               dialog#destroy ()
	     end);
    dialog#show ()


  method display_map = fun ?(scale = 1.) en image ->
    let p = GnoCanvas.pixbuf ~pixbuf:image ~props:[`ANCHOR `NW] self#root in
    p#lower_to_bottom ();
    let wx, wy = self#world_of_en en in
    p#move wx wy;
    let a = p#i2w_affine in
    a.(0) <- scale; a.(3) <- scale; 
    p#affine_absolute a;
    p

  method zoom = fun value ->
    canvas#set_pixels_per_unit value;
    current_zoom <- value


  method mouse_motion = fun ev ->
    let xc = GdkEvent.Motion.x ev 
    and yc = GdkEvent.Motion.y ev in
    let (xw, yw) = self#window_to_world xc yc in
    let en = self#en_of_world xw yw in
    lbl_xy#set_text (sprintf "%.0fm %.0fm\t" en.east en.north);
    lbl_geo#set_text (self#geo_string en);
    begin
      match wgs84_of_en, srtm#active with
	Some wgs84_of_en, true ->
	  lbl_alt#set_text (sprintf "\t%dm"(self#altitude (wgs84_of_en en)))
      | _ -> ()
    end;
    begin
      match dragging with
      Some (x0, y0 ) -> 
	let (x, y) = self#canvas#get_scroll_offsets in
	self#canvas#scroll_to (x+truncate (x0-.xc)) (y+truncate (y0-.yc))
      | None -> ()
    end;
    begin
      match grouping with
      Some (xw1, yw1) -> 
	let en1 = self#en_of_world xw1 yw1 in
	lbl_group#set_text (sprintf "[%.1fkm %.1fkm]" ((en1.east -. en.east)/.1000.) ((en1.north-.en.north)/.1000.))
      | None -> ()
    end;
    false

  method button_release = fun ev ->
    match GdkEvent.Button.button ev, grouping with
      2, _ ->
	dragging <- None; false
    | 1, Some (xw1, yw1) ->
	let xc = GdkEvent.Button.x ev in
	let yc = GdkEvent.Button.y ev in  
	let (xw2, yw2) = self#window_to_world xc yc in
	rectangle <- Some ((xw1, yw1), (xw2, yw2));
	lbl_group#set_text "";
	grouping <- None;
	false
    | _ -> false

  method button_press = fun ev ->
    let xc = GdkEvent.Button.x ev in
    let yc = GdkEvent.Button.y ev in  
    match GdkEvent.Button.button ev with
      1 ->
	let xyw = self#window_to_world xc yc in
	grouping <- Some xyw;
	true
    | 2 ->
	dragging <- Some (xc, yc);
	true
    |	_ -> false
	
  method key_press = fun ev ->
    let (x, y) = canvas#get_scroll_offsets in
    match GdkEvent.Key.keyval ev with
    | k when k = GdkKeysyms._Up -> canvas#scroll_to x (y-pan_step) ; true
    | k when k = GdkKeysyms._Down -> canvas#scroll_to x (y+pan_step) ; true
    | k when k = GdkKeysyms._Left -> canvas#scroll_to (x-pan_step) y ; true
    | k when k = GdkKeysyms._Right -> canvas#scroll_to (x+pan_step) y ; true
    | _ -> false
	  
  method any_event = fun ev ->
    match GdkEvent.get_type ev with
    | `SCROLL -> begin
	match GdkEvent.Scroll.direction (GdkEvent.Scroll.cast ev) with
	  `UP    ->
	    adj#set_value (adj#value+.adj#step_increment) ;
	    true
	| `DOWN  -> adj#set_value (adj#value-.adj#step_increment) ; true
	| _  -> false
    end
    | _ -> false
	  

   method segment = fun ?(width=1) ?fill_color en1 en2 ->
     let (x1, y1) = self#world_of_en en1
     and (x2, y2) = self#world_of_en en2 in
     let l = GnoCanvas.line ?fill_color ~props:[`WIDTH_PIXELS width] ~points:[|x1;y1;x2;y2|] canvas#root in
     l#show ();
     l
end

