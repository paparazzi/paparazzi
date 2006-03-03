open Latlong
open Printf
  
let pan_step = 50
    
type meter = float
type en = { east : meter; north : meter }
      
let _ = Srtm.add_path "SRTM"


    
(** basic canvas with menubar **************************************
 * (the vertical display in map2.ml is an instance of basic_widget)*
 *******************************************************************)
    
(* world_unit: m:pixel at scale 1. *)
class basic_widget = fun ?(height=800) ?width ?wgs84_of_en () ->  
  
  object (self)
   
(** GUI attributes *)

    val frame = GPack.vbox ~height ?width ()
	
    val menubar = GMenu.menu_bar ()
	
    val adj = GData.adjustment 
	~value:1. ~lower:0.05 ~upper:10. 
	~step_incr:0.25 ~page_incr:1.0 ~page_size:1.0 ()
  
    val canvas = GnoCanvas.canvas ()
	
    val bottom = GPack.hbox  ~height:30 ()
	
    val _w = GEdit.spin_button  ~rate:0. ~digits:2 ~width:50
	  ~height:20 ()
	
(***)   val mutable factory = new GMenu.factory (GMenu.menu_bar ())
    
    val mutable file_menu =  GMenu.menu ()
    
    val mutable lbl_x_axis = GMisc.label ~height:50 ()
	
(** other attributes *)
	
    val mutable current_zoom = 1.
    val mutable dragging = None
    val mutable grouping = None
    val mutable rectangle = None
    val mutable world_unit = 1.
    val mutable wgs84_of_en = wgs84_of_en
(***)    val mutable background =  GnoCanvas.pixbuf (GnoCanvas.canvas ())#root
    val mutable vertical_factor = 10.0
    val mutable vertical_max_level = 0.0


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
      
      background#destroy ();
      background <- GnoCanvas.pixbuf canvas#root;

     (*** factory#destroy (); ***)

      factory <- new GMenu.factory menubar;

      file_menu#destroy ();

      file_menu <- factory#add_submenu "Nav";

(** callback bindings *)

      canvas#coerce#misc#modify_bg [`NORMAL, `NAME "black"];
      ignore (canvas#event#connect#motion_notify (self#mouse_motion));
      ignore (canvas#event#connect#button_press (self#button_press));
      ignore (canvas#event#connect#button_release self#button_release);
      ignore (canvas#event#connect#after#key_press self#key_press) ;
      ignore (canvas#event#connect#enter_notify (fun _ -> self#canvas#misc#grab_focus () ; false));
      ignore (canvas#event#connect#any self#any_event);
      ignore (adj#connect#value_changed (fun () -> self#zoom adj#value));

      canvas#set_center_scroll_region false ;
      canvas#set_scroll_region (-2500000.) (-2500000.) 2500000. 2500000.;

     )
 


(** methods *)

     
    method set_wgs84_of_en = fun x -> wgs84_of_en <- Some x
	
    method set_world_unit = fun x -> world_unit <- x
	
    method get_world_unit = fun () -> world_unit

    method set_lbl_x_axis = fun s -> lbl_x_axis#set_text s
	
(** accessors to instance variables *)	
    method current_zoom = current_zoom	
    method get_vertical_factor = vertical_factor
    method get_vertical_max_level = vertical_max_level
    method set_vertical_factor = fun x -> vertical_factor <- x
    method set_vertical_max_level = fun x -> vertical_max_level <- x	
    method canvas = canvas
    method frame = frame
    method factory = factory
    method file_menu = file_menu
    method window_to_world = canvas#window_to_world
    method root = canvas#root
    method zoom_adj = adj
	
(** following display functions can be redefined by subclasses.
   they do nothing in the basic_widget *)
    method display_xy = fun s -> ()
    method display_geo = fun s -> ()
    method display_alt = fun en -> ()
    method display_group = fun s -> ()
	
   (** converts relative utm coordinates into world (ie map) coordinates *)
    method world_of_en = fun en -> 
      en.east /. world_unit, -. en.north /. world_unit
    method en_of_world = fun wx wy -> { east = wx *. world_unit;
					north = -. wy *. world_unit } 
    method geo_string = fun en ->
      match wgs84_of_en with
	None -> ""
      | Some f -> string_degrees_of_geographic (f en)

    method wgs84_of_en =
      match wgs84_of_en with
	None -> raise Not_found
      | Some f -> f
	    
    	    
    method moveto = fun en ->
      let (xw, yw) = self#world_of_en en in
      let (xc, yc) = canvas#world_to_window xw yw in
      canvas#scroll_to (truncate xc) (truncate yc)
			
    method display_map = fun ?(scale = 1.) ?(anchor = (`ANCHOR `NW)) en image ->
      background <- GnoCanvas.pixbuf ~pixbuf:image ~props:[anchor] self#root;
      background#lower_to_bottom ();
      let wx, wy = self#world_of_en en in
      background#move wx wy;
      let a = background#i2w_affine in
      a.(0) <- scale; a.(3) <- scale; 
      background#affine_absolute a;
      background
	
    method zoom = fun value ->
      canvas#set_pixels_per_unit value;
      current_zoom <- value
	  
	  
    method mouse_motion = fun ev ->
      let xc = GdkEvent.Motion.x ev 
      and yc = GdkEvent.Motion.y ev in
      let (xw, yw) = self#window_to_world xc yc in
      let en = self#en_of_world xw yw in
      self#display_xy (sprintf "%.0fm %.0fm\t" en.east en.north);
      self#display_geo (self#geo_string en);
      self#display_alt en;
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
	    self#display_group (sprintf "[%.1fkm %.1fkm]" ((en1.east -. en.east)/.1000.) ((en1.north-.en.north)/.1000.))
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
	  self#display_group "";
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
      | 2 when Gdk.Convert.test_modifier `SHIFT (GdkEvent.Button.state ev) ->
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
      | k when k = GdkKeysyms._Page_Up -> adj#set_value (adj#value+.adj#step_increment) ; true
      | k when k = GdkKeysyms._Page_Down -> adj#set_value (adj#value-.adj#step_increment) ; true
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
	    
	    
    method segment = fun ?(group = canvas#root) ?(width=1) ?fill_color en1 en2 ->
      let (x1, y1) = self#world_of_en en1
      and (x2, y2) = self#world_of_en en2 in
      let l = GnoCanvas.line ?fill_color ~props:[`WIDTH_PIXELS width] ~points:[|x1;y1;x2;y2|] group in
      l#show ();
      l
	
    method circle = fun ?(group = canvas#root) ?(width=1) ?fill_color ?(color="black") en rad ->
      let (x, y) = self#world_of_en en in
      let rad = rad /. world_unit in
      let l = GnoCanvas.ellipse ?fill_color ~props:[`WIDTH_PIXELS width; `OUTLINE_COLOR color] ~x1:(x-.rad) ~y1:(y -.rad) ~x2:(x +.rad) ~y2:(y+.rad) group in
      l#show ();
      l
    method text = fun ?(group = canvas#root) ?(fill_color = "blue") ?(x_offset = 0.0) ?(y_offset = 0.0) en1 text ->
      let (x1, y1) = self#world_of_en en1 in
      let t = GnoCanvas.text ~x:x1 ~y:y1 ~text:text ~props:[`FILL_COLOR fill_color; `X_OFFSET x_offset; `Y_OFFSET y_offset] group in
      t#show ();
      t
	
  end
    

(** canvas which inherits from basic_widget  ********************
 * - labels for displaying mouse coordinates on the map         *
 * - background switching                                       *
 *(the horizontal map in map2.ml is an instance of basic_widget)*
 ****************************************************************)

    
class widget =  fun ?(height=800) ?width ?wgs84_of_en () ->
  object(self)
    inherit (basic_widget ~height:height ?width ?wgs84_of_en ())

    val mutable lbl_xy = GMisc.label  ~height:50 ()
    val mutable lbl_geo = GMisc.label  ~height:50 ()
    val mutable lbl_alt =  GMisc.label  ~height:50 ()
    val mutable lbl_group = GMisc.label  ~height:50 ()   
(***)    val mutable menu_fact = new GMenu.factory (GMenu.menu ())
    val mutable srtm = GMenu.check_menu_item ()

    method pack_labels =
      bottom#pack lbl_xy#coerce;
      bottom#pack lbl_geo#coerce;
      bottom#pack lbl_alt#coerce;
      bottom#pack lbl_group#coerce;
      
    initializer (
      self#pack_labels;
     (*** menu_fact#destroy (); ***)
      menu_fact <- new GMenu.factory file_menu;
      srtm#destroy ();
      srtm <- menu_fact#add_check_item "SRTM" ~active:false;
      ignore (menu_fact#add_check_item "Background" ~active:true ~callback:self#switch_background);
      ignore (menu_fact#add_item "Goto" ~callback:self#goto);
     )

    method menu_fact = menu_fact
	
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
    method display_alt = fun en ->
      begin
	 match wgs84_of_en, srtm#active with
	  Some wgs84_of_en, true ->
	    lbl_alt#set_text (sprintf "\t%dm"(self#altitude (wgs84_of_en en)))
	| _ -> ()
       end
	
    method display_group = fun s ->  lbl_group#set_text s
	

    method switch_background = fun x -> if x then background#show () else background#hide ()

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
	
end
