(* ocamlc -I +lablgtk2 lablgtk.cma lablgnomecanvas.cma gtkInit.cmo medit.ml *)

open Printf
open Latlong

let sof = string_of_float
let fos = float_of_string
let sof1 = fun x -> Printf.sprintf "%.1f" x

(* World (0., 0.) seems to be at the center of the canvas *)

type meter = float
type en = { east : meter; north : meter }

let flight_plan = ref (Xml.PCData "")

module Ref = struct
  let world_unit = 2.5 (* 1 pixel = 2.5m *)
  let ref0 = ref {posn_lat = 43.237535; posn_long = 1.327747 }
  let utm0 () = utm_of WGS84 !ref0
      
  let set = fun lat lon ->
    ref0 := { posn_lat = (Deg>>Rad) lat; posn_long = (Deg>>Rad) lon }


  let world_of_en = fun en -> en.east /. world_unit, -. en.north /. world_unit
  let en_of_world = fun wx wy -> { east = wx *. world_unit; north = -. wy *. world_unit } 
  let world_of_utm = fun utm ->
    let utm0 = utm0 () in
    let x = utm.utm_x -. utm0.utm_x
    and y = -. (utm.utm_y -. utm0.utm_y) in
    (x /. world_unit, y /. world_unit)


  let geo_string = fun en ->
    let u = utm_of WGS84 !ref0 in
    let u' = {utm_x = u.utm_x +. en.east;
	      utm_y = u.utm_y +. en.north;
	      utm_zone = u.utm_zone } in
    let w = of_utm WGS84 u' in
    sprintf "%.4f %.4f" ((Rad>>Deg)w.posn_lat) ((Rad>>Deg)w.posn_long)

  let direction = fun w1 w2 ->
    let u1 = utm_of WGS84 w1
    and u2 = utm_of WGS84 w2 in
    assert (u1.utm_zone = u2.utm_zone);
    (u2.utm_x -. u1.utm_x, u2.utm_y -. u1.utm_y)
      
  let world_of_wgs84 = fun wgs84 ->
    let (dxm, dym) = direction !ref0 wgs84 in
    (dxm/. world_unit, -. dym/.world_unit)
end

let gensym = let n = ref 0 in fun prefix -> incr n; prefix ^ string_of_int !n


class type w = object
  method alt : float
  method name : string
  method en : en
  method zoom : float -> unit
  method delete : unit
end


module Waypoints = struct
  let waypoints = Hashtbl.create 13
  let add = fun w -> Hashtbl.add waypoints (w:>w) (w:>w)
  let remove = fun x -> Hashtbl.remove waypoints x
  let iter = fun f -> Hashtbl.iter f waypoints
  let clear = fun () -> Hashtbl.iter (fun w _ -> w#delete) waypoints
end

module RecentFiles = struct
  let conf_file = Filename.concat (Sys.getenv "HOME") ".meditrc"
  type f = Map of string | FP of string
  let l = ref []
  let add = fun f ->
    if not (List.mem f !l) then
      l := f :: !l
  let f_of_xml = fun x ->
    let f = Xml.attrib x "file" in
    match Xml.tag x with
      "map" -> Map f
    | "fp" -> FP f
    | _ -> failwith "RecentFile.f_of_xml"
  let xml_of_f = function
      Map f -> Xml.Element ("map", ["file",f], [])
    | FP f -> Xml.Element ("fp", ["file",f], [])    
  let load = fun () ->
    let xml = try Xml.parse_file conf_file with _ -> Xml.Element ("",[],[]) in
    l := List.map f_of_xml (Xml.children xml)
  let save = fun () ->
    let f = open_out conf_file in
    let xml = Xml.Element ("files", [], List.map xml_of_f !l) in
    output_string f (Xml.to_string_fmt xml);
    close_out f
  let iter = fun f -> List.iter f !l
end

let s = 5.
let losange = [|s;0.; 0.;s; -.s;0.; 0.;-.s|]

let georef = fun () ->
  let dialog = GWindow.window ~border_width:10 ~title:"Geo ref" () in
  let dvbx = GPack.box `VERTICAL ~packing:dialog#add () in
  let lat  = GEdit.entry ~text:"43.210" ~packing:dvbx#add () in
  let lon  = GEdit.entry ~text:"1.234" ~packing:dvbx#add () in
  let cancel = GButton.button ~label:"Cancel" ~packing: dvbx#add () in 
  let ok = GButton.button ~label:"OK" ~packing: dvbx#add () in
  ignore(cancel#connect#clicked ~callback:dialog#destroy);
  ignore(ok#connect#clicked ~callback:
      begin fun _ ->
	let lat = float_of_string lat#text in
	let lon = float_of_string lon#text in
	Ref.set lat lon;
        dialog#destroy ()
      end);
  dialog#show ()

let current_zoom = ref 1. (* Would be better not to be global ??? *)

    
class waypoint = fun root (name :string) ?(alt=0.) en ->
  let xw, yw = Ref.world_of_en en in
  object (self)
    val mutable x0 = 0.
    val mutable y0 = 0.
    val item = 
	GnoCanvas.polygon root ~points:losange
	  ~props:[`FILL_COLOR "red" ; `OUTLINE_COLOR "midnightblue" ; `WIDTH_UNITS 1.; `FILL_STIPPLE (Gdk.Bitmap.create_from_data ~width:2 ~height:2 "\002\001")]
	  
    val label = GnoCanvas.text root ~props:[`TEXT name; `X s; `Y 0.; `ANCHOR `SW]
    val mutable name = name
    val mutable alt = alt
    initializer self#move xw yw
    method name = name
    method set_name n =
      if n <> name then
	name <- n
    method alt = alt
    method label = label
    method xy = let a = item#i2w_affine in (a.(4), a.(5)) (*** item#i2w 0. 0. causes Seg Fault !***)
    method move dx dy = item#move dx dy; label#move dx dy
    method edit =
      let dialog = GWindow.window ~border_width:10 ~title:"Waypoint Edit" () in
      let dvbx = GPack.box `VERTICAL ~packing:dialog#add () in
      let en = self#en in
      let ename  = GEdit.entry ~text:name ~packing:dvbx#add () in
      let ex  = GEdit.entry ~text:(string_of_float en.east) ~packing:dvbx#add () in
      let ey  = GEdit.entry ~text:(string_of_float en.north) ~packing:dvbx#add () in
      let ea  = GEdit.entry ~text:(string_of_float alt) ~packing:dvbx#add () in
      let cancel = GButton.button ~label:"Cancel" ~packing: dvbx#add () in 
      let ok = GButton.button ~label:"OK" ~packing: dvbx#add () in
      ignore(cancel#connect#clicked ~callback:dialog#destroy);
      ignore(ok#connect#clicked ~callback:
	       begin fun _ ->
		 self#set_name ename#text;
		 alt <- float_of_string ea#text;
		 label#set [`TEXT name];
		 self#set {east = float_of_string ex#text;
			   north = float_of_string ey#text};
		 dialog#destroy ()
	       end);
      dialog#show ()



    method event (ev : GnoCanvas.item_event) =
      begin
	match ev with
	| `BUTTON_PRESS ev ->
	    begin
	      match GdkEvent.Button.button ev with
	      |	1 -> self#edit
	      |	3 -> self#delete
	      | 2 ->
		  let x = GdkEvent.Button.x ev
		  and y = GdkEvent.Button.y ev in
		  x0 <- x; y0 <- y;
		  let curs = Gdk.Cursor.create `FLEUR in
		  item#grab [`POINTER_MOTION; `BUTTON_RELEASE] curs 
		    (GdkEvent.Button.time ev)
	      | x ->   printf "%d\n" x; flush stdout;
	    end
	| `MOTION_NOTIFY ev ->
	    let state = GdkEvent.Motion.state ev in
	    if Gdk.Convert.test_modifier `BUTTON2 state then begin
	      let x = GdkEvent.Motion.x ev
	      and y = GdkEvent.Motion.y ev in
	      let dx = !current_zoom *. (x-. x0) 
	      and dy = !current_zoom *. (y -. y0) in
	      self#move dx dy ;
	      x0 <- x; y0 <- y
	    end
	| `BUTTON_RELEASE ev ->
	    if GdkEvent.Button.button ev = 2 then
	    item#ungrab (GdkEvent.Button.time ev)
	| _ -> ()
      end;
      true
   initializer ignore(item#connect#event self#event)
    method item = item
    method en =
      let (dx, dy) = self#xy in
      Ref.en_of_world dx dy
    method set en = 
      let (xw, yw) = Ref.world_of_en en
      and (xw0, yw0) = self#xy in
      self#move (xw-.xw0) (yw-.yw0)      
    method delete =
      item#destroy ();
      label#destroy ();
      Waypoints.remove (self:>w)
    method zoom (z:float) =
      let a = item#i2w_affine in
      a.(0) <- 1./.z; a.(3) <- 1./.z; 
      item#affine_absolute a;
      label#affine_absolute a
  end

let xml_of_wp = fun utm0 w ->
  let wgs84 = of_utm WGS84 { utm_x = utm0.utm_x +. w#en.east; utm_y = utm0.utm_y +. w#en.north ; utm_zone = utm0.utm_zone } in
  let alt = if w#alt = 0. then [] else ["alt", string_of_float  w#alt] in
  Xml.Element ("waypoint", ["name",w#name; 
			    "x",sof1 w#en.east; 
			    "y", sof1 w#en.north;
			    "lat",string_of_float ((Rad>>Deg) wgs84.posn_lat);
			    "lon",string_of_float ((Rad>>Deg) wgs84.posn_long)]@alt
			    ,[])

let subst_waypoints = fun xml wpts ->
  match xml with
    Xml.Element (tag, attrs, children) ->
      Xml.Element (tag, attrs, List.map (fun c -> if Xml.tag c = "waypoints" then wpts else c) children)
  | _ -> failwith "subst_waypoints"


    
let file_dialog ~title ~callback () =
  let sel = GWindow.file_selection ~title ~filename:"*.xml" ~modal:true () in
  ignore (sel#cancel_button#connect#clicked ~callback:sel#destroy);
  ignore
    (sel#ok_button#connect#clicked
       ~callback:(fun () ->
	 let name = sel#filename in
	 sel#destroy ();
	 callback name));
  sel#show ()
  

let write_mission = fun () ->
  let l = ref [] in
  Waypoints.iter (fun _ w -> l := w :: !l);
  let utm0 = (Ref.utm0 ()) in
  let children = List.map (xml_of_wp utm0) !l in
  let waypoints = Xml.Element ("waypoints", ["utm_x0", sof1 utm0.utm_x;"utm_y0", sof1 utm0.utm_y], children) in
  let fp = subst_waypoints !flight_plan waypoints in

  ignore (file_dialog ~title:"Save Flight Plan" ~callback:(fun name -> let f  = open_out name in
  fprintf f "%s\n" (Xml.to_string_fmt fp);
  fprintf f "\n";
  close_out f ) ())

let float_attrib = fun x a -> float_of_string (Xml.attrib x a)
let int_attrib = fun x a -> truncate (float_attrib x a)

let load_mission = fun root file ->
  let xml = Xml.parse_file file in

  let default_alt = float_attrib xml "alt" in

  Ref.set (float_attrib xml "lat0") (float_attrib xml "lon0");
  let utm0 = Ref.utm0 () in
  let wps = ExtXml.child xml "waypoints" in
  
  let wp_of_xml = fun xml ->
    let a = try float_attrib xml "alt" with _ -> default_alt in
    let en =
      try
	let lat = float_attrib xml "lat"
	and lon = float_attrib xml "lon" in
	let utm = utm_of WGS84 {posn_lat=(Deg>>Rad)lat; posn_long=(Deg>>Rad)lon} in
	{east = (utm.utm_x -. utm0.utm_x); north = (utm.utm_y -. utm0.utm_y) }
      with
	Xml.No_attribute _ ->
	  { east= float_attrib xml "x"; north= float_attrib xml "y" } in
    let w = new waypoint root (Xml.attrib xml "name") ~alt:a en in
    w#zoom !current_zoom;
    w in
  
  List.iter (fun w -> Waypoints.add (w:>w)) (List.map wp_of_xml (Xml.children wps));
  
  flight_plan := xml


let open_mission = fun cont root ?file () ->
  match file with
    None ->
      ignore (file_dialog ~title:"Open Flight Plan" ~callback:(fun name -> load_mission root name; cont (RecentFiles.FP name)) ())
  | Some name -> load_mission root name; cont (RecentFiles.FP name)


let display_map = fun ?(scale = 1.) x y wgs84 map_name root ->
  let image = GdkPixbuf.from_file map_name in
  let p = GnoCanvas.pixbuf ~pixbuf:image ~props:[`ANCHOR `NW] root in
  p#lower_to_bottom ();
  let wx, wy = Ref.world_of_wgs84 wgs84 in
  p#move  (wx -. x*.scale) (wy -. y*.scale);
  let a = p#i2w_affine in
  a.(0) <- scale; a.(3) <- scale; 
  p#affine_absolute a;
  p

let load_map = fun root (maps_menu_fact:GMenu.menu GMenu.factory) ref0 filename ->
  let register = fun pixbuf ->
    let mi = maps_menu_fact#add_item filename in
    ignore (mi#connect#activate (fun () -> pixbuf#destroy (); maps_menu_fact#menu#remove mi)) in
  if Filename.check_suffix filename ".xml" then
    let xml = Xml.parse_file filename in
    let map_name = Filename.concat (Filename.dirname filename) (Xml.attrib xml "file") in
 
    match Xml.attrib xml "projection" with
      "UTM" ->
	let utm_zone = try int_of_string (Xml.attrib xml "utm_zone") with _ -> fprintf stderr "Warning: utm_zone attribute not specified in '%s'; default is 31\n" filename; flush stderr; 31 in
	begin
	  match Xml.children xml with
	    p::_ ->
	      let utm_x = float_attrib p "utm_x"
	      and utm_y = float_attrib p "utm_y"
	      and x = float_attrib p "x"
	      and y = float_attrib p "y"
	      and scale = float_attrib xml "scale" /. Ref.world_unit in
	      let wgs84 = of_utm WGS84 {utm_x = utm_x; utm_y = utm_y; utm_zone = utm_zone} in
	      register (display_map ~scale x y wgs84 map_name root)
	  | _ -> failwith "Exactly one ref point please"
	end
    | _ -> failwith "Unknwown projection"
  else
    try
      Scanf.sscanf (Filename.basename filename) "F%3d_%3d" (fun x y ->
	let lbt0 = lambert_of lambertIIe ((NTF<<WGS84)ref0) in
	let x0 = float (lbt0.lbt_x - x * 10000) /. 2.5
	and y0 = -. float (lbt0.lbt_y - (268 - y) * 10000) /. 2.5 in
	register (display_map x0 y0 ref0 filename root))
    with
      _ -> failwith "XML or IGN tile expected"
    
  


let open_map = fun cont root maps_menu ?file () ->
  match file with
    None ->
      file_dialog ~title:"Open Map" ~callback:(fun name -> ignore (load_map root maps_menu !Ref.ref0 name); cont (RecentFiles.Map name)) ()
  | Some name ->
      ignore (load_map root maps_menu !Ref.ref0 name);
      cont (RecentFiles.Map name)



let create_wp = fun canvas xw yw () ->
  let en = Ref.en_of_world xw yw in
  let name = gensym "wp" in
  let x = new waypoint canvas#root name en in
  x#zoom !current_zoom;
  Waypoints.add x

let dragging = ref None


let canvas_button_release = fun (canvas:GnoCanvas.canvas) ev ->
  let state = GdkEvent.Button.state ev in
  if GdkEvent.Button.button ev = 2 then begin
    dragging := None;
    true
  end else
    false

let canvas_button_press = fun (canvas:GnoCanvas.canvas) ev ->
  let state = GdkEvent.Button.state ev in
  if GdkEvent.Button.button ev = 1 && Gdk.Convert.test_modifier `CONTROL state then
    let xc = GdkEvent.Button.x ev in
    let yc = GdkEvent.Button.y ev in
    let (xw, yw) = canvas#window_to_world xc yc in
    (* Effective creation delayed to avoid a bug in lablgtk *)
    ignore (GMain.Timeout.add 10 (fun () -> create_wp canvas xw yw (); false));
    true
  else if GdkEvent.Button.button ev = 2 then
    let xc = GdkEvent.Button.x ev in
    let yc = GdkEvent.Button.y ev in
    dragging := Some (xc, yc);
    true
  else begin
    false
  end

let canvas_key_press = fun (canvas:GnoCanvas.canvas) ev ->
  let (x, y) = canvas#get_scroll_offsets in
  match GdkEvent.Key.keyval ev with
  | k when k = GdkKeysyms._Up -> canvas#scroll_to x (y-20) ; true
  | k when k = GdkKeysyms._Down -> canvas#scroll_to x (y+20) ; true
  | k when k = GdkKeysyms._Left -> canvas#scroll_to (x-10) y ; true
  | k when k = GdkKeysyms._Right -> canvas#scroll_to (x+10) y ; true
  | _ -> false

let display_coord = fun (canvas:GnoCanvas.canvas) lbl_xy lbl_geo ev ->
  let xc = GdkEvent.Motion.x ev 
  and yc = GdkEvent.Motion.y ev in
  let (xw, yw) = canvas#window_to_world xc yc in
  let en = Ref.en_of_world xw yw in
  let d = sqrt (en.east*.en.east +. en.north*.en.north) in
  lbl_xy#set_text (sprintf "%.0fm %.0fm (d=%.0fm)\t" en.east en.north d);
  lbl_geo#set_text (Ref.geo_string en);
  begin
    match !dragging with
      Some (x0, y0 ) -> 
	let xc = GdkEvent.Motion.x ev in
	let yc = GdkEvent.Motion.y ev in
	let (x, y) = canvas#get_scroll_offsets in
	canvas#scroll_to (x+truncate (x0-.xc)) (y+truncate (y0-.yc))
    | None -> ()
  end;
  false


let main () =
  let window = GWindow.dialog ~title: "Paparazzi"
      ~border_width: 1 ~width:800 () in
  let quit = fun () ->
    RecentFiles.save ();
    GMain.Main.quit (); exit 0 in
  ignore (window#connect#destroy ~callback:quit);

  let menubar = GMenu.menu_bar ~packing:window#vbox#pack () in

 let adj = GData.adjustment 
      ~value:1. ~lower:0.05 ~upper:10. 
    ~step_incr:0.5 ~page_incr:1.0 ~page_size:1.0 () in

  let w = GEdit.spin_button ~adjustment:adj ~rate:0. ~digits:2 ~width:50 ~packing:window#vbox#add () in

  let frame = GBin.frame ~shadow_type:`IN ~height:500 ~width:700 ~packing:window#vbox#add () in
  let canvas = GnoCanvas.canvas ~packing:frame#add () in

  canvas#set_center_scroll_region false ;
  canvas#set_scroll_region (-2500.) (-2500.) 2500. 2500.;

  let zoom = fun value ->
    canvas#set_pixels_per_unit value;
    Waypoints.iter (fun _ w -> w#zoom value);
    current_zoom := value
  in

  ignore (adj#connect#value_changed (fun () -> zoom adj#value));

  let root = canvas#root in

(***   ignore (canvas#event#connect#button_press (canvas_button_press canvas)); ***)

  let factory = new GMenu.factory menubar in
  let accel_group = factory#accel_group in
  let file_menu = factory#add_submenu "File"
  and insert_menu = factory#add_submenu "Insert"
  and maps_menu = factory#add_submenu "Maps" in
  let file_menu_fact = new GMenu.factory file_menu ~accel_group
  and maps_menu_fact = new GMenu.factory maps_menu ~accel_group
  and insert_menu_fact = new GMenu.factory insert_menu ~accel_group in

  let register_recent_file = fun f ->
    RecentFiles.add f;
    match f with
      RecentFiles.Map f ->
	ignore (file_menu_fact#add_item f ~callback:(open_map (fun _ -> ()) root maps_menu_fact ~file:f))
    | RecentFiles.FP f ->
	ignore (file_menu_fact#add_item f ~callback:(open_mission (fun _ -> ()) root ~file:f)) in

  ignore (file_menu_fact#add_item "Open Flight Plan" ~key:GdkKeysyms._O ~callback:(open_mission register_recent_file root));    
  ignore (file_menu_fact#add_item "Open Map" ~key:GdkKeysyms._M ~callback:(open_map register_recent_file root maps_menu_fact));    
  ignore (file_menu_fact#add_item "Write Flight Plan" ~key:GdkKeysyms._S ~callback:write_mission);    
  ignore (file_menu_fact#add_item "Manual Ref" ~callback:georef);    
  ignore (file_menu_fact#add_item "Clear Waypoints" ~key:GdkKeysyms._C ~callback:Waypoints.clear);    
  ignore (file_menu_fact#add_item "Quit" ~key:GdkKeysyms._Q ~callback:quit);
  ignore (file_menu_fact#add_separator ());

  RecentFiles.load ();
  
  RecentFiles.iter register_recent_file;

  ignore (insert_menu_fact#add_item "Waypoint" ~key:GdkKeysyms._W ~callback:(create_wp canvas 0. 0.));


  let bottom = GPack.hbox ~packing:window#vbox#add () in
  let lbl_xy = GMisc.label ~packing:bottom#pack () in
  let lbl_geo = GMisc.label ~packing:bottom#pack () in

  ignore (canvas#event#connect#motion_notify (display_coord canvas lbl_xy lbl_geo));
  ignore (canvas#event#connect#button_press (canvas_button_press canvas));
  ignore (canvas#event#connect#button_release (canvas_button_release canvas));
  ignore (canvas#event#connect#after#key_press (canvas_key_press canvas)) ;
  ignore (canvas#event#connect#enter_notify (fun _ -> canvas#misc#grab_focus () ; false));

  window#add_accel_group accel_group;
  window#show ();

  let port = ref 2010 
  and domain = ref "127.255.255.255" in
  Arg.parse 
    [ "-b", Arg.Int (fun x -> port := x), "<Port number>\tDefault is 2010, unused if IVYBUS is set";
      "-domain", Arg.String (fun x -> domain := x), "<Network address>\tDefault is 127.255.255.255, unused if IVYBUS is set"]
    (fun x -> prerr_endline ("Don't do anything with "^x))
    "Usage: ";

  let bus =
    try Sys.getenv "IVYBUS" with
      Not_found -> Printf.sprintf "%s:%d" !domain !port in
  Ivy.init "medit" "READY" (fun _ _ -> ());
  Ivy.start bus;

  let plot_utm =
    let last_plot = ref None in
    fun (utm_x:float) (utm_y:float) ->
      let utm_zone = (Ref.utm0 ()).utm_zone in
      let (x, y) = Ref.world_of_utm {utm_x = utm_x; utm_y = utm_y; utm_zone = utm_zone } in
      (match !last_plot with
	None -> ()
      | Some (x', y') ->
	  ignore (GnoCanvas.line ~points:[|x;y;x';y'|] root));
      last_plot := Some (x,y)
  in

  ignore (Ivy.bind (fun _ args -> plot_utm (fos args.(0)/.100.) (fos args.(1)/.100.)) "GPS +[0-9]* +([0-9]*) +([0-9]*)");

  GMain.Main.main ()

let _ = main ()
    
