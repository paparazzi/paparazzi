(* ocamlc -I +lablgtk2 lablgtk.cma lablgnomecanvas.cma gtkInit.cmo medit.ml *)

open Printf
open Latlong

let path = ref ["."]

let ign = ref false

let find_file = fun file ->
  let rec loop = function
      [] -> raise Not_found
    | p::ps ->
	let f = Filename.concat p file in
	if Sys.file_exists f then f else loop ps in
  loop !path


let sof = string_of_float
let soi = string_of_int
let fos = float_of_string
let ios = int_of_string

let home = try Sys.getenv "PAPARAZZI_HOME" with Not_found -> prerr_endline "Please set the PAPARAZZI_HOME variable to the top directory"; exit 1
let (//) = Filename.concat
let default_path_flightplans = home // "conf" // "flight_plans" // ""
let default_flag_flightplans = default_path_flightplans // "*.xml"
let default_path_maps = home // "data" // "maps" // ""
let default_flag_maps = default_path_maps // "*.xml"

let file_dialog ?(filename="*.xml") ~title ~callback () =
  let sel = GWindow.file_selection ~title ~filename ~modal:true () in
  ignore (sel#cancel_button#connect#clicked ~callback:sel#destroy);
  ignore
    (sel#ok_button#connect#clicked
       ~callback:(fun () ->
	 let name = sel#filename in
	 sel#destroy ();
	 callback name));
  sel#show ()

(* World (0., 0.) seems to be at the center of the canvas *)

type meter = float
type en = { east : meter; north : meter }

let tree_view = ref None

module Ref = struct
  let world_unit = 2.5 (* 1 pixel = 2.5m *)
  let ref0 = ref None
  let get_ref0 = fun () ->
    match !ref0 with
      Some r -> r
    | None -> failwith "Ref.ret_ref0"
  let utm0 () = utm_of WGS84 (get_ref0 ())
      
  let set = fun lat lon ->
    ref0 := Some { posn_lat = (Deg>>Rad) lat; posn_long = (Deg>>Rad) lon }


  let world_of_en = fun en -> en.east /. world_unit, -. en.north /. world_unit
  let en_of_world = fun wx wy -> { east = wx *. world_unit; north = -. wy *. world_unit } 
  let world_of_utm = fun utm ->
    let utm0 = utm0 () in
    let x = utm.utm_x -. utm0.utm_x
    and y = -. (utm.utm_y -. utm0.utm_y) in
    (x /. world_unit, y /. world_unit)


  let geo_string = fun en ->
    let u = utm_of WGS84 (get_ref0 ()) in
    let u' = {utm_x = u.utm_x +. en.east;
	      utm_y = u.utm_y +. en.north;
	      utm_zone = u.utm_zone } in
    let w = of_utm WGS84 u' in
    sprintf "%.4f %.4f" ((Rad>>Deg)w.posn_lat) ((Rad>>Deg)w.posn_long)

  exception WrongUTM
  let direction = fun w1 w2 ->
    let u1 = utm_of WGS84 w1
    and u2 = utm_of WGS84 w2 in
    if u1.utm_zone = u2.utm_zone then
      (u2.utm_x -. u1.utm_x, u2.utm_y -. u1.utm_y)
    else
      raise WrongUTM
      
  let world_of_wgs84 = fun wgs84 ->
    let (dxm, dym) = direction (get_ref0 ()) wgs84 in
    (dxm/. world_unit, -. dym/.world_unit)
end

let gensym = let n = ref 0 in fun prefix -> incr n; prefix ^ string_of_int !n


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

let labelled_entry = fun text value h ->
  let _ = GMisc.label ~text ~packing:h#add () in
  GEdit.entry ~text:value ~packing:h#add ()

let save_fp = fun fp ->
  file_dialog 
    ~title:"Save Flight Plan" 
    ~callback:(fun name ->
      let f  = open_out name in
      fprintf f "%s\n" (Xml.to_string_fmt fp);
      fprintf f "\n";
      close_out f)
    ()

let (//) = Filename.concat
let pararazzi_home = Filename.dirname Sys.argv.(0) // ".." // ".." 
let example_file = pararazzi_home // "conf" // "flight_plans" // "example.xml"
let dtd = pararazzi_home // "conf" // "flight_plans" // "flight_plan.dtd"

let current_ivy = ref ({ posn_lat=0.; posn_long = 0.}, 0.)


let current_zoom = ref 1. (* Would be better not to be global ??? *)


let waypoints_node = fun () ->
   match !tree_view with
    Some xml_tree_view ->
      let xml_root = XmlEdit.root xml_tree_view in
      XmlEdit.child xml_root "waypoints"
  | None -> raise Not_found

let waypoints = fun () ->
  XmlEdit.children (waypoints_node ())

let waypoints_clear = fun () ->
  List.iter XmlEdit.delete (waypoints ())


let remove_waypoint = fun name ->
  let w = List.find (fun w -> XmlEdit.attrib w "name" = name) (waypoints ()) in
  XmlEdit.delete w

let update_waypoint = fun name en alt ->
  let w = List.find (fun w -> XmlEdit.attrib w "name" = name) (waypoints ()) in
  XmlEdit.set_attribs w ["name",name; "x",sof en.east; "y",sof en.north; "alt", sof alt]

let wp_east = fun node -> float_of_string (XmlEdit.attrib node "x")
let wp_north = fun node -> float_of_string (XmlEdit.attrib node "y")
let wp_name = fun node -> (XmlEdit.attrib node "name")

class waypoint = fun root (zoomadj:GData.adjustment) name ?(alt=0.) en ->
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
      if n <> name then begin
	name <- n;
	label#set [`TEXT name]
      end
    method alt = alt
    method label = label
    method xy = let a = item#i2w_affine in (a.(4), a.(5)) (*** item#i2w 0. 0. causes Seg Fault !***)
    method move dx dy = 
      item#move dx dy; 
      label#move dx dy
    method event (ev : GnoCanvas.item_event) =
      begin
	match ev with
	| `BUTTON_PRESS ev ->
	    begin
	      match GdkEvent.Button.button ev with
	      |	1 -> ()
	      |	3 -> self#delete false
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
	      update_waypoint name self#en alt;
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
    method delete = fun from_xmledit ->
      item#destroy ();
      label#destroy ();
      if not from_xmledit then
	remove_waypoint self#name
    method zoom (z:float) =
      let a = item#i2w_affine in
      a.(0) <- 1./.z; a.(3) <- 1./.z; 
      item#affine_absolute a;
      label#affine_absolute a
    initializer (ignore (zoomadj#connect#value_changed (fun () -> self#zoom zoomadj#value)));
  end


let rec assoc_nocase at = function
    [] -> raise Not_found
  | (a, v)::avs ->
      if String.uppercase at = String.uppercase a then v else assoc_nocase at avs

let update_wp x = function
    XmlEdit.Deleted -> x#delete true
  | XmlEdit.New_child _ -> failwith "update_wp"
  | XmlEdit.Modified attribs ->
      try
	let float_attrib = fun a -> fos (assoc_nocase a attribs) in
	let en = { east= float_attrib "x"; north= float_attrib "y" } in
	x#set en;
	x#set_name (assoc_nocase "name" attribs)
      with
	_ -> ()
      
let waypoint = fun group zoomadj node ->
  let float_attrib = 
    fun a -> try float_of_string (XmlEdit.attrib node a) with _ -> 0. in
  let alt = float_attrib "alt" in
  let en = { east= float_attrib "x"; north= float_attrib "y" } in
  let name = XmlEdit.attrib node "name" in
  let x = new waypoint group zoomadj name ~alt en in
  x#zoom !current_zoom;
  XmlEdit.connect node (update_wp x);
  x
  

let write_mission = fun () ->
  match !tree_view with
    Some v ->
      ignore (save_fp (XmlEdit.xml_of_view v))
  | _ -> 
      GToolbox.message_box ~title:"Error" "Cannot write: Please load a flight plan first"

      
let float_attrib = fun x a -> float_of_string (Xml.attrib x a)




let load_mission_xml = fun root zoomadj xml ->
  let xml_tree_view = XmlEdit.create (Dtd.parse_file dtd) xml in
  let xml_root = XmlEdit.root xml_tree_view in
  let wpts = XmlEdit.child xml_root "waypoints" in

  let default_alt = float_attrib xml "alt" in

  Ref.set (float_attrib xml "lat0") (float_attrib xml "lon0");
  let utm0 = Ref.utm0 () in
  
  let display_wp = fun node ->
    let w = waypoint root zoomadj node in
    w#zoom !current_zoom in
  
  List.iter display_wp (XmlEdit.children wpts);

  XmlEdit.connect wpts (function XmlEdit.New_child node -> display_wp node | _ -> ());
  
  tree_view := Some xml_tree_view

let load_mission = fun root zoomadj file ->
  load_mission_xml root zoomadj (Xml.parse_file file)

let open_mission = fun cont root zoomadj ?file () ->
  match file with
    None ->
      ignore (file_dialog ~title:"Open Flight Plan" ~filename:default_flag_flightplans ~callback:(fun name -> load_mission root zoomadj name; cont (RecentFiles.FP name)) ())
  | Some name -> load_mission root zoomadj name; cont (RecentFiles.FP name)


let display_map = fun ?(scale = 1.) x y wgs84 map_name root ->
  try
    let wx, wy = Ref.world_of_wgs84 wgs84 in
    let image = GdkPixbuf.from_file map_name in
    let p = GnoCanvas.pixbuf ~pixbuf:image ~props:[`ANCHOR `NW] root in
    p#lower_to_bottom ();
    p#move  (wx -. x*.scale) (wy -. y*.scale);
    let a = p#i2w_affine in
    a.(0) <- scale; a.(3) <- scale; 
    p#affine_absolute a;
    p
  with
    Ref.WrongUTM ->
      GToolbox.message_box "Error" (sprintf "Map '%s' in wrong UTM zone" map_name);
      failwith "display_map"

      
	     


let load_map = fun root filename ->
  let register = fun _pixbuf -> () in
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

let utm_of_name = fun name ->
  match Str.split (Str.regexp "[ \\t]+") name with
    "WGS84"::lat::long::_ ->
      let lat = fos lat and long = fos long in
      let wgs84 = {posn_lat = (Deg>>Rad)lat; posn_long = (Deg>>Rad)long} in
      utm_of WGS84 wgs84
  | "UTM"::z::x::y::_ ->
      {utm_x = fos x; utm_y = fos y; utm_zone=ios z}
  | _ -> 
      GToolbox.message_box "Calibration error" (sprintf "WGS84 or UTM point expected in '%s'" name);
      failwith "Calibration Error"

let distance = fun (x1,x2) (y1,y2) ->
  sqrt ((x1 -. x2)**2. +. (y1 -. y2)**2.)

let calibrate_map = fun root () ->
  match !tree_view with
    None ->
      GToolbox.message_box ~title:"Error" "Please first load or create a (dummy) flight plan"
  | Some _ ->
      file_dialog ~title:"Load image file" ~callback:(fun name ->
	let image = GdkPixbuf.from_file name in
	let p = GnoCanvas.pixbuf ~pixbuf:image ~props:[`ANCHOR `NW] root in
	p#lower_to_bottom ();
	let dialog = GWindow.window ~border_width:10 ~title:"Map calibration" () in
	let v = GPack.vbox ~packing:dialog#add () in
	let _ = GMisc.label ~text:"Choose 3 waypoints (Left Button)\nRename the waypoints with their geographic coordinates\nFor example: 'WGS84 43.123456 1.234567' or 'UTM 12 530134 3987652'\nClick the button below to save the XML result file\nExit and restart" ~packing:v#add () in
	let h = GPack.hbox ~packing:v#pack () in
	let cal = GButton.button ~label:"Calibrate" ~packing:h#add () in
	let close = GButton.button ~label:"Exit" ~packing:h#add () in
	let cancel = GButton.button ~label:"Close" ~packing:h#add () in
	ignore(cancel#connect#clicked ~callback:dialog#destroy);
	ignore(close#connect#clicked ~callback:(fun () -> exit 0));
	ignore(cal#connect#clicked ~callback:(fun _ ->
	  let coords = fun w ->
	    ((wp_east w) /. Ref.world_unit, 
	     -. (wp_north w) /. Ref.world_unit,
	 utm_of_name (wp_name w)) in
	  
	  let wpts = waypoints () in

	  let utm_zone, scale =
	    match wpts with
	      w1::w2::_w3::_ ->
		let (x1, y1, utm1) = coords w1
		and (x2, y2, utm2) = coords w2 in
		utm1.utm_zone, utm_distance utm1 utm2 /. distance (x1,y1) (x2,y2)
	    | _ -> failwith "Calibration: 3 points required" in
	  let f = fun w ->
	    let (x,y,utm) = coords w in
	    Xml.Element ("point", ["x",sof x;
				   "y",sof y;
				   "utm_x",sof utm.utm_x;
				   "utm_y",sof utm.utm_y], []) in
	  let points = List.map f wpts in
	  let xml = Xml.Element ("map", ["file", Filename.basename name;
					 "projection", "UTM";
					 "scale", sof scale;
					 "utm_zone", soi utm_zone], points) in
	  file_dialog ~filename:(sprintf "%s.xml" (Filename.chop_extension name)) ~title:"Save map ref" ~callback:(fun file ->
	    let f = open_out file in
	    Printf.fprintf f "%s\n" (Xml.to_string_fmt xml);
	    close_out f) ()));
	dialog#show ())
	()



let open_map = fun cont root ?file () ->
  if !Ref.ref0 = None then
    GToolbox.message_box ~title:"Error" "Please first load or create a flight plan"
  else
    match file with
      None ->
	file_dialog ~title:"Open Map" ~filename:default_flag_maps ~callback:(fun name -> ignore (load_map root name); cont (RecentFiles.Map name)) ()
    | Some name ->
	ignore (load_map root name);
	cont (RecentFiles.Map name)



let create_wp = fun canvas zoomadj xw yw () ->
  let en = Ref.en_of_world xw yw in
  let name = gensym "wp" in
  let node = XmlEdit.add_child (waypoints_node ()) "waypoint" ["x",sof en.east;"y",sof en.north;"name",name] in
  let x = waypoint canvas#root zoomadj node in
  ()


let dragging = ref None


let canvas_button_release = fun (_canvas:GnoCanvas.canvas) ev ->
  let state = GdkEvent.Button.state ev in
  if GdkEvent.Button.button ev = 2 then dragging := None;
  false

let canvas_button_press = fun (canvas:GnoCanvas.canvas) zoomadj ev ->
  let state = GdkEvent.Button.state ev in
  if GdkEvent.Button.button ev = 1 then
    let xc = GdkEvent.Button.x ev in
    let yc = GdkEvent.Button.y ev in
    let (xw, yw) = canvas#window_to_world xc yc in
    (* Effective creation delayed to avoid a bug in lablgtk *)
    ignore (GMain.Timeout.add 10 (fun () -> create_wp canvas zoomadj xw yw (); false));
    true
  else if GdkEvent.Button.button ev = 2 && Gdk.Convert.test_modifier `CONTROL state then
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
  if !Ref.ref0 <> None then
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
  else 
    false


let display_help = fun ()->
  GToolbox.message_box ~title:"Help" ~ok:"Close"
  "Load first a flight plan then load a map (XML files)\n\n\
  Left Button: Create a new waypoint\n\
  CTRL Middle Button: Pan the map\n\
  Wheel: Zoom/Unzoom\n\n\
  Middle Button on waypoint: Move\n\
  Right Button on waypoint: Delete"

let ivy = fun port domain root ->
  let bus =
    try Sys.getenv "IVYBUS" with
      Not_found -> Printf.sprintf "%s:%d" domain port in
  Ivy.init "medit" "READY" (fun _ _ -> ());
  Ivy.start bus;

  let plot_utm =
    let last_plot = ref None in
    fun (utm_x:float) (utm_y:float) alt ->
      let utm_zone = (Ref.utm0 ()).utm_zone in
      let utm = {utm_x = utm_x; utm_y = utm_y; utm_zone = utm_zone } in
      current_ivy := of_utm WGS84 utm, alt;
      let (x, y) = Ref.world_of_utm utm in
      (match !last_plot with
	None -> ()
      | Some (x', y') ->
	  ignore (GnoCanvas.line ~points:[|x;y;x';y'|] root));
      last_plot := Some (x,y)
  in

  ignore (Ivy.bind (fun _ args -> plot_utm (fos args.(0)/.100.) (fos args.(1)/.100.) (fos args.(2))) "GPS +[0-9]+ +([0-9]*) +([0-9]*) +[0-9\\.]* +([0-9\\.]*)")

let tiles = Hashtbl.create 7

exception Tile_not_found of string

let image_of = fun tile ->
  try
    Hashtbl.find tiles tile
  with
    Not_found ->
      let file = try find_file tile with Not_found -> raise (Tile_not_found tile) in
      Printf.fprintf stderr " (loading %s) ... " file; flush stderr;
      let img =
	match OImages.tag (OImages.load file []) with
	  OImages.Rgb24 rgb24 ->
	    rgb24
	| OImages.Index8 img ->
	    let rgb = img#to_rgb24 in
	    img#destroy;
	    rgb
	| OImages.Index16 img ->
	    let rgb = img#to_rgb24 in
	    img#destroy;
	    rgb
	| _ -> failwith "image_of"  in
      Hashtbl.add tiles tile img;
      img

let lbt_get = fun lbt ->
  let dalle_x = lbt.lbt_x / 10000
  and dalle_y = 267 - (lbt.lbt_y / 10000) in
  let file = Printf.sprintf "F%03d_%03d.png" dalle_x dalle_y in
  let i = image_of file in
  let x = truncate (float (lbt.lbt_x mod 10000) /. 2.5)
  and y = 3999-truncate (float (lbt.lbt_y mod 10000) /. 2.5) in
  i#get x y

let ignutm = fun root wgs84 sx sy scale ->
  let width = truncate (sx /. scale)
  and height = truncate (sy /. scale) in
  let utm = utm_of WGS84 wgs84 in
  let utm_x0 = utm.utm_x -. sx /. 2.
  and utm_y0 = utm.utm_y -. sy /. 2. in
  let i = new OImages.rgb24 width height in
  Printf.fprintf stderr "Row:    ";
  for x = 0 to i#width - 1 do
    Printf.fprintf stderr "\b\b\b\b%4d" x; flush stderr;
    for y = 0 to i#height - 1 do
      let xm = (float x *. scale)
      and ym = (float (i#height - 1 - y) *. scale) in
      let utm = { utm_x = (utm_x0 +. xm); utm_y = (utm_y0 +. ym); utm_zone = utm.utm_zone } in
      let wgs84 = of_utm WGS84 utm in
      let lbt = lambert_of lambertIIe ((NTF<<WGS84)wgs84) in
      i#unsafe_set x y (lbt_get lbt)
    done
  done;
  Printf.fprintf stderr "\n";
  let name = sprintf "UTM_%.0f_%.0f_%d" (utm_x0/.1000.) (utm_y0/.1000.) utm.utm_zone in
  file_dialog ~filename:(sprintf "%s.png" name) ~title:"Save map image" ~callback:(fun png ->
    i#save png (Some Images.Png) [];
    let point = fun x y utm_x utm_y ->
      Xml.Element ("point", ["x",string_of_int x;
			     "y",string_of_int y;
			     "utm_x",sof utm_x;
			     "utm_y",sof utm_y], []) in
    let points = [point 0 height utm_x0 utm_y0;
		  point 0 0 utm_x0 (utm_y0 +. sy);
		  point width height (utm_x0 +. sx) utm_y0] in
    let xml =
      Xml.Element ("map", ["file", Filename.basename png;
			   "projection", "UTM";
			   "utm_zone", soi utm.utm_zone;
			   "scale", string_of_float scale], points) in
    file_dialog ~filename:(sprintf "%s.xml" name) ~title:"Save map ref" ~callback:(fun file ->
      let f = open_out file in
      Printf.fprintf f "%s\n" (Xml.to_string_fmt xml);
      close_out f;
      load_map root file) ())
    ()

let create_ign = fun root wgs84 ->
  printf "lat=%f long=%f\n%!" wgs84.posn_lat wgs84.posn_long;
  let dialog = GWindow.window ~border_width:10 ~title:"New IGN map" () in
  let v = GPack.vbox ~packing:dialog#add () in
  let h = GPack.hbox ~packing:v#pack () in
  let sx = labelled_entry "Width (m)" "2000" h in
  let sy = labelled_entry "Height (m)" "2000" h in
  let scale = labelled_entry "Scale (m/px)" "2.5" h in
  let h = GPack.hbox ~packing:v#pack () in
  let create = GButton.button ~label:"Create" ~packing: h#add () in
  let cancel = GButton.button ~label:"Cancel" ~packing: h#add () in
  ignore(cancel#connect#clicked ~callback:dialog#destroy);
  ignore(create#connect#clicked ~callback:(fun _ ->
    try
      ignutm root wgs84 (fos sx#text) (fos sy#text) (fos scale#text);
      dialog#destroy ()
    with
      Tile_not_found s ->
	GToolbox.message_box "Error" (sprintf "IGN tile '%s' not found\nUse the -I option to set the path" s)
					  ));
  dialog#show ();;
 
  
  
  

let new_flight_plan = fun root zoomadj () ->
  let dialog = GWindow.window ~border_width:10 ~title:"New flight plan" () in
  let dvbx = GPack.box `VERTICAL ~packing:dialog#add () in
  let h = GPack.hbox ~packing:dvbx#pack () in
  let latlong  = labelled_entry "lat" "WGS84 43.210987 1.324" h in
  let alt0 = labelled_entry "ground_alt" "380" h in
  let get_current = GButton.button ~label:"Get current" ~packing: h#add () in
  ignore(get_current#connect#clicked ~callback:
	   (fun _ ->
	     let (wgs84, alt) = !current_ivy in
	     let s = sprintf "WGS84 %.6f %.6f" ((Rad>>Deg)wgs84.posn_lat)((Rad>>Deg)wgs84.posn_long) in
	     latlong#set_text s;
	     alt0#set_text (sof alt);
	   )
	);
  let h = GPack.hbox ~packing:dvbx#pack () in
  let alt = labelled_entry "alt" "430" h in
  let qfu = labelled_entry "QFU" "270" h in
  let mdfh = labelled_entry "Max dist" "500" h in

  let h = GPack.hbox ~packing:dvbx#pack () in
  let name  = labelled_entry "Name" "Test flight" h in

  let h = GPack.hbox ~packing:dvbx#pack () in
  let createfp = GButton.button ~label:"Create FP" ~packing: h#add () in
  if !ign then begin
    let createign = GButton.button ~label:"Create IGN Map" ~packing: h#add () in
    ignore(createign#connect#clicked ~callback:(fun _ -> create_ign root (Latlong.of_string latlong#text)))
  end;
  let cancel = GButton.button ~label:"Close" ~packing: h#add () in 
  ignore(cancel#connect#clicked ~callback:dialog#destroy);
  ignore(createfp#connect#clicked ~callback:
      begin fun _ ->
	let xml = Xml.parse_file example_file in
	let s = ExtXml.subst_attrib in
	let wgs84 = Latlong.of_string latlong#text in
	let xml = s "lat0" (deg_string_of_rad wgs84.posn_lat) xml in
	let xml = s "lon0" (deg_string_of_rad wgs84.posn_long) xml in
	let xml = s "ground_alt" alt0#text xml in
	let xml = s "qfu" qfu#text xml in
	let xml = s "alt" alt#text xml in
	let xml = s "max_dist_from_home" mdfh#text xml in
	let xml = s "name" name#text xml in
	load_mission_xml root zoomadj xml	
      end);
  dialog#show ();;


let main () =
  let window = GWindow.window ~title: "Paparazzi"
      ~border_width: 1 ~width:800 () in
  let quit = fun () ->
    RecentFiles.save ();
    GMain.Main.quit (); exit 0 in
  ignore (window#connect#destroy ~callback:quit);
  let vbox= GPack.vbox ~height:500 ~packing:window#add () in
  let menubar = GMenu.menu_bar ~packing:vbox#pack () in

 let zoomadj = GData.adjustment 
      ~value:1. ~lower:0.05 ~upper:10. 
    ~step_incr:0.1 ~page_incr:1.0 ~page_size:1.0 () in

  let _w = GEdit.spin_button ~adjustment:zoomadj ~rate:0. ~digits:2 ~width:50 ~packing:vbox#pack () in

  let canvas = GnoCanvas.canvas ~packing:(vbox#pack ~expand:true) () in

  canvas#set_scroll_region (-2500000.) (-2500000.) 2500000. 2500000.;
  let (x, y) = canvas#get_scroll_offsets in
  canvas#scroll_to (x-300) (y-300);

  let zoom = fun value ->
    canvas#set_pixels_per_unit value;
    current_zoom := value
  in

  ignore (zoomadj#connect#value_changed (fun () -> zoom zoomadj#value));

  let root = canvas#root in

(***   ignore (canvas#event#connect#button_press (canvas_button_press canvas)); ***)

  let factory = new GMenu.factory menubar in
  let accel_group = factory#accel_group in
  let file_menu = factory#add_submenu "File"
  and help_menu = factory#add_submenu "Help" in
  let file_menu_fact = new GMenu.factory file_menu ~accel_group
  and help_menu_fact = new GMenu.factory help_menu ~accel_group in

  let register_recent_file = fun f ->
    RecentFiles.add f;
    match f with
      RecentFiles.Map f ->
	ignore (file_menu_fact#add_item f ~callback:(open_map (fun _ -> ()) root ~file:f))
    | RecentFiles.FP f ->
	ignore (file_menu_fact#add_item f ~callback:(open_mission (fun _ -> ()) root zoomadj ~file:f)) in

  ignore (file_menu_fact#add_item "New Flight Plan" ~key:GdkKeysyms._N ~callback:(new_flight_plan root zoomadj));
  ignore (file_menu_fact#add_item "Calibrate Map" ~key:GdkKeysyms._R ~callback:(calibrate_map root));
  ignore (file_menu_fact#add_item "Open Flight Plan" ~key:GdkKeysyms._O ~callback:(open_mission register_recent_file root zoomadj));    
  ignore (file_menu_fact#add_item "Open Map" ~key:GdkKeysyms._M ~callback:(open_map register_recent_file root));    
  ignore (file_menu_fact#add_item "Write Flight Plan" ~key:GdkKeysyms._S ~callback:write_mission);    
  ignore (file_menu_fact#add_item "Clear Waypoints" ~key:GdkKeysyms._C ~callback:waypoints_clear);    
  ignore (file_menu_fact#add_item "Quit" ~key:GdkKeysyms._Q ~callback:quit);
  ignore (file_menu_fact#add_separator ());

  ignore (help_menu_fact#add_item "Keys/Mouse" ~key:GdkKeysyms._H ~callback:display_help);

  RecentFiles.load ();
  
  RecentFiles.iter register_recent_file;

  let bottom = GPack.hbox ~packing:vbox#pack () in
  let lbl_xy = GMisc.label ~packing:bottom#pack () in
  let lbl_geo = GMisc.label ~packing:bottom#pack () in

  ignore (canvas#event#connect#motion_notify (display_coord canvas lbl_xy lbl_geo));
  ignore (canvas#event#connect#button_press (canvas_button_press canvas zoomadj));
  ignore (canvas#event#connect#button_release (canvas_button_release canvas));
  ignore (canvas#event#connect#after#key_press (canvas_key_press canvas)) ;
  ignore (canvas#event#connect#enter_notify (fun _ -> canvas#misc#grab_focus () ; false));

  let scroll =  fun ev ->
    match GdkEvent.Scroll.direction ev with
      `UP -> zoomadj#set_value (zoomadj#value +. zoomadj#step_increment) ; true
    | `DOWN -> zoomadj#set_value (zoomadj#value -. zoomadj#step_increment); true
    | _ -> false in

  ignore (canvas#event#connect#any (fun ev ->
    match GdkEvent.get_type ev with
    | `SCROLL -> scroll (GdkEvent.Scroll.cast ev)
    | _ -> false)) ;

  window#add_accel_group accel_group;
  window#show ();

  let port = ref 2010 
  and domain = ref "127.255.255.255" in
  Arg.parse 
    [ "-b", Arg.Int (fun x -> port := x), "<Port number>\tDefault is 2010, unused if IVYBUS is set";
      "-domain", Arg.String (fun x -> domain := x), "<Network address>\tDefault is 127.255.255.255, unused if IVYBUS is set";
      "-I", Arg.String (fun s -> path := s :: !path), "Data directory";
      "-ign", Arg.Set ign, "To use the french 1/25000 tiles"
    ]
    (fun x -> prerr_endline ("Don't do anything with "^x))
    "Usage: ";

  ivy !port !domain root;

  GMain.Main.main ()

let _ = main ()

