(***************** Editing ONE (single) flight plan **************************)open Printf
open Latlong

module G2D = Geometry_2d


let (//) = Filename.concat
let fp_example = Env.flight_plans_path // "example.xml"
let default_path_maps =  Env.paparazzi_home // "data" // "maps"

(** Dummy flight plan (for map calibration) *)
let dummy_fp = fun latlong ->
  Xml.Element("flight_plan",
	      ["lat0", string_of_float ((Rad>>Deg)latlong.posn_lat);
	       "lon0", string_of_float ((Rad>>Deg)latlong.posn_long);
	       "alt", "42.";
	       "MAX_DIST_FROM_HOME", "1000."],
	      [Xml.Element("waypoints", [],[]);
	       Xml.Element("blocks", [],[])])



let current_fp = ref None

(** Wrapper checking there is currently no flight plan loaded *)
let if_none = fun f ->
  match !current_fp with
    Some _ ->
      GToolbox.message_box "Error" "Only one editable flight plan at a time"
  | None ->
      f ()

let save_fp = fun () ->
  match !current_fp with
    None -> () (* Nothing to save *)
  | Some (fp, filename) ->
      match GToolbox.select_file ~title:"Save Flight Plan" ~filename () with
	None -> ()
      | Some file -> 
	  let f  = open_out file in
	  fprintf f "<!DOCTYPE flight_plan SYSTEM \"flight_plan.dtd\">\n\n";
	  fprintf f "%s\n" (ExtXml.to_string_fmt fp#xml);
	  close_out f

let close_fp = fun () ->
  match !current_fp with
    None -> () (* Nothing to close *)
  | Some (fp, _filename) ->
      let close = fun () ->
	fp#destroy ();
	current_fp := None in
      match GToolbox.question_box ~title:"Closing flight plan" ~buttons:["Close"; "Save&Close"; "Cancel"] "Do you want to save/close ?" with
	2 -> save_fp (); close ()
      | 1 -> close ()	  
      | _ -> ()
	  
let load_xml_fp = fun geomap editor_frame accel_group ?(xml_file=Env.flight_plans_path) xml ->
  Map2d.set_georef_if_none geomap (MapFP.georef_of_xml xml);
  let fp = new MapFP.flight_plan ~editable:true ~show_moved:false geomap "red" Env.flight_plan_dtd xml in
  editor_frame#add fp#window;
  current_fp := Some (fp,xml_file);

  (** Add waypoints as geo references *)
   List.iter 
    (fun w ->
      let (i, w) = fp#index w in
      geomap#add_info_georef (sprintf "%s" w#name) (w :> < pos : geographic>))
    fp#waypoints;
  
  fp

let labelled_entry = fun ?width_chars text value h ->
  let _ = GMisc.label ~text ~packing:h#add () in
  GEdit.entry ?width_chars ~text:value ~packing:h#add ()

let new_fp = fun geomap editor_frame accel_group () ->
  if_none (fun () ->
    let dialog = GWindow.window ~border_width:10 ~title:"New flight plan" () in
    let dvbx = GPack.box `VERTICAL ~packing:dialog#add () in
    let h = GPack.hbox ~packing:dvbx#pack () in
    let default_latlong =
      match geomap#georef with
	None -> "WGS84 37.21098 -113.45678"
      | Some geo -> Latlong.string_of geo in
    let latlong  = labelled_entry ~width_chars:25 "Geographic Reference" default_latlong h in
    let alt0 = labelled_entry ~width_chars:4 "Ground Alt" "380" h in
    let h = GPack.hbox ~packing:dvbx#pack () in
    let alt = labelled_entry ~width_chars:4 "Default Alt" "430" h in
    let qfu = labelled_entry ~width_chars:4 "QFU" "270" h in
    let mdfh = labelled_entry ~width_chars:4 "Max distance from HOME" "500" h in
    
    let h = GPack.hbox ~packing:dvbx#pack () in
    let name  = labelled_entry "Name" "Test flight" h in
    
    let h = GPack.hbox ~packing:dvbx#pack () in
    let cancel = GButton.button ~stock:`CANCEL ~packing: h#add () in 
    ignore(cancel#connect#clicked ~callback:dialog#destroy);

    let createfp = GButton.button ~stock:`OK ~packing: h#add () in
    createfp#grab_default ();
    ignore(createfp#connect#clicked ~callback:
	     begin fun _ ->
	       let xml = Xml.parse_file fp_example in
	       let s = ExtXml.subst_attrib in
	       let wgs84 = Latlong.of_string latlong#text in
	       let xml = s "lat0" (deg_string_of_rad wgs84.posn_lat) xml in
	       let xml = s "lon0" (deg_string_of_rad wgs84.posn_long) xml in
	       let xml = s "ground_alt" alt0#text xml in
	       let xml = s "qfu" qfu#text xml in
	       let xml = s "alt" alt#text xml in
	       let xml = s "max_dist_from_home" mdfh#text xml in
	       let xml = s "name" name#text xml in
	       ignore (load_xml_fp geomap editor_frame accel_group xml);
	       dialog#destroy ()
	     end);
    dialog#show ())


let loading_error = fun xml_file e ->
  let m = sprintf "Error while loading %s:\n%s" xml_file e in
  GToolbox.message_box "Error" m


let load_xml_file = fun geomap editor_frame accel_group xml_file ->
  try
    let xml = Xml.parse_file xml_file in
    ignore (load_xml_fp geomap editor_frame accel_group ~xml_file xml);
    geomap#fit_to_window ();
    match GWindow.toplevel geomap#canvas with
      Some w ->
	w#set_title (sprintf "%s (%s)" w#title (Filename.basename xml_file))
    | None -> ()
  with
    Dtd.Prove_error(e) -> loading_error xml_file (Dtd.prove_error e)
  | Dtd.Check_error(e) -> loading_error xml_file (Dtd.check_error e)
  | Xml.Error e -> loading_error xml_file (Xml.error e)
      

      
(** Loading a flight plan for edition *)
let load_fp = fun geomap editor_frame accel_group () ->
  if_none (fun () ->
    match GToolbox.select_file ~title:"Open flight plan" ~filename:(Env.flight_plans_path // "*.xml") () with
      None -> ()
    | Some xml_file -> load_xml_file geomap editor_frame accel_group xml_file)

let create_wp = fun geomap geo ->
  match !current_fp with
    None -> 
      GToolbox.message_box "Error" "Load a flight plan first";
      failwith "create_wp"
  | Some (fp,_) ->
      let w = fp#add_waypoint geo in 
      geomap#add_info_georef (sprintf "%s" w#name) (w :> < pos : geographic>);
      w



let ref_point_of_waypoint = fun xml ->
  Xml.Element("point", ["x",Xml.attrib xml "x";
			"y",Xml.attrib xml "y";
			"geo", Xml.attrib xml "name"],[])


(** Calibration of chosen image (requires a dummy flight plan) *)
let calibrate_map = fun (geomap:MapCanvas.widget) editor_frame accel_group () ->
  match !current_fp with
  | Some (_fp,_) ->  GToolbox.message_box "Error" "Close current flight plan before calibration"
  | None ->
      match GToolbox.select_file ~filename:(default_path_maps // "") ~title:"Open Image" () with
	None -> ()
      | Some image -> 
	  (** Displaying the image in the NW corner *)
	  let pixbuf = GdkPixbuf.from_file image in
	  let pix = GnoCanvas.pixbuf ~pixbuf ~props:[`ANCHOR `NW] geomap#canvas#root in
	  let (x0, y0) = geomap#canvas#get_scroll_offsets in
	  let (x,y) = geomap#canvas#window_to_world (float x0) (float y0) in
	  pix#move x y;

	  (** Open a dummy flight plan *)
	  let dummy_georef =
	    match geomap#georef with
	      None -> {posn_lat = (Deg>>Rad)43.; posn_long = (Deg>>Rad)1. }
	    | Some geo -> geo in
	  let fp_xml = dummy_fp dummy_georef in
	  let fp = load_xml_fp geomap editor_frame accel_group fp_xml in
	  
	  (** Dialog to finish calibration *)
	  let dialog = GWindow.window ~border_width:10 ~title:"Map calibration" () in
	  let v = GPack.vbox ~packing:dialog#add () in
	  let _ = GMisc.label ~text:"Choose 2 (or more) waypoints (Ctrl-Left)\nRename the waypoints with their geographic coordinates\nFor example: 'WGS84 43.123456 1.234567' or 'UTM 530134 3987652 12' or 'LBT2e 123456 543210'\nClick the button below to save the XML result file\n" ~packing:v#add () in
	  let h = GPack.hbox ~packing:v#pack () in
	  let cancel = GButton.button ~stock:`CLOSE ~packing:h#add () in
	  let cal = GButton.button ~stock:`OK ~packing:h#add () in
	  let destroy = fun () ->
	    dialog#destroy ();
	    close_fp ();
	    pix#destroy () in
	  ignore(cancel#connect#clicked ~callback:destroy);
	  ignore(cal#connect#clicked ~callback:(fun _ ->
	    let points = List.map XmlEdit.xml_of_node fp#waypoints in
	    let points = List.map ref_point_of_waypoint points in
	    let xml = Xml.Element ("map", 
				   ["file", Filename.basename image;
				    "projection", geomap#projection],
				   points) in
	    match GToolbox.select_file ~filename:(default_path_maps//".xml") ~title:"Save calibrated map" () with
	      None -> ()
	    | Some xml_file ->
		let f = open_out xml_file in
		Printf.fprintf f "%s\n" (Xml.to_string_fmt xml);
		close_out f));
	  cal#grab_default ();
	  dialog#show ()

let radius = ref 30.
let path = ref (ref [])
let cur_arc = ref None
and cur_seg = ref None
and cur_f = ref { G2D.x2D = 0.; y2D = 0. }

let set_segment = fun s p1 p2 ->
  let x1 = p1.G2D.x2D and y1 = p1.G2D.y2D
  and x2 = p2.G2D.x2D and y2 = p2.G2D.y2D in
  s#set [`POINTS [|x1;y1;x2;y2|]]

let arc_from_points = fun (geomap:MapCanvas.widget) c l f s ->
  let cl = G2D.vect_make c l
  and cf = G2D.vect_make c f in
  let pol_cl = G2D.cart2polar cl in
  let al = pol_cl.G2D.theta2D
  and af = (G2D.cart2polar cf).G2D.theta2D in
  let xc = c.G2D.x2D and yc = c.G2D.y2D in
  let (a1, a2) = if s > 0. then (al, af) else (af, al) in
  geomap#arc ~nb_points:10 ~fill_color:"blue" ~width:2 (xc,yc) pol_cl.G2D.r2D a1 a2

    (** Update path after waypoint move *)
let update_path = fun (geomap:MapCanvas.widget) path waypoint ->
  let n = waypoint#name in
  let rec loop = function
      [] -> failwith (sprintf "update_path: %s not found" n)
    | [p] -> [p]
    | (wp, Some arc, Some seg, _f, radius)::wps ->
	let new_wps = if wp#name = waypoint#name then wps else loop wps in
	begin
	  match new_wps with
	    [] -> failwith "unreachable"
	  | (wp1, _arc1, _seg1, f1, _r1)::new_wps' -> (* Previous *)
	      let wp1_2D = geomap#pt2D_of wp1#pos
	      and wp_2D = geomap#pt2D_of wp#pos in
	      match new_wps' with
		[] -> (* wp is the second point: simple segment *)
		  set_segment seg wp1_2D wp_2D;
		  (wp, Some arc, Some seg, wp1_2D, radius)::new_wps
	      | _ -> (* At least 2 points before *)
		  let (c, f, s) = G2D.arc_segment f1 wp1_2D wp_2D radius in
		  set_segment seg f wp_2D;
		  arc#destroy ();
		  let new_arc = arc_from_points geomap c wp1_2D f s in
		  (wp, Some new_arc, Some seg, f, radius)::new_wps
	end
    | _ -> failwith "update_path" in
  path := loop !path

      

let path_button = fun (geomap:MapCanvas.widget) (xw, yw) ->
  let geo = geomap#of_world (xw, yw) in
  let wp = create_wp geomap geo in
  let cur_path = !path in
  wp#connect (fun () -> update_path geomap cur_path wp);
  if ! !path = [] then
    cur_f := {G2D.x2D=xw; G2D.y2D=yw};
  cur_path := (wp, !cur_arc, !cur_seg, !cur_f, !radius) :: ! cur_path;
  cur_arc := Some (GnoCanvas.line geomap#canvas#root);
  cur_seg := Some (geomap#segment ~fill_color:"blue" ~width:2 geo geo)

let path_notify (geomap:MapCanvas.widget) (xw, yw) =
  match ! !path with
    [] -> false (** Empty path: nothing to do *)
  | (wp1, _, _, ll,_) :: p -> (** Last is wp1 *)
      begin
	match !cur_seg with
	  None -> failwith "path_notify"
	| Some segment -> 
	    let l = geomap#pt2D_of wp1#pos
	    and cur_2D = {G2D.x2D = xw; y2D = yw } in
	    match p with
	      [] -> (** Only 1 point in the current path:add a simple segment*)
		set_segment segment l cur_2D;
	    | _::_ -> (** Already 2 points: add an arc and a segment *)
		match !cur_arc with
		  None -> failwith "path_notify"
		| Some arc ->
		    arc#destroy ();
		    let (c, f, s) = G2D.arc_segment ll l cur_2D !radius in
		    set_segment segment f cur_2D;
		    let arc = arc_from_points geomap c l f s in
		    cur_arc := Some arc;
		    cur_f := f
      end;	    
      true

	
let path_close = fun () ->
  let destroy = fun ref ->
    match !ref with
      None -> () 
    | Some s -> s#destroy (); ref := None in
  destroy cur_arc;
  destroy cur_seg;
  begin
    match !current_fp with
      None -> ()
    | Some (fp, _) ->
	fp#insert_path (List.map (fun (wp,_,_,_,r) -> (wp, r)) (List.rev ! !path));
  end;
  path := ref []

let path_change_radius = function
    `UP -> radius := !radius *. 1.1
  | `DOWN -> radius := !radius /. 1.1
  | _ -> ()

