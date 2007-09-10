(*
 * $Id$
 *
 * Basic log plotter
 *  
 * Copyright (C) 2007- ENAC, Pascal Brisset, Antoine Drouin
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

open Printf

let (//) = Filename.concat
let logs_dir = Env.paparazzi_home // "var" // "logs"
let sample_kml = Env.paparazzi_home // "data/maps/sample_path.kml"

class type text_value = object method text : string end


let double__ = 
  let underscore = Str.regexp "_" in
  fun s -> Str.global_replace underscore "__" s

let remove_same_t = fun l ->
  let rec loop prev = function
      (t1,y1)::(((t2,y2)::l) as l')->
	if t1 = t2 then
	  let y = if (y2-y1)*(y1-prev) > 0 then y2 else y1 in
	  loop y ((t1,y)::l)
	else
	  (t1,y1)::loop y1 l'
    | l -> l in
  loop 0 l

let rec remove_older t0 = function
    (t,_)::l when t < t0 -> remove_older t0 l
  | l -> l

let rec remove_newer t0 = function
    (t,v)::l when t <= t0 -> (t,v)::remove_newer t0 l
  | l -> []

let compute_ticks = fun min_y max_y ->
  let delta = max_y -. min_y in
  
  let scale = log delta /. log 10. in
  let d = 10. ** floor scale in
  let u = 
    if delta < 2.*.d then d/.5. 
    else if delta < 5.*.d then d/.2.
    else d in
  let tick_min = min_y -. mod_float min_y u in
  (delta, scale, u, tick_min)
	


let (//) = Filename.concat

let colors = [|"red"; "blue"; "green"; "orange"; "purple"; "magenta"|]

type curve = { values: (float*float) array; color : string }

let labelled_entry = fun ?width_chars text value (h:GPack.box) ->
  let label = GMisc.label ~text ~packing:h#pack () in
  label, GEdit.entry ?width_chars ~text:value ~packing:h#pack ()

class plot = fun ~width ~height ~packing () ->
  let da = GMisc.drawing_area ~width ~height ~show:true ~packing () in
  let curves = Hashtbl.create 3 in
  object (self)
    val mutable min_x = max_float
    val mutable max_x = min_float
    val mutable min_y = max_float
    val mutable max_y = -.max_float
    val mutable scale_events = []
    val mutable color_index = 0
    val mutable csts = ([] : float list)
    val mutable auto_scale = true


    method private update_scale = fun values ->
      let n = Array.length values in
      min_x <- min min_x (fst values.(0));
      max_x <- max max_x (fst values.(n-1));
      for i = 0 to n - 1 do
	let _, y = values.(i) in
	min_y <- min min_y y;
	max_y <- max max_y y
      done

    method reset_scale = fun () ->
      if auto_scale then begin
	(* Recomputes the min and max *)
	min_x <- max_float;
	min_y <- max_float;
	max_x <- min_float;
	max_y <- -.max_float;
	Hashtbl.iter (fun _ curve ->
	  self#update_scale curve.values)
	  curves;
	self#wake ()
      end

    method auto_scale = auto_scale
    method set_auto_scale = fun x ->
      auto_scale <- x;
      self#reset_scale ();
      self#redraw ()

    method min_x () = min_x
    method min_y () = min_y
    method set_min_x = fun x -> if not self#auto_scale then begin min_x <- x; self#redraw () end
    method set_min_y = fun x -> if not self#auto_scale then begin min_y <- x; self#redraw () end
    method max_x () = max_x
    method max_y () = max_y
    method set_max_x = fun x -> if not self#auto_scale then begin max_x <- x; self#redraw () end
    method set_max_y = fun x -> if not self#auto_scale then begin max_y <- x; self#redraw () end

    method scale_event = fun cb -> scale_events <- cb :: scale_events
    method wake = fun () -> List.iter (fun cb -> cb ()) scale_events;

    method destroy = fun () -> ()

    method drawing_area = da

    method add_cst = fun v ->
      csts <- v :: csts;
      self#redraw ()

    method delete_cst = fun v ->
      csts <- List.filter (fun x -> x <> v) csts;
      self#redraw ()

    method add_curve = fun (name:string) (values:(float*float) array) ->
      let curve = { values = values; color = colors.(color_index) } in
      Hashtbl.add curves name curve;
      color_index <- (color_index + 1) mod Array.length colors;
      if auto_scale then begin
	self#update_scale values;
	self#wake ()
      end;
      self#wake ();
      self#redraw ();
      curve
	
    method delete_curve = fun (name:string) ->
      Hashtbl.remove curves name;
      self#reset_scale ();
      self#redraw ()

    method redraw = fun () ->
      let {Gtk.width=width; height=height} = da#misc#allocation in
      let dr = GDraw.pixmap ~width ~height ~window:da () in
      dr#set_foreground (`NAME "white");
      dr#rectangle ~x:0 ~y:0 ~width ~height ~filled:true ();

      let left_margin = 50
      and bottom_margin = 20
      and tick_len = 5
      and margin = 3 in
      
      let scale_x = fun x -> left_margin + truncate ((x-.min_x)*. float (width-left_margin) /. (max_x -. min_x))
      and scale_y = fun y -> height-bottom_margin - truncate ((y-.min_y)*. float (height-bottom_margin) /. (max_y -. min_y)) in

      (* Constants *)
      List.iter (fun v ->
	dr#set_foreground (`NAME "black");
	dr#lines [(left_margin, scale_y v); (width, scale_y v)])
	csts;

      let context = da#misc#create_pango_context in
      context#set_font_by_name "sans 8 ";
      
      let layout = context#create_layout in

      (* Curves *)
      let title_y = ref margin in
      Hashtbl.iter (fun title curve ->
	let points = Array.to_list (Array.map (fun (t, v) -> (scale_x t, scale_y v)) curve.values) in
	(* let points = remove_same_t points in *)
	let points = remove_older (scale_x min_x) points in
	let points = remove_newer (scale_x max_x) points in
	dr#set_foreground (`NAME curve.color);
	dr#lines points;

	(* Title *)
	Pango.Layout.set_text layout title;
	let (w, h) = Pango.Layout.get_pixel_size layout in
	dr#rectangle ~x:(width-h-margin) ~y:!title_y ~width:h ~height:h ~filled:true ();

	dr#set_foreground `BLACK;
	dr#put_layout ~x:(width-2*margin-w-h) ~y:(!title_y) layout;
	title_y := !title_y + h + margin)
	curves;

      (* Graduations *)
      if Hashtbl.length curves > 0 then begin
	dr#set_foreground `BLACK;

	(* Y *)
	let (min_y, max_y) = 
	  if max_y > min_y then (min_y, max_y)
	  else  let d = abs_float max_y /. 10. in (max_y -. d, max_y +. d) in

	let delta, scale, u, tick_min = compute_ticks min_y max_y in

	for i = 0 to truncate (delta/.u) + 1 do
	  let tick = tick_min +. float i *. u in
	  let y = scale_y tick in
	  let s = Printf.sprintf "%.*f" (Pervasives.max 0 (2-truncate scale)) tick in
	  Pango.Layout.set_text layout s;
	  let (w, h) = Pango.Layout.get_pixel_size layout in
	  dr#put_layout ~x:(left_margin-margin-w) ~y:(y-h/2) layout;
	  
	  dr#lines [(left_margin,y);(left_margin+tick_len,y)]
	done;
	
	(* Time *)
	let delta, scale, u, tick_min = compute_ticks min_x max_x in
	let y = height-bottom_margin in
	for i = 0 to truncate (delta/.u) + 1 do
	  let tick = tick_min +. float i *. u in
	  let x = scale_x tick in
	  let s = Printf.sprintf "%.*f" (Pervasives.max 0 (2-truncate scale)) tick in
	  Pango.Layout.set_text layout s;
	  let (w, h) = Pango.Layout.get_pixel_size layout in
	  dr#put_layout ~x:(x-w/2) ~y:(y+margin) layout;
	  
	  dr#lines [(x,y);(x,y-tick_len)]
	done
      end;
	
      
      (* Actually draw *)
      (new GDraw.drawable da#misc#window)#put_pixmap ~x:0 ~y:0 dr#pixmap

    initializer(ignore (da#event#connect#expose ~callback:(fun _ -> self#redraw (); false)))
  end


let pprz_float = function
    Pprz.Int i -> float i
  | Pprz.Float f -> f 
  | Pprz.Int32 i -> Int32.to_float i
  | Pprz.String s -> float_of_string s
  | Pprz.Array _ -> 0.


let logs_menus = ref []

let write_kml = fun plot log_name values ->
  let xs = (List.assoc "utm_east" values)
  and ys = (List.assoc "utm_north" values) 
  and zs = (List.assoc "utm_zone" values) 
  and alts = (List.assoc "alt" values) in
  let l = ref [] in
  let t_min = plot#min_x ()
  and t_max = plot#max_x () in
  let t_min = if t_min = max_float then min_float else t_min in
  let t_max = if t_max = min_float then max_float else t_max in
  for i = 0 to Array.length xs - 1 do
    let t = fst xs.(i) in
    if t_min <= t && t < t_max then
      let x = snd xs.(i) /. 100.
      and y = snd ys.(i) /. 100.
      and z = truncate (snd zs.(i))
      and a = snd alts.(i) /. 100. in
      let utm = { LL.utm_x = x; LL.utm_y = y; LL.utm_zone = z } in
      if z <> 0 then
	l := (LL.of_utm LL.WGS84 utm, a) :: !l
  done;
  let l = List.rev !l in
  let xml = Xml.parse_file sample_kml in
  let doc = ExtXml.child xml "Document" in
  let place = ExtXml.child doc "Placemark" in
  let line = ExtXml.child place "LineString" in

  let coords =
    String.concat " " (List.map (fun (p, a) -> sprintf "%.6f,%.6f,%f" ((Rad>>Deg)p.posn_long) ((Rad>>Deg)p.posn_lat) a) l) in
  let coordinates = Xml.Element ("coordinates", [], [Xml.PCData coords]) in

  let line = ExtXml.subst_child "coordinates" coordinates line in
  let place = ExtXml.subst_child "LineString" line place in
  let name = Xml.Element ("name", [], [Xml.PCData log_name]) in
  let place = ExtXml.subst_child "name" name place in
  let doc = ExtXml.subst_child "Placemark" place doc in
  let doc = ExtXml.subst_child "name" (Xml.Element ("name", [], [Xml.PCData log_name])) doc in
  let xml = ExtXml.subst_child "Document" doc xml in

  let title = "Save KML" in
  let dialog = GWindow.file_chooser_dialog ~action:`SAVE ~title () in
  ignore (dialog#set_current_folder logs_dir);
  dialog#add_filter (GFile.filter ~name:"kml" ~patterns:["*.kml"] ());
  dialog#add_button_stock `CANCEL `CANCEL ;
  dialog#add_select_button_stock `SAVE `SAVE ;
  let _ = dialog#set_current_name (log_name^".kml") in
  begin match dialog#run (), dialog#filename with
    `SAVE, Some name ->
      dialog#destroy ();
      let f = open_out name in
      fprintf f "%s\n" (Xml.to_string_fmt xml);
      close_out f
  | _ -> dialog#destroy ()
  end





let add_ac_submenu = fun ?(factor=object method text="1" end) plot menubar (curves_menu_fact: GMenu.menu GMenu.factory) ac menu_name l ->
  let menu = GMenu.menu () in
  let menuitem = GMenu.menu_item ~label:menu_name () in
  menuitem#set_submenu menu;
  menubar#menu#append menuitem;
  
  let menu_fact = new GMenu.factory menu in
  
  (* Build the msg menus *)
  List.iter
    (fun (msg_name, l) ->
      let menu = menu_fact#add_submenu (double__ msg_name) in
      let menu_fact = new GMenu.factory menu in
      (* Build the field menus *)
      List.iter
	(fun (f, values) ->
	  let name = sprintf "%s:%s:%s" menu_name msg_name f in
	  let callback = fun _ ->
	    let factor = try float_of_string factor#text with _ -> 1. in
	    let values = Array.map (fun (t,v) -> (t, v*.factor)) values in
	    let curve = plot#add_curve name values in
	    let eb = GBin.event_box ~width:10 ~height:10 () in
	    eb#coerce#misc#modify_bg [`NORMAL, `NAME curve.color];
	    let item = curves_menu_fact#add_image_item ~image:eb#coerce ~label:name () in
	    
	    let delete = fun () ->
	      plot#delete_curve name;
	      curves_menu_fact#menu#remove (item :> GMenu.menu_item) in
	    ignore (item#connect#activate ~callback:delete)
	  in
	  ignore (menu_fact#add_item ~callback (double__ f)))
	l
    )
    l;
  ignore (menu_fact#add_separator ());
  let callback = fun () ->
    let gps_values = List.assoc "GPS" l in
    write_kml plot menu_name gps_values in
  ignore (menu_fact#add_item ~callback "Export KML path")
    
    

let load_log = fun ?factor (plot:plot) (menubar:GMenu.menu_shell GMenu.factory) curves_fact xml_file ->
  let xml = Xml.parse_file xml_file in
  let data_file =  ExtXml.attrib xml "data_file" in

  let protocol = ExtXml.child xml "protocol" in

  (* In the old days, telemetry class was named telemetry_ap ... *)
  let class_name =
    try
      let name = "telemetry_ap" in
      let _ = ExtXml.child protocol ~select:(fun x -> Xml.attrib x "name" = name) "class" in
      name
    with _ -> "telemetry" in

  let module M = struct let name = class_name let xml = protocol end in
  let module P = Pprz.MessagesOfXml(M) in

  let f = Ocaml_tools.find_file [Filename.dirname xml_file] data_file in
  let f = Ocaml_tools.open_compress f in
  let acs = Hashtbl.create 3 in (* indexed by A/C *)
  try
    while true do
      let l = input_line f in
      try
	Scanf.sscanf l "%f %s %[^\n]"
	  (fun t ac m ->
	    if not (Hashtbl.mem acs ac) then
	      Hashtbl.add acs ac (Hashtbl.create 97);
	    let msgs = Hashtbl.find acs ac in

	    (*Elements of [acs] are assoc lists of [fields] indexed by msg id*)
	    let msg_id, vs = P.values_of_string m in
	    if not (Hashtbl.mem msgs msg_id) then
	      Hashtbl.add msgs msg_id (Hashtbl.create 97);
	    let fields = Hashtbl.find msgs msg_id in

	    (* Elements of [fields] are values indexed by field name *)
	    List.iter (fun (f, v) -> Hashtbl.add fields f (t, v)) vs
	  )
      with
	exc -> prerr_endline (Printexc.to_string exc)
    done
  with
    End_of_file ->
      close_in f;
      (* Compile the data to ease the menu building *)
      Hashtbl.iter (* For all A/Cs *)
	(fun ac msgs ->
	  let menu_name = sprintf "%s:%s" (String.sub data_file 0 18) ac in

	  (* First sort by message id *)
	  let l = ref [] in 
	  Hashtbl.iter (fun msg fields -> l := (P.message_of_id msg, fields):: !l) msgs;
	  let msgs = List.sort (fun (a,_) (b,_) -> compare a b) !l in

	  let msgs =
	    List.map (fun (msg, fields) ->
	      let l = ref [] in 
	      Hashtbl.iter
		(fun f v -> if not (List.mem f !l) then l := f :: !l)
		fields;
	      let l = List.map (fun f ->
		let values = Hashtbl.find_all fields f in
		let values = List.map (fun (t, v) -> (t, pprz_float v)) values in
		let values = Array.of_list values in
		Array.sort compare values;
		(f, values))
		  (List.sort compare !l) in
	      (msg.Pprz.name, l))
	      msgs in
	  
	  (* Store data for other windows *)
	  logs_menus := (ac, menu_name, msgs) :: !logs_menus;
	  
	  add_ac_submenu ?factor plot menubar curves_fact ac menu_name msgs;
	)
	acs



let open_log = fun ?factor plot menubar curves_fact () ->
  ignore (Log_file.chooser ~callback:(fun name -> load_log ?factor plot menubar curves_fact name) ())


let remove_fst_and_snd = function
    _::_::l -> l
  | l -> l

let screenshot_hint_name = 
  let n = ref 0 in
  fun () ->
    match !logs_menus with
      (_, menu_name, _)::_ -> sprintf "%s.png" menu_name
    | _ -> incr n; sprintf "pprz_log-%d.png" !n
	  
let screenshot = fun frame () ->
  let width, height = Gdk.Drawable.get_size frame#misc#window in
  let dest = GdkPixbuf.create width height () in
  GdkPixbuf.get_from_drawable ~dest ~width ~height frame#misc#window;

  let title = "Save snapshot" in
  let dialog = GWindow.file_chooser_dialog ~action:`SAVE ~title () in
  ignore (dialog#set_current_folder logs_dir);
  dialog#add_filter (GFile.filter ~name:"png" ~patterns:["*.png"] ());
  dialog#add_button_stock `CANCEL `CANCEL ;
  dialog#add_select_button_stock `SAVE `SAVE ;
  let name = screenshot_hint_name () in
  let _ = dialog#set_current_name name in
  begin match dialog#run (), dialog#filename with
    `SAVE, Some name ->
      dialog#destroy ();
      GdkPixbuf.save name "png" dest;
  | _ -> dialog#destroy ()
  end


let rec plot_window = fun init ->
  let plotter = GWindow.window ~allow_shrink:true ~title:"Log Plotter" () in
  plotter#set_icon (Some (GdkPixbuf.from_file Env.icon_file));  
  let vbox = GPack.vbox ~packing:plotter#add () in
  let quit = fun () -> GMain.Main.quit (); exit 0 in

  let tooltips = GData.tooltips () in

  let menubar = GMenu.menu_bar ~packing:vbox#pack () in
  let factory = new GMenu.factory menubar in
  let accel_group = factory#accel_group in
  let file_menu = factory#add_submenu "File" in
  let file_menu_fact = new GMenu.factory file_menu ~accel_group in

  let width = 900 and height = 200 in
  let h = GPack.hbox ~packing:vbox#pack () in
  let plot = new plot ~width ~height ~packing:(vbox#pack ~expand:true) () in

  let open_log_item = file_menu_fact#add_item "Open Log" ~key:GdkKeysyms._O in
  
  ignore (file_menu_fact#add_item "New" ~key:GdkKeysyms._N ~callback:(fun () -> plot_window []));
(*
  let close = fun () -> plotter#destroy () in
  ignore (file_menu_fact#add_item "Close" ~key:GdkKeysyms._W ~callback:close); *)

  ignore (file_menu_fact#add_item "Save screenshot" ~key:GdkKeysyms._S ~callback:(screenshot plotter));
  ignore (file_menu_fact#add_item "Quit" ~key:GdkKeysyms._Q ~callback:quit);
  let curves_menu = factory#add_submenu "Curves" in
  let curves_menu_fact = new GMenu.factory curves_menu in

  tooltips#set_tip plot#drawing_area#coerce ~text:"Drop a messages field here to draw it";
  ignore (plotter#connect#destroy ~callback:(fun () -> plot#destroy (); quit ()));

  (* Auto Scale *)
  let auto_scale = GButton.check_button ~label:"Auto Scale" ~active:true ~packing:h#pack () in
  ignore (auto_scale#connect#toggled (fun () -> plot#set_auto_scale auto_scale#active));
  let bounds = [
    ("Tmin", plot#min_x, plot#set_min_x);
    ("Tmax", plot#max_x, plot#set_max_x);
    ("Ymin", plot#min_y, plot#set_min_y);
    ("Ymax", plot#max_y, plot#set_max_y)] in

  let entries = 
    List.map (fun (label, value, action) ->
      let _, entry= labelled_entry ~width_chars:8 label "" h in
      plot#scale_event (fun () -> entry#set_text (string_of_float (value ())));
      ignore (entry#connect#activate ~callback:(fun () -> action (float_of_string entry#text)));
      entry)
      bounds in

  let active_min_maxs = fun () ->
    let b = not auto_scale#active in
    List.iter (fun entry -> entry#misc#set_sensitive b) entries in

  ignore (auto_scale#connect#toggled active_min_maxs);
  active_min_maxs ();

  (* Constants *)
  let _, cst = labelled_entry ~width_chars:5 "Constant:" "" h in
  let add_cst = fun s ->
    let v = float_of_string s in
    plot#add_cst v;
    let eb = GBin.event_box ~width:10 ~height:10 () in
    eb#coerce#misc#modify_bg [`NORMAL, `NAME "black"];
    let item = curves_menu_fact#add_image_item ~image:eb#coerce ~label:s () in
    
    let delete = fun () ->
      plot#delete_cst v;
      curves_menu#remove (item :> GMenu.menu_item) in
    ignore (item#connect#activate ~callback:delete);
  in
  ignore (cst#connect#activate ~callback:(fun () ->add_cst cst#text));
  tooltips#set_tip cst#coerce ~text:"Enter value for a constant curve";

 (* Factor *)
  let factor_label, factor = labelled_entry ~width_chars:5 "Scale next by" "1." h in
  tooltips#set_tip factor#coerce ~text:"Scale next curve (e.g. 0.0174 to convert deg in rad, 57.3 to convert rad in deg)";

  List.iter
    (fun (ac, menu_name, msgs) ->
      add_ac_submenu ~factor:(factor:>text_value) plot factory curves_menu_fact ac menu_name msgs) 
    !logs_menus;

  ignore(open_log_item#connect#activate ~callback:(fun () -> let factor = (factor:>text_value) in open_log ~factor plot factory curves_menu_fact ()));


  List.iter (fun f -> load_log ~factor:(factor:>text_value) plot factory curves_menu_fact f) init;

  plotter#add_accel_group accel_group;
  plotter#show ()




let _ =
  let logs = ref [] in
  Arg.parse
    []
    (fun x -> logs := x :: !logs)
    "Usage: plot <log files>";

  plot_window !logs;

  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do 
    ignore (Glib.Main.iteration true) 
  done
