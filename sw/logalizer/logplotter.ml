(*
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

open Latlong
open Printf

let (//) = Filename.concat
let logs_dir = Env.paparazzi_home // "var" // "logs"
let sample_kml = Env.paparazzi_home // "data/maps/sample_path.kml"
let verbose = ref false

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


let colors = [|(255,0,0);(0,255,0); (0,0,255); (245,215,20); (245,20,245); (30,225,225); (110,30,230)|]

type curve = { values: (float*float) array; color : int*int*int }

let labelled_entry = fun ?width_chars text value (h:GPack.box) ->
  let label = GMisc.label ~text ~packing:h#pack () in
  label, GEdit.entry ?width_chars ~text:value ~packing:h#pack ()

let logs_menus = ref []

let screenshot_hint_name =
  let n = ref 0 in
  fun extension ->
    let basename =
      match !logs_menus with
	(_, menu_name, _, _)::_ -> begin
	  match Str.split (Str.regexp ":") menu_name with
	    menu_prefix::_ -> sprintf "%s" menu_prefix
	  | _ -> sprintf "%s" menu_name
	end
      | _ -> incr n; sprintf "pprz_log-%d" !n in
    sprintf "%s.%s" basename extension

let save_dialog = fun extension callback ->
  let title = "Save snapshot" in
  let dialog = GWindow.file_chooser_dialog ~action:`SAVE ~title () in
  ignore (dialog#set_current_folder logs_dir);
  dialog#add_filter (GFile.filter ~name:extension ~patterns:["*."^extension] ());
  dialog#add_button_stock `CANCEL `CANCEL ;
  dialog#add_select_button_stock `SAVE `SAVE ;
  let name = screenshot_hint_name extension in
  let _ = dialog#set_current_name name in
  begin match dialog#run (), dialog#filename with
    `SAVE, Some name ->
      dialog#destroy ();
      callback name
  | _ -> dialog#destroy ()
  end


let fig_renderer = fun (width, height) ->
  let scale = 12 in (* 1200ppi (for xfig) / 100ppi (for screen) *)
  let width = width * scale
  and height = height * scale in
  object (self)
    val mutable pen_color = Fig.black
    val mutable text = ""
    val mutable objects = []

    method unit = fun x -> scale * x

    method size = (width, height)

    method init = fun () ->
      self#rectangle 0 0 width height ()

    method set_color = fun (r, g, b) ->
      let (id, obj) = Fig.color r g b in
      pen_color <- id;
      objects <- obj :: objects

    method lines = fun points ->
      objects <- Fig.polyline ~pen_color points :: objects

    method rectangle = fun x y width height ?(filled=false) () ->
      let area_fill = if filled then Fig.filled else -1
      and p1 = (x,y)
      and p2 = (x+width,y+height) in
      let points = [p1; p2] in
      objects <- Fig.polyline ~pen_color ~fill_color:pen_color ~sub_type:Fig.Box ~area_fill points :: objects

    val font_size = 10

    method create_text = fun s ->
      text <- s;
      ((3*font_size*scale)/4*String.length s, font_size*scale)

    method put_text = fun x y ->
      let font = Fig.Postscript Fig.Helvetica in
      let obj = Fig.text ~font ~font_size ~color:pen_color (x,y+font_size*scale) text in
      objects <- obj :: objects

    method draw = fun () ->
      let fig = Fig.create objects in
      save_dialog "fig" (fun name -> Fig.write name fig)
  end



class plot = fun ~width ~height ~packing () ->
  let curves = Hashtbl.create 3
  and left_margin = 50
  and bottom_margin = 25
  and top_margin = 20 in

  object (self)
    val mutable min_x = max_float
    val mutable max_x = -.max_float
    val mutable min_y = max_float
    val mutable max_y = -.max_float
    val mutable scale_events = []
    val mutable color_index = 0
    val mutable csts = ([] : float list)
    val mutable auto_scale = true
    val mutable press_x = 0.
    val mutable press_y = 0.
    val mutable motion_x = 0.
    val mutable motion_y = 0.
    val mutable pressed_button = None

    inherit Gtk_tools.pixmap_in_drawin_area ~width ~height ~packing () as pida

    method unscale_x = fun width x ->
      min_x +. (x -. float left_margin) *. (max_x -. min_x) /.  float (width-left_margin)

    method unscale_y = fun height y ->
      let graph_height = height - bottom_margin - top_margin in
      min_y -. (y -. float  (top_margin+graph_height)) *. (max_y -. min_y) /. float graph_height

    method private update_scale = fun ?(update_x=true) values ->
      let n = Array.length values in
      if update_x then begin
	min_x <- min min_x (fst values.(0));
	max_x <- max max_x (fst values.(n-1))
      end;
      for i = 0 to n - 1 do
	let x, y = values.(i) in
	if min_x <= x && x <= max_x then begin
	  min_y <- min min_y y;
	  max_y <- max max_y y
	end
      done

    method reset_scale = fun ?(update_x = true) () ->
      if auto_scale || not update_x then begin
	(* Recomputes the min and max *)
	if update_x then begin
	  min_x <- max_float;
	  max_x <- -.max_float
	end;
	min_y <- max_float;
	max_y <- -.max_float;
	Hashtbl.iter
	  (fun _ curve -> self#update_scale ~update_x curve.values)
	  curves;
	self#wake ()
      end

    method auto_scale = auto_scale
    method set_auto_scale = fun x ->
      auto_scale <- x;
      self#reset_scale ();
      self#recompute ()

    method min_x () = min_x
    method min_y () = min_y
    method set_min_x = fun x -> if not self#auto_scale then begin min_x <- x; self#recompute () end
    method set_min_y = fun x -> if not self#auto_scale then begin min_y <- x; self#recompute () end
    method max_x () = max_x
    method max_y () = max_y
    method set_max_x = fun x -> if not self#auto_scale then begin max_x <- x; self#recompute () end
    method set_max_y = fun x -> if not self#auto_scale then begin max_y <- x; self#recompute () end

    method scale_event = fun cb -> scale_events <- cb :: scale_events
    method wake = fun () -> List.iter (fun cb -> cb ()) scale_events;

    method destroy = fun () -> ()

    method add_cst = fun v ->
      csts <- v :: csts;
      self#recompute ()

    method delete_cst = fun v ->
      csts <- List.filter (fun x -> x <> v) csts;
      self#recompute ()

    method add_curve = fun (name:string) (values:(float*float) array) ->
      let curve = { values = values; color = colors.(color_index) } in
      Hashtbl.add curves name curve;
      color_index <- (color_index + 1) mod Array.length colors;
      if auto_scale then begin
	self#update_scale values;
	self#wake ()
      end;
      self#wake ();
      self#recompute ();
      curve

    method delete_curve = fun (name:string) ->
      Hashtbl.remove curves name;
      self#reset_scale ();
      self#recompute ()

    method da_renderer = fun () ->
      let da = self#drawing_area in
      let context = da#misc#create_pango_context in
      let () = context#set_font_by_name "sans 8 " in
      let layout = context#create_layout in
      let dr = self#get_pixmap () in
      object (self)
	method size =
	  let s = da#misc#allocation in
	  (s.Gtk.width, s.Gtk.height)

	method init = fun () ->
	  dr#set_foreground (`NAME "white");
	  let (width, height) = self# size in
	  dr#rectangle ~x:0 ~y:0 ~width ~height ~filled:true ()

	method set_color = fun (r, g, b) ->
	  dr#set_foreground (`RGB (256*r, 256*g, 256*b))

	method lines = dr#lines

	method rectangle = fun x y width height ?(filled=false) () ->
	  dr#rectangle ~x ~y ~width ~height ~filled ()

	method create_text = fun s ->
	  Pango.Layout.set_text layout s;
	  Pango.Layout.get_pixel_size layout

	method put_text = fun x y ->
	  dr#put_layout ~x ~y layout

	method draw = fun () ->
	  pida#redraw ()

	method unit = fun x -> x
      end

    method export_fig = fun () ->
      let renderer = fig_renderer (self#da_renderer ())#size in
      self#recompute ~renderer ()

    method recompute = fun ?(renderer=self#da_renderer ()) ?(low_res=false) () ->
      let (width, height) = renderer#size in
      renderer#init ();

      let tick_len = renderer#unit 5
      and margin = renderer#unit 3
      and left_margin = renderer#unit left_margin
      and bottom_margin = renderer#unit bottom_margin
      and top_margin = renderer#unit top_margin in

      let black = (0,0,0) in

      let graph_height = height - bottom_margin - top_margin in

      let scale_x = fun x -> left_margin + truncate ((x-.min_x)*. float (width-left_margin) /. (max_x -. min_x))
      and scale_y = fun y -> top_margin+graph_height - truncate ((y-.min_y)*. float graph_height /. (max_y -. min_y)) in

      (* Constants *)
      List.iter
	(fun v ->
	  renderer#set_color black;
	  renderer#lines [(left_margin, scale_y v); (width, scale_y v)])
	csts;

      (* Curves *)
      let title_y = ref margin in
      Hashtbl.iter (fun title curve ->
        let step = if low_res then 10 else 1 in
        let points = ref [] in
        Array.iteri (fun i (t, v) -> if t > min_x && t <= max_x && (i mod step = 0) then points := List.rev_append [(scale_x t, scale_y v)] !points) curve.values;
	renderer#set_color curve.color;
        if List.length !points > 0 then renderer#lines !points;

	(* Title *)
	let (w, h) = renderer#create_text title in
	renderer#rectangle (width-h-margin) !title_y h h ~filled:true ();

	renderer#set_color black;
	renderer#put_text (width-2*margin-w-h) (!title_y);
	title_y := !title_y + h + margin)
	curves;

      (* Graduations *)
      if Hashtbl.length curves > 0 then begin
	renderer#set_color black;

	(* Y *)
	let (min_y, max_y) =
	  if max_y > min_y then (min_y, max_y)
	  else  let d = abs_float max_y /. 10. in (max_y -. d, max_y +. d) in

	let delta, scale, u, tick_min = compute_ticks min_y max_y in

	for i = 0 to truncate (delta/.u) + 1 do
	  let tick = tick_min +. float i *. u in
	  let y = scale_y tick in
	  if y < height - bottom_margin then
	    let s = Printf.sprintf "%.*f" (Pervasives.max 0 (2-truncate scale)) tick in
	    let (w, h) = renderer#create_text s in
	    renderer#put_text (left_margin-margin-w) (y-h/2);

	    renderer#lines [(left_margin,y);(left_margin+tick_len,y)]
	done;

	(* Time *)
	let delta, scale, u, tick_min = compute_ticks min_x max_x in
	let y = height in
	for i = 0 to truncate (delta/.u) + 1 do
	  let tick = tick_min +. float i *. u in
	  let x = scale_x tick in
	  if left_margin < x && x < width then
	    let s = Printf.sprintf "%.*f" (Pervasives.max 0 (2-truncate scale)) tick in
	    let (w, h) = renderer#create_text s in
	    let y = y-margin-h in
	    renderer#put_text (x-w/2) y;

	    renderer#lines [(x,y);(x,y-tick_len)]
	done
      end;

      (* Draw a rectangle of the current selection *)
      begin
	match pressed_button with
	 Some 1 ->
	   let width = abs (truncate (motion_x -. press_x))
	   and height = abs (truncate (motion_y -. press_y)) in
	   if width > 5 && height > 5 then
	     let x = truncate (min press_x motion_x)
	     and y = truncate (min press_y motion_y) in
	     renderer#set_color black;
	     renderer#rectangle x y width height ();
	| _ -> ()
      end;

      (* Actually draw *)
      renderer#draw ()

    method scroll = fun dpx dpy ->
      let scale_x = (max_x -. min_x) /. float (width-left_margin)
      and scale_y = (max_y -. min_y) /. float (height - bottom_margin - top_margin) in
      let dx = scale_x *. float dpx
      and dy = scale_y *. float dpy in
      min_x <- min_x +. dx;
      max_x <- max_x +. dx;
      min_y <- min_y +. dy;
      max_y <- max_y +. dy;
      self#recompute ~low_res:true ()

    method button_press = fun ev ->
      pressed_button <- Some (GdkEvent.Button.button ev);
      match GdkEvent.Button.button ev with
	1 | 2 -> (* left button, store position *)
	  press_x <- GdkEvent.Button.x ev;
	  press_y <- GdkEvent.Button.y ev;
	  true
      | 3 when not auto_scale -> (* right button, reset scale *)
	  self#reset_scale ~update_x:false ();
	  self#recompute ();
	  true
      | _ -> false

    method motion_notify = fun ev ->
      let x = GdkEvent.Motion.x ev
      and y = GdkEvent.Motion.y ev in
      match pressed_button with
	Some 1 ->
	  motion_x <- x;
	  motion_y <- y;
	  self#recompute ~low_res:true ();
	  true
      |	Some 2 -> (* middle button, scroll *)
	  self#scroll (truncate (press_x-.x)) (truncate (y-.press_y));
	  press_x <- x;
	  press_y <- y;
	  true
      | _ -> false


    method button_release = fun ev ->
      pressed_button <- None;
      match GdkEvent.Button.button ev with
	1 -> (* left button, change scale *)
	  let release_x = GdkEvent.Button.x ev
	  and release_y = GdkEvent.Button.y ev in
	  if abs_float (release_x -. press_x) > 25.
	      && abs_float (release_y -. press_y) > 25. then begin
		let {Gtk.width=width;height=height}= self#drawing_area#misc#allocation in
		let new_max_x = self#unscale_x width (max press_x release_x) in
		min_x <- self#unscale_x width (min press_x release_x);
		max_x <- new_max_x;
		let new_max_y = self#unscale_y height (min press_y release_y)in
		min_y <- self#unscale_y height (max press_y release_y);
		max_y <- new_max_y;
		auto_scale <- false;
		self#recompute ()
	      end;
	  true
      | 2 -> self#recompute (); (* recompute at full resolution after a scroll *)
          true
      | _ -> false

    method zoom = fun ev ->
      let {Gtk.width=width;height=height}= self#drawing_area#misc#allocation
      and dx = (max_x -. min_x) and dy = (max_y -. min_y) in
      let alpha_x =
	(GdkEvent.Scroll.x ev -. float left_margin)
	  /. float (width-left_margin)
      and alpha_y =
	(float height-.GdkEvent.Scroll.y ev-. float (top_margin+bottom_margin))
	  /. float (height-top_margin-bottom_margin)
      and ev_state = GdkEvent.Scroll.state ev in
      let ctrl_mod = Gdk.Convert.test_modifier `CONTROL ev_state
      and shift_mod = Gdk.Convert.test_modifier `SHIFT ev_state in
      match GdkEvent.Scroll.direction ev with
	`UP -> (* Zoom factor 2 *)
	  if not shift_mod then begin
	    min_x <- min_x +. dx *. alpha_x /. 2.;
	    max_x <- min_x +. dx /. 2.
	  end;
	  if not ctrl_mod then begin
	    min_y <- min_y +. dy *. alpha_y /. 2.;
	    max_y <- min_y +. dy /. 2.
	  end;
	  auto_scale <- false;
	  self#recompute ();
	  true
      | `DOWN -> (* Unzoom factor 2 *)
	  if not shift_mod then begin
	    min_x <- min_x -. dx *. alpha_x;
	    max_x <- min_x +. dx *. 2.
	  end;
	  if not ctrl_mod then begin
	    min_y <- min_y -. dy *. alpha_y;
	    max_y <- min_y +. dy *. 2.
	  end;
	  auto_scale <- false;
	  self#recompute ();
	  true
      | _ -> false


    initializer ignore (self#drawing_area#event#connect#expose ~callback:(fun _ -> self#recompute (); false))

    initializer ignore (self#drawing_area#event#add [`BUTTON_PRESS; `BUTTON_MOTION; `BUTTON_RELEASE; `SCROLL])

    initializer ignore (self#drawing_area#event#connect#button_press ~callback:self#button_press);
    initializer ignore (self#drawing_area#event#connect#motion_notify ~callback:self#motion_notify);
    initializer ignore (self#drawing_area#event#connect#button_release ~callback:self#button_release);
    initializer ignore (self#drawing_area#event#connect#scroll ~callback:self#zoom);
  end


let pprz_float = function
    Pprz.Int i -> float i
  | Pprz.Float f -> f
  | Pprz.Int32 i -> Int32.to_float i
  | Pprz.Int64 i -> Int64.to_float i
  | Pprz.String s -> let v = try float_of_string s with _ -> 0. in v
  | Pprz.Char c -> let v = try float_of_string (String.make 1 c) with _ -> 0. in v
  | Pprz.Array _ -> 0.


let rec select_gps_values = function
    [] -> []
  | (m, values)::_ when m.Pprz.name = "GPS" ->
      let xs = List.assoc "utm_east" values
      and ys = List.assoc "utm_north" values
      and zs = List.assoc "utm_zone" values
      and alts = List.assoc "alt" values in
      let l = ref [] in
      for i = 0 to Array.length xs - 1 do
	let z = truncate (snd zs.(i))
	and a = snd alts.(i) /. 1000. in
	if z <> 0 && a > 0. then
	    let t = fst xs.(i)
	    and x = snd xs.(i) /. 100.
	    and y = snd ys.(i) /. 100. in
	    let utm = { utm_x = x; utm_y = y; utm_zone = z } in
	    l := (t, of_utm WGS84 utm, a) :: !l
      done;
      List.rev !l
  | (m, values)::_ when m.Pprz.name = "GPS_INT" ->
      let lats = List.assoc "lat" values
      and lons = List.assoc "lon" values
      and alts = List.assoc "hmsl" values in
      let l = ref [] in
      for i = 0 to Array.length lats - 1 do
	let a = snd alts.(i) /. 1000. in
	if a > 0. then
	  let t = fst lats.(i)
	  and lat = snd lats.(i) /. 1e7
	  and lon = snd lons.(i) /. 1e7 in
	  let wgs84 = make_geo lat lon in
	  l := (t, wgs84, a) :: !l
      done;
      List.rev !l


  | _ :: rest ->
	select_gps_values rest


let write_kml = fun plot log_name values ->
  let t_min = plot#min_x ()
  and t_max = plot#max_x () in
  let t_min = if t_min = max_float then -. max_float else t_min in
  let t_max = if t_max = -. max_float then max_float else t_max in

  let l = List.filter (fun (t,_,_) -> t_min <= t && t < t_max) values in

  let xml = Xml.parse_file sample_kml in
  let doc = ExtXml.child xml "Document" in
  let place = ExtXml.child doc "Placemark" in
  let line = ExtXml.child place "LineString" in

  let coords =
    String.concat " " (List.map (fun (_,p, a) -> sprintf "%.6f,%.6f,%f" ((Rad>>Deg)p.posn_long) ((Rad>>Deg)p.posn_lat) a) l) in
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


let bracket_regexp = Str.regexp "\\["


let add_ac_submenu = fun ?(export=false) protocol ?(factor=object method text="1" end) plot menubar (curves_menu_fact: GMenu.menu GMenu.factory) ac menu_name l raw_msgs ->
  let menu = GMenu.menu () in
  let menuitem = GMenu.menu_item ~label:menu_name () in
  menuitem#set_submenu menu;
  menubar#menu#append menuitem;

  let menu_fact = new GMenu.factory menu in

  (* Build the msg menus *)
  List.iter
    (fun (msg, l) ->
      let msg_name = msg.Pprz.name in
      let menu = menu_fact#add_submenu (double__ msg_name) in
      let menu_fact = new GMenu.factory menu in
      (* Build the field menus *)
      List.iter (* forall fields *)
	(fun (f, values) ->
	  let callback = fun _ ->
	    (* Remove the . for an array field name *)
	    let f' = List.hd (Str.split bracket_regexp f) in

	    let alt_unit_coef =  (List.assoc f' msg.Pprz.fields).Pprz.alt_unit_coef in
	    let name = sprintf "%s:%s:%s:%s" menu_name msg_name f factor#text
	    and (a, b) = Ocaml_tools.affine_transform factor#text
	    and (a', b') = Ocaml_tools.affine_transform alt_unit_coef in
	    let a = a *. a' and b = a*.b' +. b in
	    let values = Array.map (fun (t,v) -> (t, v*.a+.b)) values in
	    let curve = plot#add_curve name values in
	    let eb = GBin.event_box ~width:10 ~height:10 () in
	    let (r, g, b) = curve.color in
	    eb#coerce#misc#modify_bg [`NORMAL, `RGB (256*r,256*g,256*b)];
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
    write_kml plot menu_name (select_gps_values l) in
  ignore (menu_fact#add_item ~callback "Export KML path");
  let callback = fun ?no_gui () ->
    Export.popup ?no_gui protocol menu_name raw_msgs in
  ignore (menu_fact#add_item ~callback "Export CSV");
  if export then
    callback ~no_gui:true ()


let load_log = fun ?export ?factor (plot:plot) (menubar:GMenu.menu_shell GMenu.factory) curves_fact xml_file ->
  Debug.call 'p' (fun f ->  fprintf f "load_log: %s\n" xml_file);
  let xml = Xml.parse_file xml_file in
  let data_file =  ExtXml.attrib xml "data_file" in

  Debug.call 'p' (fun f ->  fprintf f "data_file: %s\n" data_file);

  let protocol = ExtXml.child xml "protocol" in

  (* In the old days, telemetry class was named telemetry_ap ... *)
  let class_name =
    try
      let name = "telemetry_ap" in
      let _ = begin try ExtXml.child protocol ~select:(fun x -> Xml.attrib x "name" = name) "msg_class"
        with Not_found -> ExtXml.child protocol ~select:(fun x -> Xml.attrib x "name" = name) "class"
      end in
      name
    with _ -> "telemetry" in

  Debug.call 'p' (fun f ->  fprintf f "class_name: %s\n" class_name);

  let module M = struct let name = class_name let xml = protocol end in
  let module P = Pprz.MessagesOfXml(M) in

  let f =
    try
      Ocaml_tools.find_file [Filename.dirname xml_file] data_file
    with
      Not_found ->
	fprintf stderr "File '%s' not found\n%!" data_file;
	let data_file = Filename.chop_extension (Filename.basename xml_file) ^ ".data" in
	fprintf stderr "Trying with '%s'\n%!" data_file;
	try
          Ocaml_tools.find_file [Filename.dirname xml_file] data_file
        with Not_found ->
          fprintf stderr "File '%s' not found\n%!" data_file;
	  failwith "Data file not found" in
  let f = Ocaml_tools.open_compress f in
  let acs = Hashtbl.create 3 in (* indexed by A/C *)
  try
    while true do
      let l = input_line f in
      try
	Scanf.sscanf l "%f %s %[^\n]"
	  (fun t ac m ->
	    if not (Hashtbl.mem acs ac) then
	      Hashtbl.add acs ac (Hashtbl.create 97, ref []);
	    let msgs, raw_msgs = Hashtbl.find acs ac in

	    (*Elements of [acs] are assoc lists of [fields] indexed by msg id*)
	    let msg_id, vs = P.values_of_string m in
	    if not (Hashtbl.mem msgs msg_id) then
	      Hashtbl.add msgs msg_id (Hashtbl.create 97);
	    let fields = Hashtbl.find msgs msg_id in

	    (* Elements of [fields] are values indexed by field name *)
	    List.iter
	      (fun (f, value) ->
		match value with
		  Pprz.Array array ->
		    Array.iteri
		      (fun i scalar ->
			let f = sprintf "%s[%d]" f i in
			Hashtbl.add fields f (t, scalar))
		      array
		| scalar ->
		    Hashtbl.add fields f (t, scalar))
	      vs;

	    let msg_name = (P.message_of_id msg_id).Pprz.name in
	    raw_msgs := (t, msg_name, vs) :: !raw_msgs
	  )
      with
	exc ->
	  if !verbose then
	    prerr_endline (Printexc.to_string exc)
    done
  with
    End_of_file ->
      close_in f;
      (* Compile the data to ease the menu building *)
      Hashtbl.iter (* For all A/Cs *)
	(fun ac (msgs, raw_msgs) ->
	  let raw_msgs = List.rev !raw_msgs in
	  let menu_name = sprintf "%s:%s" (Filename.chop_extension (Filename.basename xml_file)) ac in

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
	      let sorted_fields = List.sort compare !l in

	      let field_values_assoc =
		List.map
		  (fun f ->
		    let values = Hashtbl.find_all fields f in
		    let values = List.map (fun (t, v) -> (t, pprz_float v)) values in
		    let values = Array.of_list values in
		    Array.sort compare values;
		    (f, values))
		  sorted_fields in
	      (msg, field_values_assoc))
	      msgs in

	  (* Store data for other windows *)
	  logs_menus :=  !logs_menus @ [(ac, menu_name, (msgs, raw_msgs), protocol)];

	  add_ac_submenu ?export protocol ?factor plot menubar curves_fact ac menu_name msgs raw_msgs;
	)
	acs



let open_log = fun ?factor plot menubar curves_fact () ->
  ignore (Log_file.chooser ~callback:(fun name -> load_log ?factor plot menubar curves_fact name) ())


let screenshot = fun frame ->
  let width, height = Gdk.Drawable.get_size frame#misc#window in
  let dest = GdkPixbuf.create width height () in
  GdkPixbuf.get_from_drawable ~dest ~width ~height frame#misc#window;
  save_dialog
    "png"
    (fun name -> GdkPixbuf.save name "png" dest)



(** Table of current windows, to be able to quit when the last one is closed
 FIXME: should be shared with plotter.ml *)
let windows = Hashtbl.create 3

(*****************************************************************************)
let rec plot_window = fun ?export init ->
  let plotter = GWindow.window ~allow_shrink:true ~title:"Log Plotter" () in

  (* Register the window *)
  let oid = plotter#get_oid in
  Hashtbl.add windows oid ();

  plotter#set_icon (Some (GdkPixbuf.from_file Env.icon_log_file));
  let vbox = GPack.vbox ~packing:plotter#add () in
  let quit = fun () -> GMain.Main.quit (); exit 0 in
  let close = fun () ->
    plotter#destroy ();
    Hashtbl.remove windows oid;
    if Hashtbl.length windows = 0 then
      quit () in

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

  let delayed_screenshot = fun () ->
    ignore (GMain.Idle.add (fun () -> screenshot plot#drawing_area; false)) in
  ignore (file_menu_fact#add_item "Save screenshot" ~key:GdkKeysyms._S ~callback:delayed_screenshot);

  let delayed_export = fun () ->
    ignore (GMain.Idle.add (fun () -> (try plot#export_fig () with exc -> prerr_endline (Printexc.to_string exc)); false)) in
  ignore (file_menu_fact#add_item "Export fig" ~key:GdkKeysyms._X ~callback:delayed_export);
  ignore (file_menu_fact#add_separator ());
  ignore (file_menu_fact#add_item "Close" ~key:GdkKeysyms._W ~callback:close);
  ignore (file_menu_fact#add_item "Quit" ~key:GdkKeysyms._Q ~callback:quit);

  let curves_menu = factory#add_submenu "Curves" in
  let curves_menu_fact = new GMenu.factory curves_menu in
  tooltips#set_tip curves_menu#coerce ~text:"Delete";

  ignore (plotter#connect#destroy ~callback:close);

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
  let factor_label, factor = labelled_entry ~width_chars:5 "Scale next by" "1+0" h in
  tooltips#set_tip factor#coerce ~text:"Scale next curve (e.g. 0.0174 to convert deg in rad, 57.3 to convert rad in deg, 1.8+32 to convert Celsius into Fahrenheit)";

  List.iter
    (fun (ac, menu_name, (msgs, raw_msgs), protocol) ->
      add_ac_submenu protocol ~factor:(factor:>text_value) plot factory curves_menu_fact ac menu_name msgs raw_msgs)
    !logs_menus;

  ignore(open_log_item#connect#activate ~callback:(fun () -> let factor = (factor:>text_value) in open_log ~factor plot factory curves_menu_fact ()));


  List.iter (fun f -> load_log ?export ~factor:(factor:>text_value) plot factory curves_menu_fact f) init;

  plotter#add_accel_group accel_group;
  plotter#show ()



(***************************** Main ****************************************)
let () =
  let logs = ref []
  and export = ref false in
  Arg.parse
    [ ("-export_csv", Arg.Set export, "Export in CSV in batch mode according to saved preferences (conf/%gconf.xml)");
      ("-v", Arg.Set verbose, "Verbose")]
    (fun x -> logs := x :: !logs)
    "Usage: logplotter <log files>";

  plot_window ~export: !export !logs;

  if not !export then
    let loop = Glib.Main.create true in
    while Glib.Main.is_running loop do
      ignore (Glib.Main.iteration true)
    done
