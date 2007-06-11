(*
 * $Id$
 *
 * Real time plotter
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

open Printf

class type text_value = object method text : string end


module P = Pprz.Messages (struct let name = "telemetry" end)


let double__ = 
  let underscore = Str.regexp "_" in
  fun s -> Str.global_replace underscore "__" s


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
    val mutable max_y = min_float
    val mutable scale_events = []
    val mutable color_index = 0
    val mutable csts = ([] : float list)
    val mutable auto_scale = true

    method auto_scale = auto_scale
    method set_auto_scale = fun x -> auto_scale <- x
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
	let n = Array.length values in
	min_x <- min min_x (fst values.(0));
	max_x <- max max_x (fst values.(n-1));
	for i = 0 to n - 1 do
	  let _, y = values.(i) in
	  min_y <- min min_y y;
	  max_y <- max max_y y
	done;
	self#wake ()
      end;
      self#redraw ();
      curve
	
    method delete_curve = fun (name:string) ->
      Hashtbl.remove curves name;
      self#redraw ()

    method redraw = fun () ->
      let {Gtk.width=width; height=height} = da#misc#allocation in
      let dr = GDraw.pixmap ~width ~height ~window:da () in
      dr#set_foreground (`NAME "white");
      dr#rectangle ~x:0 ~y:0 ~width ~height ~filled:true ();

      (* Time Graduations *)
      let context = da#misc#create_pango_context in
      context#set_font_by_name "sans 10 ";
      let layout = context#create_layout in
      
      let _f = fun x y s ->
	Pango.Layout.set_text layout s;
	let (w, h) = Pango.Layout.get_pixel_size layout in
	dr#put_layout ~x ~y:(y-h) ~fore:`BLACK layout in
      
      let scale_x = fun x -> truncate ((x-.min_x)*. float width /. (max_x -. min_x))
      and scale_y = fun y -> height - truncate ((y-.min_y)*. float height /. (max_y -. min_y)) in

      (* Constants *)
      List.iter (fun v ->
	dr#set_foreground (`NAME "black");
	dr#lines [(0, scale_y v); (width, scale_y v)])
	csts;

      (* Curves *)
      Hashtbl.iter (fun _ curve ->
	let points = Array.to_list (Array.map (fun (t, v) -> (scale_x t, scale_y v)) curve.values) in
	dr#set_foreground (`NAME curve.color);
	dr#lines points)
	curves;
      (new GDraw.drawable da#misc#window)#put_pixmap ~x:0 ~y:0 dr#pixmap

    initializer(ignore (da#event#connect#expose ~callback:(fun _ -> self#redraw (); false)))
  end


(* Indexed by log *)
let logs = Hashtbl.create 3

let pprz_float = function
    Pprz.Int i -> float i
  | Pprz.Float f -> f 
  | Pprz.Int32 i -> Int32.to_float i
  | Pprz.String s -> float_of_string s
  | Pprz.Array _ -> failwith "pprz_float"


let logs_menus = ref []


(* FIXME: awfull: rebuild the menu from scratch for every new window *)
let add_ac_submenu = fun ?(factor=object method text="1" end) plot menubar (curves_menu_fact: GMenu.menu GMenu.factory) ac menu_name msgs ->
  let menu = GMenu.menu () in
  let menuitem = GMenu.menu_item ~label:menu_name () in
  menuitem#set_submenu menu;
  menubar#menu#append menuitem;
  
  let menu_fact = new GMenu.factory menu in
  let l = ref [] in 
  Hashtbl.iter (fun msg fields -> l := (P.message_of_id msg, fields):: !l) msgs;
  let l = List.sort (fun (a,_) (b,_) -> compare a b) !l in
  
  (* Build the msg menus *)
  List.iter
    (fun (msg, fields) ->
      let menu = menu_fact#add_submenu (double__ msg.Pprz.name) in
      let menu_fact = new GMenu.factory menu in
      let l = ref [] in 
      Hashtbl.iter
	(fun f v -> if not (List.mem f !l) then l := f :: !l)
	fields;
      let l = List.sort compare !l in
      (* Build the field menus *)
      List.iter
	(fun f ->
	  let values = Hashtbl.find_all fields f in
	  let values = List.map (fun (t, v) -> (t, pprz_float v)) values in
	  let values = Array.of_list values in
	  Array.sort compare values;
	  let name = sprintf "%s:%s:%s:%s" menu_name ac msg.Pprz.name f in
	  let callback = fun _ ->
	    let factor = float_of_string factor#text in
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
    l
    
    

let load_log = fun ?factor (plot:plot) (menubar:GMenu.menu_shell GMenu.factory) curves_fact xml_file ->
  let xml = Xml.parse_file xml_file in
  let data_file =  ExtXml.attrib xml "data_file" in

  let f = Ocaml_tools.find_file [Filename.dirname xml_file] data_file in
  let f = Ocaml_tools.open_compress f in
  let acs = Hashtbl.create 3 in (* indexed by A/C *)
  Hashtbl.add logs data_file acs;
  try
    while true do
      let l = input_line f in
      try
	Scanf.sscanf l "%f %s %[^\n]"
	  (fun t ac m ->
	    if not (Hashtbl.mem acs ac) then
	      Hashtbl.add acs ac (Hashtbl.create 97);
	    let msgs = Hashtbl.find acs ac in

	    let msg_id, vs = P.values_of_string m in
	    if not (Hashtbl.mem msgs msg_id) then
	      Hashtbl.add msgs msg_id (Hashtbl.create 97);
	    let fields = Hashtbl.find msgs msg_id in

	    List.iter (fun (f, v) -> Hashtbl.add fields f (t, v)) vs
	  )
      with
	exc -> prerr_endline (Printexc.to_string exc)
    done
  with
    End_of_file ->
      close_in f;
      (* Build the A/C menus *)
      Hashtbl.iter
	(fun ac msgs ->
	  let menu_name = sprintf "%s:%s" (String.sub data_file 0 18) ac in
	  (* Store it for other windows *)
	  logs_menus := (ac, menu_name, msgs) :: !logs_menus;

	  add_ac_submenu ?factor plot menubar curves_fact ac menu_name msgs;
	)
	acs



let file_dialog ~title ~callback () =
  let sel = GWindow.file_selection ~title ~filename:"var/logs/*.log" ~modal:true () in
  ignore (sel#cancel_button#connect#clicked ~callback:sel#destroy);
  ignore
    (sel#ok_button#connect#clicked
       ~callback:(fun () ->
	 let name = sel#filename in
	 sel#destroy ();
	 callback name));
  sel#show ()


let open_log = fun ?factor plot menubar curves_fact () ->
  ignore (file_dialog ~title:"Open Log" ~callback:(fun name -> load_log ?factor plot menubar curves_fact name) ())


let remove_fst_and_snd = function
    _::_::l -> l
  | l -> l


let rec plot_window = fun init ->
  let plotter = GWindow.window ~allow_shrink:true ~title:"Log Plotter" () in
  let vbox = GPack.vbox ~packing:plotter#add () in
  let quit = fun () -> GMain.Main.quit (); exit 0 in

  let tooltips = GData.tooltips () in

  let menubar = GMenu.menu_bar ~packing:vbox#pack () in
  let factory = new GMenu.factory menubar in
  let accel_group = factory#accel_group in
  let file_menu = factory#add_submenu "File" in
  let file_menu_fact = new GMenu.factory file_menu ~accel_group in

  let width = 900 and height = 200 in
  let plot = new plot ~width ~height ~packing:(vbox#pack ~expand:true) () in

  let open_log_item = file_menu_fact#add_item "Open Log" ~key:GdkKeysyms._O in
  
  ignore (file_menu_fact#add_item "New" ~key:GdkKeysyms._N ~callback:(fun () -> plot_window []));
(*
  let close = fun () -> plotter#destroy () in
  ignore (file_menu_fact#add_item "Close" ~key:GdkKeysyms._W ~callback:close); *)

  ignore (file_menu_fact#add_item "Quit" ~key:GdkKeysyms._Q ~callback:quit);
  let curves_menu = factory#add_submenu "Curves" in
  let curves_menu_fact = new GMenu.factory curves_menu in

  List.iter
    (fun (ac, menu_name, msgs) ->
      add_ac_submenu plot factory curves_menu_fact ac menu_name msgs) 
    !logs_menus;

  let h = GPack.hbox ~packing:vbox#pack () in

  tooltips#set_tip plot#drawing_area#coerce ~text:"Drop a messages field here to draw it";
  ignore (plotter#connect#destroy ~callback:(fun () -> plot#destroy ()));

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
      let _, entry= labelled_entry ~width_chars:5 label "" h in
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
  let factor_label, factor = labelled_entry ~width_chars:5 "Scale" "1." h in
  tooltips#set_tip factor#coerce ~text:"Scale next curve (e.g. 0.0174 to convert deg in rad)";

  ignore(open_log_item#connect#activate ~callback:(let factor = (factor:>text_value) in open_log ~factor plot factory curves_menu_fact));


  List.iter (fun f -> load_log ~factor:(factor:>text_value) plot factory curves_menu_fact f) init;

  plotter#add_accel_group accel_group;
  plotter#show ()




let _ =
  let logs = ref [] in
  Arg.parse
    []
    (fun x -> logs := x :: !logs)
    "Usage: ";

  plot_window !logs;

  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do 
    ignore (Glib.Main.iteration true) 
  done
