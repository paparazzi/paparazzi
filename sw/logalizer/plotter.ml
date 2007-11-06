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


let (//) = Filename.concat

let dnd_targets = [ { Gtk.target = "STRING"; flags = []; info = 0} ]
let parse_dnd =
  let sep = Str.regexp ":" in
  fun s ->
    match Str.split sep s with
      [s; c; m; f] -> (s, c, m, f, 1.)
    | [s; c; m; f; factor] -> (s, c, m, f, float_of_string factor)
    | _ -> failwith (Printf.sprintf "parse_dnd: %s" s)


let colors = [|"red"; "blue"; "green"; "orange"; "purple"; "magenta"|]

let labelled_entry = fun ?width_chars text value (h:GPack.box) ->
  let label = GMisc.label ~text ~packing:h#pack () in
  label, GEdit.entry ?width_chars ~text:value ~packing:h#pack ()

type values = { mutable array: float option array; mutable index: int; color : string }

let create_values = fun size color ->
  { array = Array.create size None; index = 0; color = color }

type status = 
    Run 
  | Suspend (* Display is freezed, data are updated *)
  | Stop    (* Display is active, data are not updated *)

class plot = fun ~size ~width ~height ~packing () ->
  let da = GMisc.drawing_area ~width ~height ~show:true ~packing () in
  let curves = Hashtbl.create 3 in
  object (self)
    val mutable min = max_float
    val mutable max = min_float
    val mutable size = size
    val mutable dt = 0.5
    val mutable color_index = 0
    val mutable timer = None
    val mutable csts = ([] : float list)
    val mutable status = Run
    val mutable auto_scale = true

    method auto_scale = auto_scale
    method set_auto_scale = fun x -> auto_scale <- x
    method min = min
    method set_min = fun x -> min <- x
    method max = max
    method set_max = fun x -> max <- x

    method suspend = fun () ->
      status <- Suspend

    method stop = fun () ->
      status <- Stop

    method restart = fun () ->
      status <- Run

    method destroy = fun () ->
      self#stop_timer ()

    method drawing_area = da

    method add_cst = fun v ->
      csts <- v :: csts

    method delete_cst = fun v ->
      csts <- List.filter (fun x -> x <> v) csts

    method reset () =
      if auto_scale then begin
	min <- max_float;
	max <- min_float
      end;
      Hashtbl.iter (fun _ a ->
	a.index <- 0;
	for i = 0 to Array.length a.array - 1 do a.array.(i) <- None done)
	curves

    method set_size = fun new_size ->
      if new_size <> size && new_size > 0 then begin
	Hashtbl.iter (fun _ a ->
	  let new_array = Array.create new_size None in
	  for i = 0 to Pervasives.min size new_size - 1 do
	    new_array.(new_size - 1 - i) <- a.array.((a.index-i+size) mod size)
	  done;
	  a.array <- new_array;
	  a.index <- new_size - 1)
	  curves;
	size <- new_size
      end

    method create_curve = fun (name:string) ->
      let color = colors.(color_index) in
      let values = create_values size color in
      color_index <- (color_index+1) mod Array.length colors;
      Hashtbl.add curves name values;
      values
	
    method delete_curve = fun name ->
      Hashtbl.remove curves name

    method add_value = fun name v ->
      if status <> Stop then
	let a = Hashtbl.find curves name in
	a.array.(a.index) <- Some v;
	if auto_scale then begin
	  min <- Pervasives.min min v;
	  max <- Pervasives.max max v
	end

    method shift = fun () ->
      Hashtbl.iter
	(fun _ a ->
	  (* Shift *)
	  a.index <- (a.index + 1) mod (Array.length a.array);
	  a.array.(a.index) <- None)
	curves
	
    method update_curves = fun () ->
      if Hashtbl.length curves > 0 then
	try
	  if status <> Stop then
	    self#shift ();
	  if status <> Suspend then
	    let {Gtk.width=width; height=height} = da#misc#allocation in
	    let dr = GDraw.pixmap ~width ~height ~window:da () in
	    dr#set_foreground (`NAME "white");
	    dr#rectangle ~x:0 ~y:0 ~width ~height ~filled:true ();
	    let margin = Pervasives.min (height / 10) 20 in

	    (* Time Graduations *)
	    let context = da#misc#create_pango_context in
	    context#set_font_by_name ("sans " ^ string_of_int (margin/2));
	    let layout = context#create_layout in

	    Pango.Layout.set_text layout "X";
	    let (_, h) = Pango.Layout.get_pixel_size layout in

	    let f = fun x y s ->
	      Pango.Layout.set_text layout s;
	      let (w, h) = Pango.Layout.get_pixel_size layout in
	      dr#put_layout ~x ~y:(y-h/2) ~fore:`BLACK layout in
	    
	    let t = dt *. float size in
	    f (width-width/size) (height-h/2) "0";
	    f (width/2) (height-h/2) (Printf.sprintf "-%.1fs" (t/.2.));
	    f 0 (height-h/2) (Printf.sprintf "-%.1fs" t);

	    (* Y graduations *)
	    let (min, max) = 
	      if max > min then (min, max)
	      else  let d = abs_float max /. 10. in (max -. d, max +. d) in
	    let delta = max -. min in
	    
	    let dy = float (height-2*margin) /. delta in
	    let y = fun v ->
	      height - margin - truncate ((v-.min)*.dy) in

	    let scale = log delta /. log 10. in
	    let d = 10. ** floor scale in
	    let u = 
	      if delta < 2.*.d then d/.5. 
	      else if delta < 5.*.d then d/.2.
	      else d in
	    let tick_min = min -. mod_float min u in
	    for i = 0 to truncate (delta/.u) + 1 do
	      let tick = tick_min +. float i *. u in
	      f 0 (y tick) (Printf.sprintf "%.*f" (Pervasives.max 0 (2-truncate scale)) tick)
	    done;

	    (* Constants *)
	    List.iter (fun v ->
	      dr#set_foreground (`NAME "black");
	      dr#lines [(0, y v); (width-width/size, y v)])
	      csts;
	    
	    Hashtbl.iter
	      (fun _ a ->
		(* Draw *)
		let curve = ref [] in
		assert (size = Array.length a.array);
		for i = 0 to size - 1 do
		  let i' = (i+a.index) mod size in
		  match a.array.(i') with
		    None -> ()
		  | Some v ->
		      curve := ((i * width) / size, y v) :: !curve;
		done;
		if !curve <> [] then begin
		  dr#set_foreground (`NAME a.color);
		  dr#lines !curve;
		end;
		(new GDraw.drawable da#misc#window)#put_pixmap ~x:0 ~y:0 dr#pixmap)
	      curves
	with
	  exc ->
	    prerr_endline (Printexc.to_string exc)

    method stop_timer = fun () ->
      match timer with
	None -> ()
      | Some t -> GMain.Timeout.remove t
	    
    method set_update_time = fun delay ->
      self#stop_timer ();
      dt <- delay;
      timer <- Some (GMain.Timeout.add (truncate (dt*.1000.)) (fun () ->self#update_curves (); true))
  end

let update_time = ref 0.5
let size = ref 500

type window = { title : string; geometry : string; curves : string list }

let default_window = {title="Plotter"; geometry=""; curves=[]}

let rec plot_window = fun window ->
  let plotter = GWindow.window ~allow_shrink:true ~title:window.title () in
  ignore (plotter#parse_geometry window.geometry);
  plotter#set_icon (Some (GdkPixbuf.from_file Env.icon_file));
  let vbox = GPack.vbox ~packing:plotter#add () in
  let quit = fun () -> GMain.Main.quit (); exit 0 in

  let tooltips = GData.tooltips () in

  let menubar = GMenu.menu_bar ~packing:vbox#pack () in
  let factory = new GMenu.factory menubar in
  let accel_group = factory#accel_group in
  let file_menu = factory#add_submenu "Plot" in
  let file_menu_fact = new GMenu.factory file_menu ~accel_group in
  
  ignore (file_menu_fact#add_item "New" ~key:GdkKeysyms._N ~callback:(fun () -> plot_window {window with curves=[]}));
(*
  let close = fun () -> plotter#destroy () in
  ignore (file_menu_fact#add_item "Close" ~key:GdkKeysyms._W ~callback:close); *)
  let reset_item = file_menu_fact#add_item "Reset" ~key:GdkKeysyms._L in
  let suspend_item = file_menu_fact#add_item "Suspend" ~key:GdkKeysyms._S in
  let stop_item = file_menu_fact#add_item "Stop" ~key:GdkKeysyms._C in
  let start_item = file_menu_fact#add_item "Restart" ~key:GdkKeysyms._X in
  ignore (file_menu_fact#add_item "Quit" ~key:GdkKeysyms._Q ~callback:quit);
  let curves_menu = factory#add_submenu "Curves" in
  let curves_menu_fact = new GMenu.factory curves_menu in
  tooltips#set_tip reset_item#coerce ~text:"Reset the current display and the current data";
  tooltips#set_tip curves_menu#coerce ~text:"Delete the curve";
  tooltips#set_tip suspend_item#coerce ~text:"Freeze the display while the data are still updated";
  tooltips#set_tip stop_item#coerce ~text:"Freeze the data update while the display is active (e.g. resizable)";
  tooltips#set_tip start_item#coerce ~text:"UnFreeze";

  let h = GPack.hbox ~packing:vbox#pack () in

  let width = 900 and height = 200 in
  let plot = new plot ~size: !size ~width ~height ~packing:(vbox#pack ~expand:true) () in
  tooltips#set_tip plot#drawing_area#coerce ~text:"Drop a messages field here to draw it";
  ignore (plotter#connect#destroy ~callback:(fun () -> plot#destroy (); quit ()));

  (* Auto Scale *)
  let auto_scale = GButton.check_button ~label:"Auto Scale" ~active:true ~packing:h#pack () in
  let _, min_entry= labelled_entry ~width_chars:5 "Min" "" h in
  let _, max_entry= labelled_entry ~width_chars:5 "Max" "" h in
  min_entry#misc#set_sensitive false;
  max_entry#misc#set_sensitive false;
  ignore (auto_scale#connect#toggled (fun () -> let b = auto_scale#active in plot#set_auto_scale b; min_entry#misc#set_sensitive (not b); max_entry#misc#set_sensitive (not b)));
  ignore (GMain.Timeout.add 1000 (fun () -> if plot#auto_scale then begin min_entry#set_text (string_of_float plot#min);  max_entry#set_text (string_of_float plot#max) end; true));
  ignore (min_entry#connect#activate ~callback:(fun () -> if not plot#auto_scale then plot#set_min (float_of_string min_entry#text)));
  ignore (max_entry#connect#activate ~callback:(fun () -> if not plot#auto_scale then plot#set_max (float_of_string max_entry#text)));

  (* Update time slider *)
  let adj = GData.adjustment ~lower:0.1 ~value: !update_time ~step_incr:0.1 ~upper:11.0 () in
  let scale = GRange.scale `HORIZONTAL ~digits:2 ~adjustment:adj ~packing:h#add () in
  ignore (adj#connect#value_changed ~callback:(fun () -> plot#set_update_time adj#value));
  plot#set_update_time adj#value;
  tooltips#set_tip scale#coerce ~text:"Update time (s)";

  (* Size slider *)
  let adj = GData.adjustment ~lower:10. ~value:(float !size) ~step_incr:10. ~upper:1010. () in
  let scale = GRange.scale `HORIZONTAL ~digits:0 ~adjustment:adj ~packing:h#add () in
  ignore (adj#connect#value_changed ~callback:(fun () -> plot#set_size (truncate adj#value)));
  tooltips#set_tip scale#coerce ~text:"Memory size";

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

  (* Callbacks *)
  ignore (reset_item#connect#activate ~callback:plot#reset);
  ignore (suspend_item#connect#activate ~callback:plot#suspend);
  ignore (stop_item#connect#activate ~callback:plot#stop);
  ignore (start_item#connect#activate ~callback:plot#restart);


  let add_curve = fun ?(factor=1.) name ->
    let (sender, class_name, msg_name, field_name, factor') = parse_dnd name in
    let factor = factor *. factor' in
    let name = Printf.sprintf "%s:%f" name factor in
    let cb = fun _sender values ->
      let v = float_of_string (Pprz.string_assoc field_name values) *. factor in
      plot#add_value name v in
    
    let module P = Pprz.Messages (struct let name = class_name end) in
    let binding = 
      if sender = "*" then
	P.message_bind msg_name cb
      else
	P.message_bind ~sender msg_name cb in
    
    let curve = plot#create_curve name in
    let eb = GBin.event_box ~width:10 ~height:10 () in
    eb#coerce#misc#modify_bg [`NORMAL, `NAME curve.color];
    let item = curves_menu_fact#add_image_item ~image:eb#coerce ~label:name () in
      
    let delete = fun () ->
	plot#delete_curve name;
      Ivy.unbind binding;
      curves_menu#remove (item :> GMenu.menu_item) in
    ignore (item#connect#activate ~callback:delete) in

  (* Drag and drop handler *)
  let data_received = fun context ~x ~y data ~info ~time ->
    let factor =  float_of_string factor#text in
    try
      let name = data#data in
      add_curve ~factor name
    with
      exc -> prerr_endline (Printexc.to_string exc)
  in
  plotter#drag#dest_set dnd_targets ~actions:[`COPY];
  ignore (plotter#drag#connect#data_received ~callback:(data_received));

  (* Init curves *)
  List.iter add_curve window.curves;

  plotter#add_accel_group accel_group;
  plotter#show ()




let _ =
  let ivy_bus = ref "127.255.255.255:2010"
  and init = ref [default_window] in

  let add_init = fun s ->
    match !init with
      [] -> failwith "unreachable"
    | x::xs -> init := {x with curves = s::x.curves} :: xs in

  let set_title = fun s ->
    match !init with
      [] -> failwith "unreachable"
    | x::xs -> init := {x with title = s} :: xs in

  let set_geometry = fun s ->
    match !init with
      [] -> failwith "unreachable"
    | x::xs -> init := {x with geometry = s} :: xs in

  Arg.parse
    [ "-b", Arg.String (fun x -> ivy_bus := x), "<bus>  Bus\tDefault is 127.255.255.255:2010";
      "-c", Arg.String (fun x -> add_init x), "<curve>  Add a curve (e.g. '*:telemetry:BAT:voltage'). The curve is inserted into the last open window (cf -n option)";

      (* no code yet *)
      "-t", Arg.String set_title, "<title>  Set the last opened window title (cf -n option)";
      "-g", Arg.String set_geometry, "<geometry>  Set the last opened window geometry ( '500x500+100+100' )";

      "-n", Arg.Unit (fun () -> init := default_window :: !init), "Open another window for the next curves";
      "-m", Arg.Set_int size, (Printf.sprintf "<size>  Memory size (default %d)" !size);
      "-u", Arg.Set_float update_time, (Printf.sprintf "<time>  Update time in s (default %.2f)" !update_time)]
    (fun x -> prerr_endline ("WARNING: don't do anything with "^x))
    "Usage: ";

  (** Connect to the Ivy bus *)
  Ivy.init "Paparazzi messages" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  List.iter plot_window !init;

  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do ignore (Glib.Main.iteration true) done
