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


module P = Pprz.Messages (struct let name = "telemetry" end)


let double__ = 
  let underscore = Str.regexp "_" in
  fun s -> Str.global_replace underscore "__" s


let (//) = Filename.concat

let colors = [|"red"; "blue"; "green"; "orange"; "purple"; "magenta"|]

let labelled_entry = fun ?width_chars text value (h:GPack.box) ->
  let label = GMisc.label ~text ~packing:h#pack () in
  label, GEdit.entry ?width_chars ~text:value ~packing:h#pack ()

class plot = fun ~width ~height ~packing () ->
  let da = GMisc.drawing_area ~width ~height ~show:true ~packing () in
  let curves = Hashtbl.create 3 in
  object (self)
    val mutable min_x = max_float
    val mutable max_x = min_float
    val mutable min = max_float
    val mutable max = min_float
    val mutable color_index = 0
    val mutable csts = ([] : float list)
    val mutable auto_scale = true

    method auto_scale = auto_scale
    method set_auto_scale = fun x -> auto_scale <- x
    method min = min
    method set_min = fun x -> min <- x
    method max = max
    method set_max = fun x -> max <- x

    method destroy = fun () -> ()

    method drawing_area = da

    method add_cst = fun v ->
      csts <- v :: csts

    method delete_cst = fun v ->
      csts <- List.filter (fun x -> x <> v) csts

    method add_curve = fun (name:string) (values:(float*float) array) ->
      Hashtbl.add curves name values
	
    method delete_curve = fun (name:string) ->
      Hashtbl.remove curves name

    method update_curves = fun () ->
      let {Gtk.width=width; height=height} = da#misc#allocation in
      let dr = GDraw.pixmap ~width ~height ~window:da () in
      dr#set_foreground (`NAME "white");
      dr#rectangle ~x:0 ~y:0 ~width ~height ~filled:true ();

      (* Time Graduations *)
      let context = da#misc#create_pango_context in
      context#set_font_by_name "sans 10 ";
      let layout = context#create_layout in
      
      let f = fun x y s ->
	Pango.Layout.set_text layout s;
	let (w, h) = Pango.Layout.get_pixel_size layout in
	dr#put_layout ~x ~y:(y-h) ~fore:`BLACK layout in
      ()
  end


(* Indexed by log *)
let logs = Hashtbl.create 3

let pprz_float = function
    Pprz.Int i -> float i
  | Pprz.Float f -> f 
  | Pprz.Int32 i -> Int32.to_float i
  | Pprz.String s -> float_of_string s
  | Pprz.Array _ -> failwith "pprz_float"


let load_log = fun (plot:plot) (menubar:GMenu.menu_shell GMenu.factory) xml_file ->
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
	  let menu = menubar#add_submenu menu_name in
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
		  let callback = fun _ -> plot#add_curve name values; plot#update_curves () in
		  ignore (menu_fact#add_item ~callback (double__ f)))
		l
	      )
	    l
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


let open_log = fun plot menubar () ->
  ignore (file_dialog ~title:"Open Log" ~callback:(fun name -> load_log plot menubar name) ())


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

  ignore (file_menu_fact#add_item "Open Log" ~key:GdkKeysyms._O ~callback:(open_log plot factory));  
  
  ignore (file_menu_fact#add_item "New" ~key:GdkKeysyms._N ~callback:(fun () -> plot_window []));
(*
  let close = fun () -> plotter#destroy () in
  ignore (file_menu_fact#add_item "Close" ~key:GdkKeysyms._W ~callback:close); *)

  ignore (file_menu_fact#add_item "Quit" ~key:GdkKeysyms._Q ~callback:quit);
  let curves_menu = factory#add_submenu "Curves" in
  let curves_menu_fact = new GMenu.factory curves_menu in

  let h = GPack.hbox ~packing:vbox#pack () in

  tooltips#set_tip plot#drawing_area#coerce ~text:"Drop a messages field here to draw it";
  ignore (plotter#connect#destroy ~callback:(fun () -> plot#destroy ()));

  (* Auto Scale *)
  let auto_scale = GButton.check_button ~label:"Auto Scale" ~active:true ~packing:h#pack () in
  ignore (auto_scale#connect#toggled (fun () -> plot#set_auto_scale auto_scale#active));
  let _, min_entry= labelled_entry ~width_chars:5 "Min" "" h in
  let _, max_entry= labelled_entry ~width_chars:5 "Max" "" h in
  ignore (GMain.Timeout.add 1000 (fun () -> if plot#auto_scale then begin min_entry#set_text (string_of_float plot#min);  max_entry#set_text (string_of_float plot#max) end; true));
  ignore (min_entry#connect#activate ~callback:(fun () -> if not plot#auto_scale then plot#set_min (float_of_string min_entry#text)));
  ignore (max_entry#connect#activate ~callback:(fun () -> if not plot#auto_scale then plot#set_max (float_of_string max_entry#text)));

  (* X range *)
  let _, _x_min_entry= labelled_entry ~width_chars:5 "Min" "" h in
  let _, _x_max_entry= labelled_entry ~width_chars:5 "Max" "" h in

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

  List.iter (fun f -> load_log plot factory f) init;

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
