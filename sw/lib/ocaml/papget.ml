(*
 * $Id$
 *
 * Paparazzi widgets
 *  
 * Copyright (C) 2008 ENAC
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
module PC = Papget_common
module PR = Papget_renderer
let (//) = Filename.concat

class type item = object
  method config : unit -> Xml.xml
  method deleted : bool
end

class message = fun ?sender ?(class_name="telemetry") msg_name ->
  object
    val mutable callbacks = []
    method connect = fun f cb -> callbacks <- (f, cb) :: callbacks
    method msg_name = msg_name
    initializer
      let module P = Pprz.Messages (struct let name = class_name end) in
      let cb = fun _sender values -> 
	List.iter 
	  (fun (field_name, cb) -> 
	    let field = Pprz.string_assoc field_name values in
	    cb field) 
	  callbacks  in
      ignore (P.message_bind ?sender msg_name cb)
  end

class field = fun msg_obj field_name ->
  object (self)
    val mutable last_val = ""
    method update_field = fun value -> last_val <- value
    initializer
      msg_obj#connect field_name self#update_field
  end


let regexp_plus = Str.regexp "\\+" 
let affine_transform = fun format value ->
  let value = float_of_string value in
  let a, b =
    match Str.split regexp_plus format with
      [a;b] -> float_of_string a, float_of_string b
    | [a] -> float_of_string a, 0.
    | _ -> 1., 0. in
  string_of_float (value *. a +. b)

class type canvas_item_type = 
  object
    method connect : unit -> unit
    method deleted : bool
    method edit : unit -> unit
    method event : GnoCanvas.item_event -> bool
    method renderer : Papget_renderer.t
    method update : string -> unit
    method xy : float * float
  end
    

class canvas_item = fun canvas_renderer ->
  let canvas_renderer = (canvas_renderer :> PR.t) in
  object (self)
    val mutable motion = false
    val mutable renderer = canvas_renderer
    val mutable x_press = 0.
    val mutable y_press = 0.
    val mutable deleted = false
    val mutable dialog_widget = None
	
    method renderer = renderer

    method xy = 
      let (x0, y0) = renderer#item#i2w 0. 0. in
      renderer#item#parent#w2i x0 y0

    method deleted = deleted

    method update = fun value ->
      (renderer#update:string->unit) value

    method event = fun (ev : GnoCanvas.item_event) ->
      let item = (renderer#item :> PR.movable_item) in
      match ev with
	`BUTTON_PRESS ev ->
	  begin
	    match GdkEvent.Button.button ev with
	    | 1 ->
		motion <- false;
		let x = GdkEvent.Button.x ev and y = GdkEvent.Button.y ev in
		let (xm, ym) = renderer#item#parent#w2i x y in
		let (x0, y0) = renderer#item#i2w 0. 0. in
		let (xi, yi) = renderer#item#parent#w2i x0 y0 in
		x_press <- xm -. xi; y_press <- ym -. yi;
		let curs = Gdk.Cursor.create `FLEUR in
		item#grab [`POINTER_MOTION; `BUTTON_RELEASE] curs 
		  (GdkEvent.Button.time ev)
	    | _ -> ()
	  end;
	  true
      | `MOTION_NOTIFY ev ->
	  let state = GdkEvent.Motion.state ev in
	  if Gdk.Convert.test_modifier `BUTTON1 state then begin
	    motion <- true;
	    let x = GdkEvent.Motion.x ev
	    and y = GdkEvent.Motion.y ev in
	    let (xw, yw) = renderer#item#parent#w2i x y in
	    item#set [`X (xw-.x_press); `Y (yw-.y_press)];
	    renderer#item#parent#affine_relative [|1.;0.;0.;1.;0.;0.|]
	  end;
	  true
      | `BUTTON_RELEASE ev ->
	  if GdkEvent.Button.button ev = 1 then begin
	    item#ungrab (GdkEvent.Button.time ev);
	    if not motion then begin
	      self#edit ()
	    end;
	    motion <- false
	  end;
	  true
      | _ -> false

    method edit = fun () ->
      let file = Env.paparazzi_src // "sw" // "lib" // "ocaml" // "widgets.glade" in
      let dialog = new Gtk_papget_editor.papget_editor ~file () in

      let tagged_renderers = Lazy.force PR.lazy_tagged_renderers in
      let strings = List.map fst tagged_renderers in

      let (combo, (tree, column)) = GEdit.combo_box_text ~packing:dialog#box_item_chooser#add ~strings () in
      tree#foreach 
	(fun _path row ->
	  if tree#get ~row ~column = renderer#tag then begin
	    combo#set_active_iter (Some row);
	    true
	  end else
	    false);
      
      let connect_item_editor = fun () ->
	begin (* Remove the current child ? *)
	  try
	    let child = dialog#box_item_editor#child  in
	    dialog#box_item_editor#remove child
	  with
	    Gpointer.Null -> ()
	end;
	renderer#edit dialog#box_item_editor#add in

      connect_item_editor ();

      (* Connect the renderer chooser *)
      ignore (combo#connect#changed
		(fun () ->
		  match combo#active_iter with
		  | None -> ()
		  | Some row ->
		      let data = combo#model#get ~row ~column in
		      if data <> renderer#tag then
			let new_renderer = List.assoc data tagged_renderers in
			let group = renderer#item#parent in
			let (x, y) = renderer#item#i2w 0. 0. in
			let (x, y) = group#w2i x y in
			renderer#item#destroy ();
			renderer <- new_renderer group x y;
			self#connect ();
			connect_item_editor ()));

      (* Connect the buttons *)
      ignore (dialog#button_delete#connect#clicked
		(fun () -> 
		  dialog#papget_editor#destroy ();
		  renderer#item#destroy ();
		  deleted <- true));
      ignore (dialog#button_ok#connect#clicked (fun () -> dialog#papget_editor#destroy ()));

      dialog_widget <- Some dialog

    val mutable connection =
      canvas_renderer#item#connect#event (fun _ -> false)
    method connect = fun () ->
      let item = (renderer#item :> PR.movable_item) in
      connection <- item#connect#event self#event

    initializer
      self#connect ()
  end

class canvas_float_item = fun canvas_renderer ->
  object
    inherit canvas_item canvas_renderer as super

    val mutable affine = "1"
	
    method update = fun value ->
      super#update (affine_transform affine value)

    method edit = fun () ->
      super#edit ();
      match dialog_widget with
	None -> ()
      | Some dialog ->
	  (* Set the current value *)
	  dialog#entry_scale#set_text affine;
          (* Connect the scale entry *)
	  let callback = fun () ->
	    affine <- dialog#entry_scale#text in
	  ignore (dialog#entry_scale#connect#activate ~callback);
	  dialog#hbox_scale#misc#show ()
  end


class canvas_display_float_item = fun ~config (msg_obj:message) field_name (canvas_renderer:PR.t) ->
  object
    inherit field msg_obj field_name as super
    inherit canvas_float_item canvas_renderer as item

    initializer
      affine <- PC.get_prop "scale" config "1"

    method update_field = fun value ->
      if not deleted then begin
	super#update_field value;
	item#update value
      end

    method config = fun () ->
      let props = renderer#config () in
      let field = sprintf "%s:%s" msg_obj#msg_name field_name in
      let field_prop = PC.property "field" field
      and scale_prop = PC.property "scale" affine in
      let (x, y) = item#xy in
      let attrs =
	[ "type", "message_field";
	  "display", String.lowercase item#renderer#tag;
	  "x", sprintf "%.0f" x; "y", sprintf "%.0f" y ] in
      Xml.Element ("papget", attrs, field_prop::scale_prop::props)
  end


(****************************************************************************)
class canvas_setting_item = fun variable canvas_renderer ->
  object
    inherit canvas_float_item canvas_renderer as item

    method clicked = fun value ->
      (variable#set : float -> unit) value

    initializer
      variable#connect item#update
  end


(****************************************************************************)
(** A clickable item is not editable: The #edit method is overiden with a
    provided callback *)
class canvas_clickable_item = fun type_ properties callback canvas_renderer ->
  object
    inherit canvas_item canvas_renderer as item
    method edit = fun () -> callback () 

    method config = fun () ->
      let props = renderer#config () in
      let (x, y) = item#xy in
      let attrs =
	[ "type", type_;
	  "display", String.lowercase item#renderer#tag;
	  "x", sprintf "%.0f" x; "y", sprintf "%.0f" y ] in
      Xml.Element ("papget", attrs, properties@props)
  end


class canvas_goto_block_item = fun properties callback (canvas_renderer:PR.t) ->
  object
    inherit canvas_clickable_item "goto_block" properties callback canvas_renderer as item
  end

class canvas_variable_setting_item = fun properties callback (canvas_renderer:PR.t) ->
  object
    inherit canvas_clickable_item "variable_setting" properties callback canvas_renderer
  end



let dnd_source = fun (widget:GObj.widget) papget_xml ->
  let dnd_targets = [ { Gtk.target = "STRING"; flags = []; info = 0} ] in
  widget#drag#source_set dnd_targets ~modi:[`BUTTON1] ~actions:[`COPY];
  let data_get = fun _ (sel:GObj.selection_context) ~info ~time ->
    sel#return (Xml.to_string papget_xml) in
  ignore (widget#drag#connect#data_get ~callback:data_get);
