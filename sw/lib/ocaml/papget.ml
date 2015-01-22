(*
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
module E = Expr_syntax
let (//) = Filename.concat

class type item = object
  method config : unit -> Xml.xml
  method deleted : bool
end

class type value =
object
  method last_value : string
  method connect : (string -> unit) -> unit
  method config : unit -> Xml.xml list
  method type_ : string
end



(** [index_of_fields s] Returns i if s matches x[i] else 0. *)
let base_and_index =
  let field_regexp = Str.regexp "\\([^\\.]+\\)\\[\\([0-9]+\\)\\]" in
  fun field_descr ->
    if Str.string_match field_regexp field_descr 0 then
      ( Str.matched_group 1 field_descr,
        int_of_string (Str.matched_group 2 field_descr))
    else
      (field_descr, 0)


class message_field = fun ?sender ?(class_name="telemetry") msg_name field_descr ->
object
  val mutable callbacks = []
  val mutable last_value = "0."

  method last_value = last_value

  method connect = fun cb -> callbacks <- cb :: callbacks
  method config = fun () ->
    let field = sprintf "%s:%s" msg_name field_descr in
    let ac_id = match sender with None -> [] | Some id -> [PC.property "ac_id" id] in
    [ PC.property "field" field ] @ ac_id
  method type_ = "message_field"

  initializer
  let module P = Pprz.Messages (struct let name = class_name end) in
  let process_message = fun _sender values ->
    let (field_name, index) = base_and_index field_descr in
    let value =
      match Pprz.assoc field_name values with
          Pprz.Array array -> array.(index)
        | scalar -> scalar in

    last_value <- Pprz.string_of_value value;

    List.iter (fun cb -> cb last_value) callbacks in
  ignore (P.message_bind ?sender msg_name process_message)
end


let hash_vars = fun ?sender expr ->
  let htable = Hashtbl.create 3 in
  let rec loop = function
      E.Ident i -> prerr_endline i
    | E.Int _ | E.Float _ -> ()
    | E.Call (_id, list) | E.CallOperator (_id, list) -> List.iter loop list
    | E.Index (_id, e) -> loop e
    | E.Deref (_e, _f) as deref -> fprintf stderr "Warning: Deref operator is not allowed in Papgets expressions (%s)" (E.sprint deref)
    | E.Field (i, f) ->
      if not (Hashtbl.mem htable (i,f)) then
        let msg_obj = new message_field ?sender i f in
        Hashtbl.add htable (i, f) msg_obj in
  loop expr;
  htable


let wrap = fun f ->
  fun x y -> string_of_float (f (float_of_string x) (float_of_string y))
let eval_bin_op = function
  | "*" -> wrap ( *. )
  | "+" -> wrap ( +. )
  | "-" -> wrap ( -. )
  | "/" -> wrap ( /. )
  | "**" -> wrap ( ** )
  | op -> failwith (sprintf "Papget.eval_expr '%s'" op)

let eval_expr = fun (extra_functions:(string * (string list -> string)) list) h e ->
  let rec loop = function
      E.Ident ident -> failwith (sprintf "Papget.eval_expr '%s'" ident)
    | E.Int int -> string_of_int int
    | E.Float float -> string_of_float float
    | E.CallOperator (ident, [e1; e2]) ->
      eval_bin_op ident (loop e1) (loop e2)
    | E.Call (ident, args) when List.mem_assoc ident extra_functions ->
      (List.assoc ident extra_functions) (List.map loop args)
    | E.Call (ident, _l) | E.CallOperator (ident, _l) ->
      failwith (sprintf "Papget.eval_expr '%s(...)'" ident)
    | E.Index (ident, _e) -> failwith (sprintf "Papget.eval_expr '%s[...]'" ident)
    | E.Deref (_e, _f) as deref -> failwith (sprintf "Papget.eval_expr Deref operator is not allowed in Papgets expressions (%s)" (E.sprint deref))
    | E.Field (i, f) ->
      try
        (Hashtbl.find h (i,f))#last_value
      with
          Not_found -> failwith (sprintf "Papget.eval_expr '%s.%s'" i f)
  in loop e



class expression = fun ?(extra_functions=[]) ?sender expr ->
  let h = hash_vars ?sender expr in
object
  val mutable callbacks = []
  val mutable last_value = "0."

  method last_value = last_value

  method connect = fun cb -> callbacks <- cb :: callbacks

  method config = fun () ->
    let ac_id = match sender with None -> [] | Some id -> [PC.property "ac_id" id] in
    [ PC.property "expr" (Expr_syntax.sprint expr)] @ ac_id

  method type_ = "expression"

  initializer
    Hashtbl.iter
      (fun (i,f) (msg_obj:value) ->
        let val_updated = fun _new_val ->
          last_value <- eval_expr extra_functions h expr;
          List.iter (fun cb -> cb last_value) callbacks
        in
        msg_obj#connect val_updated)
      h
end




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


class canvas_item = fun ~config canvas_renderer ->
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
    try
      (renderer#update:string->unit) value
    with
        exc -> prerr_endline (Printexc.to_string exc)

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
          (* get item and window size *)
          let bounds = item#get_bounds in
          let w, h = Gdk.Drawable.get_size item#canvas#misc#window in
          if not motion then begin
            self#edit ()
          end
          else if (truncate bounds.(0) > w)  || (truncate bounds.(2)  < 0) || (truncate bounds.(1) > h) || (truncate bounds.(3) < 0) then begin
            (* delete an item if placed out of the window on the left or top side *)
            item#destroy ();
            deleted <- true
          end;
          motion <- false
        end;
        true
      | _ -> false

  method edit = fun () ->
    let file = Env.paparazzi_src // "sw" // "lib" // "ocaml" // "widgets.glade" in
    let dialog = new Gtk_papget_editor.papget_editor ~file () in

    let ac_id = PC.get_prop "ac_id" config "Any" in
    dialog#toplevel#set_title ("Papget Editor (A/C: "^ac_id^")");

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
    if PC.get_prop "locked" config "false" = "false" then
      let item = (renderer#item :> PR.movable_item) in
      connection <- item#connect#event self#event

  initializer
    self#connect ()
end

class canvas_float_item = fun ~config canvas_renderer ->
object
  inherit canvas_item ~config canvas_renderer as super

  val mutable affine = "1"

  method update = fun value ->
    let scaled_value =
      try
        let (a, b) =  Ocaml_tools.affine_transform affine
        and fvalue = float_of_string value in
        string_of_float (fvalue *. a +. b)
      with
          _ -> value in
    super#update scaled_value

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


class canvas_display_float_item = fun ~config (msg_obj:value) (canvas_renderer:PR.t) ->
object (self)
  inherit canvas_float_item ~config canvas_renderer as item

  initializer
    affine <- PC.get_prop "scale" config "1";
    msg_obj#connect self#update_field

  method update_field = fun value ->
    if not deleted then begin
      item#update value
    end

  method config = fun () ->
    let renderer_props = renderer#config ()
    and val_props = msg_obj#config ()
    and scale_prop = PC.property "scale" affine in
    let (x, y) = item#xy in
    let attrs =
      [ "type", msg_obj#type_;
        "display", String.lowercase item#renderer#tag;
        "x", sprintf "%.0f" x; "y", sprintf "%.0f" y ] in
    Xml.Element ("papget", attrs, scale_prop::val_props@renderer_props)
end


(****************************************************************************)
(** A clickable item is not editable: The #edit method is overiden with a
    provided callback *)
class canvas_clickable_item = fun type_ properties callback canvas_renderer ->
object
  inherit canvas_item ~config:properties canvas_renderer as item
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



(****************************************************************************)
class canvas_video_plugin_item = fun properties (canvas_renderer:PR.t) ->
object
  inherit canvas_item ~config:properties canvas_renderer as item
  method config = fun () ->
    let props = renderer#config () in
    let (x, y) = item#xy in
    let attrs =
      [ "type", "video_plugin";
        "display", String.lowercase item#renderer#tag;
        "x", sprintf "%.0f" x; "y", sprintf "%.0f" y ] in
    Xml.Element ("papget", attrs, properties@props)
end

