(*
 * Paparazzi widget renderers
 *
 * Copyright (C) 2008-2009 ENAC, Pascal Brisset
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
let (//) = Filename.concat

class type movable_item =
object
  inherit GnoCanvas.base_item
  method set : GnomeCanvas.group_p list -> unit
end

class type t =
object
  method tag : string
  method edit : (GObj.widget -> unit) -> unit
  method item : movable_item
  method update : string -> unit
  method config : unit -> Xml.xml list
end


(*************************** Text ***********************************)
class canvas_text = fun ?(config=[]) canvas_group x y ->
  let group = GnoCanvas.group ~x ~y canvas_group in
  let text = GnoCanvas.text ~text:"_" group in
object (self)
  val mutable format = PC.get_prop "format" config "%.2f"
  val mutable size = float_of_string (PC.get_prop "size" config "15.")
  val mutable color = PC.get_prop "color" config "#00ff00"

  method tag = "Text"
  method item = (group :> movable_item)
  method config = fun () ->
    [ PC.property "format" format;
      PC.float_property "size" size;
      PC.property "color" color ]
  method update = fun (value : string) ->
    let renderer = fun x ->
      try sprintf (Obj.magic format) (float_of_string x) with _ -> x in
    text#set [`SIZE_POINTS size; `TEXT (renderer value); `FILL_COLOR color; `ANCHOR `NW]


  method edit = fun (pack:GObj.widget -> unit) ->
    let file = Env.paparazzi_src // "sw" // "lib" // "ocaml" // "widgets.glade" in
    let text_editor = new Gtk_papget_text_editor.table_text_editor ~file () in
    pack text_editor#table_text_editor#coerce;

      (* Initialize the entries *)
    text_editor#entry_format#set_text format;
    text_editor#spinbutton_size#set_value size;
    text_editor#comboboxentry_color#set_active 0;

      (* Connect the entries *)
    let callback = fun () ->
      format <- text_editor#entry_format#text in
    ignore (text_editor#entry_format#connect#activate ~callback);
    let callback = fun () ->
      size <- text_editor#spinbutton_size#value in
    ignore (text_editor#spinbutton_size#connect#value_changed ~callback);
    let callback = fun () ->
      color <- text_editor#comboboxentry_color#entry#text in
    ignore (text_editor#comboboxentry_color#connect#changed ~callback);
end


(***************************Vertical Ruler ***********************************)
let affine_pos_and_angle xw yw angle =
  let cos_a = cos angle in
  let sin_a = sin angle in
  [| cos_a ; sin_a ; ~-. sin_a; cos_a; xw ; yw |]
let affine_pos xw yw = affine_pos_and_angle xw yw 0.


class canvas_ruler = fun ?(config=[]) canvas_group x y ->
  let h = float_of_string (PC.get_prop "height" config "100.")
  and index_on_right = bool_of_string (PC.get_prop "index_on_right" config "false")
  and point_per_unit = float_of_string (PC.get_prop "point_per_unit" config "2.")
  and w = float_of_string (PC.get_prop "width" config "32.")
  and step = int_of_string (PC.get_prop "step" config "10") in
  let text_props=[`ANCHOR `CENTER; `FILL_COLOR "white"]
  and index_width = 10. in

  let root = GnoCanvas.group ~x ~y canvas_group in
  let r = GnoCanvas.group root in

  let props = (text_props@[`ANCHOR `EAST]) in

  (* One step drawer *)
  let draw = fun i value ->
    let i = i * step in
    let y = -. point_per_unit *. (float i -. value) in
    if y >= -. h && y <= h then begin
      let text = Printf.sprintf "%d" i in
      ignore (GnoCanvas.text ~text ~props ~y ~x:(w*.0.75) r);
      ignore (GnoCanvas.line ~points:[|w*.0.8;y;w-.1.;y|] ~fill_color:"white" r)
    end;
    let y = y -. float step /. 2. *. point_per_unit in
    if y >= -. h && y <= h then
      ignore(GnoCanvas.line ~points:[|w*.0.8;y;w-.1.;y|] ~fill_color:"white" r)
  in

  let drawer = fun value ->
    (* Remove previous items *)
    List.iter (fun i -> i#destroy ()) r#get_items;
    let v = truncate value / step in
    let k = truncate (h /. point_per_unit) / step in
    for i = Pervasives.max 0 (v - k) to (v + k) do
      draw i value
    done
  in

  (** Yellow index *)
  let _ = GnoCanvas.line ~points:[|0.;0.;w-.1.;0.|] ~fill_color:"yellow" root in
  let s = index_width in
  let idx = GnoCanvas.polygon ~points:[|0.;0.;-.s;s/.2.;-.s;-.s/.2.|] ~fill_color:"yellow" root in
  let () =
    if index_on_right then
      idx#affine_absolute (affine_pos_and_angle w 0. Latlong.pi) in

object
  method tag = "Ruler"
  method edit = fun (pack:GObj.widget -> unit) -> ()
  method update = fun value ->
    let value = float_of_string value in
    drawer value
  method item = (root :> movable_item)
  method config = fun () ->
    [ PC.float_property "height" h;
      PC.property "index_on_right" (sprintf "%b" index_on_right);
      PC.float_property "width" w;
      PC.float_property "point_per_unit" point_per_unit;
      PC.property "step" (sprintf "%d" step) ]
end

(*************************** Gauge ***********************************)
class canvas_gauge = fun ?(config=[]) canvas_group x y ->
  let size = PC.get_prop "size" config "50." in
  (*let text_props = [`ANCHOR `CENTER; `FILL_COLOR "white"] in*)

  let r1 = Pervasives.max 10. ((float_of_string size) /. 2.) in
  let r2 = r1 +. 3. in
  let r3 = 3.5 in
  let max_rot = 2. *. Latlong.pi /. 3. in

  let root = GnoCanvas.group ~x ~y canvas_group in
  (*let gauge = GnoCanvas.group root in*)

  (*let props = (text_props@[`ANCHOR `EAST]) in*)

  let _ = GnoCanvas.ellipse ~x1:r2 ~y1:r2 ~x2:(-.r2) ~y2:(-.r2)
    ~props:[`NO_FILL_COLOR; `OUTLINE_COLOR "grey"; `WIDTH_PIXELS 6]  root in
  let points = [|0.;-.r1;0.;-.r1+.3.|] in
  let props = [`WIDTH_PIXELS 2; `FILL_COLOR "red"] in
  let _ = GnoCanvas.line ~points ~props root in
  let il = GnoCanvas.line ~points ~props root in
  let () = il#affine_absolute (affine_pos_and_angle 0. 0. (-. Latlong.pi /. 3.)) in
  let ill = GnoCanvas.line ~points ~props root in
  let () = ill#affine_absolute (affine_pos_and_angle 0. 0. (-. 2. *. Latlong.pi /. 3.)) in
  let ir = GnoCanvas.line ~points ~props root in
  let () = ir#affine_absolute (affine_pos_and_angle 0. 0. (Latlong.pi /. 3.)) in
  let irr = GnoCanvas.line ~points ~props root in
  let () = irr#affine_absolute (affine_pos_and_angle 0. 0. (2. *. Latlong.pi /. 3.)) in

  let idx = GnoCanvas.polygon ~points:[|r3-.0.2;0.;0.;-.r1;-.(r3-.0.2);0.|]
    ~props:[`FILL_COLOR "red"; `OUTLINE_COLOR "white"] root in
  let _ = GnoCanvas.ellipse ~x1:r3 ~y1:r3 ~x2:(-.r3) ~y2:(-.r3) ~props:[`OUTLINE_COLOR "grey"] ~fill_color:"red" root in
  let text_min = GnoCanvas.text ~x:(-.r1) ~y:(r1/.2.) ~props:[`ANCHOR `NE; `FILL_COLOR "#00ff00"] root in
  let text_max = GnoCanvas.text ~x:r1 ~y:(r1/.2.) ~props:[`ANCHOR `NW; `FILL_COLOR "#00ff00"] root in
  let text_mid = GnoCanvas.text ~x:0. ~y:(-.r2-.3.) ~props:[`ANCHOR `SOUTH; `FILL_COLOR "#00ff00"] root in
  let text_text = GnoCanvas.text ~x:0. ~y:(r2+.3.) ~props:[`ANCHOR `NORTH; `FILL_COLOR "#00ff00"] root in

object
  val mutable min = PC.get_prop "min" config "-50."
  val mutable max = PC.get_prop "max" config "50."
  val mutable text = PC.get_prop "text" config ""

  method tag = "Gauge"
  method edit = fun (pack:GObj.widget -> unit) ->
    let file = Env.paparazzi_src // "sw" // "lib" // "ocaml" // "widgets.glade" in
    let gauge_editor = new Gtk_papget_gauge_editor.table_gauge_editor ~file () in
    pack gauge_editor#table_gauge_editor#coerce;

      (* Initialize the entries *)
    gauge_editor#entry_min#set_text min;
    gauge_editor#entry_max#set_text max;
    gauge_editor#entry_text#set_text text;

      (* Connect the entries *)
    let callback = fun () ->
      min <- gauge_editor#entry_min#text in
    ignore (gauge_editor#entry_min#connect#activate ~callback);
    let callback = fun () ->
      max <- gauge_editor#entry_max#text in
    ignore (gauge_editor#entry_max#connect#activate ~callback);
    let callback = fun () ->
      text <- gauge_editor#entry_text#text in
    ignore (gauge_editor#entry_text#connect#activate ~callback);

  method update = fun value ->
    let value = float_of_string value in
      (* Gauge drawer *)
    let fmin = float_of_string min in
    let fmax = float_of_string max in
    let rot = ref (-.max_rot +. 2. *. max_rot *. (value -. fmin) /. (fmax -. fmin)) in
    if !rot > max_rot then rot := max_rot;
    if !rot < -.max_rot then rot := -.max_rot;
    idx#affine_absolute (affine_pos_and_angle 0. 0. !rot);
    text_min#set [`TEXT min];
    text_max#set [`TEXT max];
    text_mid#set [`TEXT (string_of_float ((fmin +. fmax)/.2.))];
    text_text#set [`TEXT text]

  method item = (root :> movable_item)
  method config = fun () ->
    [ PC.property "min" min;
      PC.property "max" max;
      PC.property "size" size;
      PC.property "text" text ]
end

(*************************** Led ***********************************)
class canvas_led = fun ?(config=[]) canvas_group x y ->
  let size = float_of_string (PC.get_prop "size" config "15.") in

  let root = GnoCanvas.group ~x ~y canvas_group in

  let r = (Pervasives.max 2. (size /. 2.)) +. 1. in
  let led = GnoCanvas.ellipse ~x1:r ~y1:r ~x2:(-.r) ~y2:(-.r)
    ~props:[`NO_FILL_COLOR; `OUTLINE_COLOR "grey"; `WIDTH_UNITS 2.]  root in

  let led_text = GnoCanvas.text ~x:(-.r-.3.) ~y:0. ~props:[`ANCHOR `EAST; `FILL_COLOR "#00ff00"] root in

object
  val mutable size = float_of_string (PC.get_prop "size" config "15.")
  val mutable text = PC.get_prop "text" config ""
  val mutable test_value = float_of_string (PC.get_prop "test_value" config "0.")
  val mutable test_inv = bool_of_string (PC.get_prop "test_invert" config "false")

  method tag = "Led"
  method edit = fun (pack:GObj.widget -> unit) ->
    let file = Env.paparazzi_src // "sw" // "lib" // "ocaml" // "widgets.glade" in
    let led_editor = new Gtk_papget_led_editor.table_led_editor ~file () in
    pack led_editor#table_led_editor#coerce;

      (* Initialize the entries *)
    led_editor#entry_text#set_text text;
    led_editor#spinbutton_size#set_value size;
    led_editor#spinbutton_test#set_value test_value;

      (* Connect the entries *)
    let callback = fun () ->
      text <- led_editor#entry_text#text in
    ignore (led_editor#entry_text#connect#activate ~callback);
    let callback = fun () ->
      size <- led_editor#spinbutton_size#value in
    ignore (led_editor#spinbutton_size#connect#activate ~callback);
    let callback = fun () ->
      test_value <- led_editor#spinbutton_test#value in
    ignore (led_editor#spinbutton_test#connect#activate ~callback);
    let callback = fun () ->
      test_inv <- led_editor#check_invert#active in
    ignore (led_editor#check_invert#connect#toggled ~callback);

  method update = fun value ->
    let value = float_of_string value in
    let inv = if test_inv then not else (fun x -> x) in
      (* Led drawer *)
    if inv (value = test_value) then led#set [`FILL_COLOR "red"]
    else led#set [`FILL_COLOR "#00ff00"];
    let r = (Pervasives.max 2. (size /. 2.)) +. 1. in
    led#set [`X1 r; `Y1 r; `X2 (-.r); `Y2 (-.r)];
    led_text#set [`TEXT text; `SIZE_POINTS size; `X (-.r-.3.)]

  method item = (root :> movable_item)
  method config = fun () ->
    [ PC.float_property "size" size;
      PC.property "text" text ]
end

(****************************************************************************)
class canvas_button = fun ?(config=[]) canvas_group x y ->
  let icon = PC.get_prop "icon" config "icon_file" in
  let pixbuf = GdkPixbuf.from_file (Env.gcs_icons_path // icon) in
  let group = GnoCanvas.group ~x ~y canvas_group in
  let _item = GnoCanvas.pixbuf ~pixbuf group in
object
  method tag = "Button"
  method item = (group :> movable_item)
  method edit = fun (pack:GObj.widget -> unit) -> ()
  method update = fun (value:string) -> ()
  method config = fun () ->
    [ PC.property "icon" icon]
  initializer
    group#raise_to_top ();
end


(****************************************************************************)
class canvas_mplayer = fun ?(config=[]) canvas_group x y ->
  let video_feed = PC.get_prop "video_feed" config "video_URI" in
  let width = float_of_string (PC.get_prop "width" config "320.")
  and height = float_of_string (PC.get_prop "height" config "240.") in
  let socket = GWindow.socket () in
  let group = GnoCanvas.group ~x ~y canvas_group in
  let item = GnoCanvas.widget ~width ~height ~widget:socket group in

object
  method tag = "Mplayer"
  method item = (group :> movable_item)
  method edit = fun (pack:GObj.widget -> unit) -> ()
  method update = fun (value:string) ->
    let zoom = try float_of_string value with _ -> 1. in
    item#set [`WIDTH (width /. zoom); `HEIGHT (height /. zoom)]
  method config = fun () ->
    [ PC.property "video_feed" video_feed;
      PC.float_property "width" width;
      PC.float_property "height" height ]
  initializer
    group#lower_to_bottom ();
    let com = sprintf "exec mplayer -vo xv -really-quiet -nomouseinput %s -wid 0x%lx -geometry %.0fx%.0f" video_feed socket#xwindow width height in
    let dev_null = Unix.descr_of_out_channel (open_out "/dev/null") in
    ignore (Unix.create_process "/bin/sh" [|"/bin/sh"; "-c"; com|] dev_null dev_null dev_null)
end


(****************************************************************************)
class canvas_plugin = fun ?(config=[]) canvas_group x y ->
  let command = PC.get_prop "command" config "missing_plugin_command" in
  let width = float_of_string (PC.get_prop "width" config "320.")
  and height = float_of_string (PC.get_prop "height" config "240.") in
  let socket = GWindow.socket () in
  let group = GnoCanvas.group ~x ~y canvas_group in
  let item = GnoCanvas.widget ~width ~height ~widget:socket group in

object
  method tag = "Plugin"
  method item = (group :> movable_item)
  method edit = fun (pack:GObj.widget -> unit) -> ()
  method update = fun (value:string) ->
    let zoom = try float_of_string value with _ -> 1. in
    item#set [`WIDTH (width /. zoom); `HEIGHT (height /. zoom) ]
  method config = fun () ->
    [ PC.property "command" command;
      PC.float_property "width" width;
      PC.float_property "height" height ]
  initializer
    group#lower_to_bottom ();
    let com = sprintf "exec %s0x%lx" command socket#xwindow in
    let dev_null = Unix.descr_of_out_channel (open_out "/dev/null") in
    ignore (Unix.create_process "/bin/sh" [|"/bin/sh"; "-c"; com|] dev_null dev_null dev_null)
end



let renderers =
  [ (new canvas_text :> ?config:Xml.xml list -> #GnoCanvas.group -> float -> float -> t);
    (new canvas_ruler :> ?config:Xml.xml list -> #GnoCanvas.group -> float -> float -> t);
    (new canvas_gauge :> ?config:Xml.xml list -> #GnoCanvas.group -> float -> float -> t);
    (new canvas_led :> ?config:Xml.xml list -> #GnoCanvas.group -> float -> float -> t) ]

let lazy_tagged_renderers = lazy
  (let x = 0. and y = 0.
  and group = (GnoCanvas.canvas ())#root in
   List.map
     (fun constructor ->
       let o = constructor ?config:None group x y in
       (o#tag, constructor))
     renderers)

