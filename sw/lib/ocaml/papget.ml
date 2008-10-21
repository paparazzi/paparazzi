open Printf

let affine_pos_and_angle xw yw angle =
  let cos_a = cos angle in
  let sin_a = sin angle in
  [| cos_a ; sin_a ; ~-. sin_a; cos_a; xw ; yw |]
let affine_pos xw yw = affine_pos_and_angle xw yw 0.

let (//) = Filename.concat

class message = fun ?sender ?(class_name="telemetry") msg_name ->
  object
    val mutable callbacks = []
    method connect = fun f cb -> callbacks <- (f, cb) :: callbacks
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
    method update = fun value -> last_val <- value
    initializer
      msg_obj#connect field_name self#update
  end


class type movable_item =
    object
      inherit GnoCanvas.base_item
      method set : GnomeCanvas.group_p list -> unit
    end



class canvas_text = fun ?(text="") canvas_group x y ->
  let text = GnoCanvas.text ~x ~y ~text canvas_group in
  object (self)
    val mutable format = "%.2f"
    val mutable size = 15.
 
    method item = text
    method set_format = fun f -> format <- f 
    method update = fun value ->
      let renderer = fun x -> sprintf (Obj.magic format) (float_of_string x) in
      text#set [`SIZE_POINTS size; `TEXT (renderer value); `FILL_COLOR "green"; `ANCHOR `NW]


    method edit = fun (pack:GObj.widget -> unit) ->
      let file = Env.paparazzi_src // "sw" // "lib" // "ocaml" // "widgets.glade" in
      let text_editor = new Gtk_papget_text_editor.table_text_editor ~file () in
      pack text_editor#table_text_editor#coerce;

      (* Initialize the entries *)
      text_editor#entry_format#set_text format;
      text_editor#spinbutton_size#set_value size;
      
      (* Connect the entries *)
      let callback = fun () ->
	format <- text_editor#entry_format#text in
      ignore (text_editor#entry_format#connect#activate ~callback);
      let callback = fun () ->
	size <- text_editor#spinbutton_size#value in
      ignore (text_editor#spinbutton_size#connect#value_changed ~callback);
  end


(********************************* Ruler *************************************)
class canvas_ruler = fun ?(index_on_right=false) ?(text_props=[`ANCHOR `CENTER; `FILL_COLOR "white"]) ?(max=1000) ?(scale=2.) ?(w=32.) ?(index_width=10.) ?(step=10) ?(h=100.) canvas_group x y ->
  let root = GnoCanvas.group ~x ~y canvas_group in
  let r = GnoCanvas.group root in
  let height = scale *. float max in
  
  (* Grey background *)
  let _ = GnoCanvas.rect ~x1:0. ~y1:(-.height) ~x2:w ~y2:height ~fill_color:"#808080" r in
  let props = (text_props@[`ANCHOR `EAST]) in
  
  (* One step drawer *)
  let tab = Array.create (max/step) false in
  let draw = fun i ->
    let i = i * step in
    let y = -. scale *. float i in
    let text = Printf.sprintf "%d" i in
    let _ = GnoCanvas.text ~text ~props ~y ~x:(w*.0.75) r in
    let _ = GnoCanvas.line ~points:[|w*.0.8;y;w-.1.;y|] ~fill_color:"white" r in
    let y = y -. float step /. 2. *. scale in
    let _ = GnoCanvas.line ~points:[|w*.0.8;y;w-.1.;y|] ~fill_color:"white" r in
    () in
  
  let lazy_drawer = fun v ->
    Printf.printf "v=%f\n%!" v;
    let v = truncate v / step in
    let k = truncate (h /. scale) / step in
    for i = Pervasives.max 0 (v - k) to min (v + k) (Array.length tab - 1) do (* FIXME *)
      if not tab.(i) then begin
	tab.(i) <- true;
	draw i
      end
    done in
  
  (** Yellow index *)
  let _ = GnoCanvas.line ~points:[|0.;0.;w-.1.;0.|] ~fill_color:"yellow" root in
  let s = index_width in
  let idx = GnoCanvas.polygon ~points:[|0.;0.;-.s;s/.2.;-.s;-.s/.2.|] ~fill_color:"yellow" root in
  let () = 
    if index_on_right then
      idx#affine_absolute (affine_pos_and_angle w 0. Latlong.pi) in
  
  (** Mask (bottom & top) *)
  let _ = GnoCanvas.rect ~x1:0. ~y1:(-.height) ~x2:w ~y2:(-.h) ~fill_color:"black" root in
  let _ = GnoCanvas.rect ~x1:0. ~y1:height ~x2:w ~y2:h ~fill_color:"black" root in
  
  object
    method edit = fun (pack:GObj.widget -> unit) -> ()
    method update = fun value ->
      let value = float_of_string value in
      r#affine_absolute (affine_pos 0. 0.);
      lazy_drawer value;
      r#affine_absolute (affine_pos 0. (scale*.value));
    method item = root
  end
    





class canvas_item = fun msg_obj field_name canvas_renderer ->
  object (self)
    inherit field msg_obj field_name as super

    val mutable motion = false
    val mutable renderer = canvas_renderer

    method update = fun value ->
      super#update value;
      renderer#update value

    method event = fun (ev : GnoCanvas.item_event) ->
      let item = (renderer#item :> movable_item) in
      match ev with
	`BUTTON_PRESS ev ->
	  begin
	    match GdkEvent.Button.button ev with
	    | 1 ->
		motion <- false;
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
	    item#set [`X xw; `Y yw]
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

      let strings = ["Text"; "Ruler"] in

      let _ = GEdit.combo_box_text ~packing:dialog#box_item_chooser#add ~strings () in
      
      renderer#edit dialog#box_item_editor#add;

      (* Connect the buttons *)
      ignore (dialog#button_ok#connect#clicked (fun () -> dialog#papget_editor#destroy ()))


    initializer
      let item = (renderer#item :> movable_item) in
      ignore (item#connect#event self#event)
  end
