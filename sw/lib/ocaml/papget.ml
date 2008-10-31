open Printf

let affine_pos_and_angle xw yw angle =
  let cos_a = cos angle in
  let sin_a = sin angle in
  [| cos_a ; sin_a ; ~-. sin_a; cos_a; xw ; yw |]
let affine_pos xw yw = affine_pos_and_angle xw yw 0.

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


class type movable_item =
    object
      inherit GnoCanvas.base_item
      method set : GnomeCanvas.group_p list -> unit
    end


class type renderer =
  object
    method tag : string
    method edit : (GObj.widget -> unit) -> unit
    method item : movable_item
    method update : string -> unit
    method config : unit -> Xml.xml list
  end

let get_property = fun attr_name xml ->
  let attr = ExtXml.child ~select:(fun x -> ExtXml.attrib x "name" = attr_name) xml "property" in
  ExtXml.attrib attr "value"


let get_prop = fun name children default ->
  let xml = Xml.Element ("", [], children) in
  try get_property name xml with _ -> default

let property = fun name value ->
  Xml.Element("property", [ "name", name; "value", value ], [])

let float_property = fun name value ->
  property name (string_of_float value)

class canvas_text = fun ?(config=[]) canvas_group x y ->
  let group = GnoCanvas.group ~x ~y canvas_group in
  let text = GnoCanvas.text group in
  object (self)
    val mutable format = get_prop "format" config "%.2f"
    val mutable size = float_of_string (get_prop "size" config "15.")
    val mutable color = get_prop "color" config "green"
 
    method tag = "Text"
    method item = (group :> movable_item)
    method config = fun () ->
      [ property "format" format;
	float_property "size" size;
	property "color" color ]
    method set_format = fun f -> format <- f 
    method set_size = fun f -> size <- f 
    method set_color = fun f -> color <- f 
    method update = fun value ->
      let renderer = fun x -> sprintf (Obj.magic format) (float_of_string x) in
      text#set [`SIZE_POINTS size; `TEXT (renderer value); `FILL_COLOR color; `ANCHOR `NW]


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


(***************************Vertical Ruler ***********************************)
class canvas_ruler = fun ?(config=[]) canvas_group x y ->

  let h = float_of_string (get_prop "height" config "100.")
  and index_on_right = bool_of_string (get_prop "index_on_right" config "false")
  and scale = float_of_string (get_prop "scale" config "2.")
  and w = float_of_string (get_prop "width" config "32.")
  and step = int_of_string (get_prop "step" config "10") in
  let text_props=[`ANCHOR `CENTER; `FILL_COLOR "white"]
  and index_width = 10. in

  let root = GnoCanvas.group ~x ~y canvas_group in
  let r = GnoCanvas.group root in
  
  let props = (text_props@[`ANCHOR `EAST]) in
  
  (* One step drawer *)
  let draw = fun i value ->
    let i = i * step in
    let y = -. scale *. (float i -. value) in
    if y >= -. h && y <= h then begin
      let text = Printf.sprintf "%d" i in
      ignore (GnoCanvas.text ~text ~props ~y ~x:(w*.0.75) r);
      ignore(GnoCanvas.line ~points:[|w*.0.8;y;w-.1.;y|] ~fill_color:"white" r)
    end;
    let y = y -. float step /. 2. *. scale in
    if y >= -. h && y <= h then
      ignore(GnoCanvas.line ~points:[|w*.0.8;y;w-.1.;y|] ~fill_color:"white" r)
  in
  
  let drawer = fun value ->
    (* Remove previous items *)
    List.iter (fun i -> i#destroy ()) r#get_items;
    let v = truncate value / step in
    let k = truncate (h /. scale) / step in
    for i = Pervasives.max 0 (v - k) to (v + k) do
      draw i value
    done in
  
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
    method config = fun () -> config (* Not editable *)
  end

(****************************************************************************)
class canvas_button = fun ?(config=[]) canvas_group x y ->
  let icon = get_prop "icon" config "icon_file" in
  let pixbuf = GdkPixbuf.from_file (Env.gcs_icons_path // icon) in
  let group = GnoCanvas.group ~x ~y canvas_group in
  let _item = GnoCanvas.pixbuf ~pixbuf group in
  object
    method tag = "Button"
    method item = (group :> movable_item)
    method edit = fun (pack:GObj.widget -> unit) -> ()
    method update = fun (value:string) -> ()
    method config = fun () ->
      [ property "icon" icon]
  end


(****************************************************************************)
class widget_renderer = fun (tag:string) widget ?(config=[]) canvas_group x y ->
  let group = GnoCanvas.group ~x ~y canvas_group in
  let item = GnoCanvas.widget ~width:50. ~height:50. ~widget group in
  object
    method tag = tag
    method edit = fun (_:GObj.widget->unit) -> () 
    method item = (item :> movable_item)
    method update = fun (_:string) -> ()
    method config = fun () -> (config : Xml.xml list)
  end


let renderers =
  [ (new canvas_text :> ?config:Xml.xml list -> #GnoCanvas.group -> float -> float -> renderer);
    (new canvas_ruler :> ?config:Xml.xml list -> #GnoCanvas.group -> float -> float -> renderer) ] 

let lazy_tagged_renderers = lazy
    (let x = 0. and y = 0.
    and group = (GnoCanvas.canvas ())#root in
    List.map
      (fun constructor ->
	let o = constructor ?config:None group x y in
	(o#tag, constructor))
      renderers)
    

class canvas_item = fun canvas_renderer ->
  let canvas_renderer = (canvas_renderer :> renderer) in
  object (self)
    val mutable motion = false
    val mutable renderer = canvas_renderer
    val mutable x_press = 0.
    val mutable y_press = 0.
    val mutable deleted = false
	
    method renderer = renderer

    method xy = 
      let (x0, y0) = renderer#item#i2w 0. 0. in
      renderer#item#parent#w2i x0 y0

    method deleted = deleted

    method update = fun value ->
      (renderer#update:string->unit) value

    method event = fun (ev : GnoCanvas.item_event) ->
      let item = (renderer#item :> movable_item) in
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

      let tagged_renderers = Lazy.force lazy_tagged_renderers in
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
      ignore (dialog#button_ok#connect#clicked (fun () -> dialog#papget_editor#destroy ()))

    val mutable connection =
      canvas_renderer#item#connect#event (fun _ -> false)
    method connect = fun () ->
      let item = (renderer#item :> movable_item) in
      connection <- item#connect#event self#event

    initializer
      self#connect ()
  end

class canvas_display_item = fun msg_obj field_name canvas_renderer ->
  object
    inherit field msg_obj field_name as super
    inherit canvas_item canvas_renderer as item

    method update_field = fun value ->
      if not deleted then begin
	super#update_field value;
	item#update value
      end

    method config = fun () ->
      let props = renderer#config () in
      let field = sprintf "%s:%s" msg_obj#msg_name field_name in
      let prop = property "field" field in
      let (x, y) = item#xy in
      let attrs =
	[ "type", "message_field";
	  "display", String.lowercase item#renderer#tag;
	  "x", sprintf "%.0f" x; "y", sprintf "%.0f" y ] in
      Xml.Element ("papget", attrs, prop::props)
  end


(****************************************************************************)
class canvas_setting_item = fun variable canvas_renderer ->
  object
    inherit canvas_item canvas_renderer as item

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


class canvas_goto_block_item = fun properties callback canvas_renderer ->
  object
    inherit canvas_clickable_item "goto_block" properties callback canvas_renderer as item
  end

class canvas_variable_setting_item = fun properties callback canvas_renderer ->
  object
    inherit canvas_clickable_item "variable_setting" properties callback canvas_renderer
  end
