open Printf

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

class canvas_item = fun canvas_group x1 y1 ->
  let x2 = x1 +. 100. and y2 = y1 +. 25. in
  let box = GnoCanvas.rect ~props:[`X1 x1; `X2 x2; `Y1 y1; `Y2 y2; (* `NO_OUTLINE_COLOR*) `OUTLINE_COLOR "green"] canvas_group in
  object (self)
    val box = box
    method set_size = fun width height ->
      box#set [`X2 (x1+.width); `Y2 (y1+.height)]
  end

class canvas_text = fun ?(text="") canvas_group x y msg_obj field_name ->
  let text = GnoCanvas.text ~x ~y ~text canvas_group in
  object (self)
    inherit field msg_obj field_name as super
    val mutable format = "%.2f"
    val mutable motion = false
    val mutable size = 15.
    initializer
      ignore (text#connect#event self#event)
    method event = fun (ev : GnoCanvas.item_event) ->
      match ev with
	(*** `ENTER_NOTIFY _ -> prerr_endline "enter"; box#set [`OUTLINE_COLOR "green"]; true
      | `LEAVE_NOTIFY _ -> prerr_endline "leave"; box#set [`NO_OUTLINE_COLOR]; true
***)
      | `BUTTON_PRESS ev ->
	  begin
	    match GdkEvent.Button.button ev with
	    | 1 ->
		motion <- false;
		let curs = Gdk.Cursor.create `FLEUR in
		text#grab [`POINTER_MOTION; `BUTTON_RELEASE] curs 
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
	    let (xw, yw) = canvas_group#w2i x y in
	    text#set [`X xw; `Y yw]
	  end;
	  true
      | `BUTTON_RELEASE ev ->
	  if GdkEvent.Button.button ev = 1 then begin
	    text#ungrab (GdkEvent.Button.time ev);
	    if not motion then begin
	      let file = Env.paparazzi_src // "sw" // "lib" // "ocaml" // "widgets.glade" in
	      let dialog = new Gtk_papget_text_editor.papget_text_editor ~file () in
	      (* Initialize the entries *)
	      dialog#entry_format#set_text format;
	      dialog#spinbutton_size#set_value size;

	      (* Connect the entries *)
	      let callback = fun () ->
		format <- dialog#entry_format#text in
	      ignore (dialog#entry_format#connect#activate ~callback);
	      let callback = fun () ->
		size <- dialog#spinbutton_size#value in
	      ignore (dialog#spinbutton_size#connect#value_changed ~callback);

	      (* Connect the buttons *)
	      ignore (dialog#button_ok#connect#clicked (fun () -> dialog#papget_text_editor#destroy ()))
	    end;
	    motion <- false
	  end;
	  true
      | _ -> false
 
    method set_format = fun f -> format <- f 
    method update = fun value ->
      super#update value;
      let renderer = fun x -> sprintf (Obj.magic format) (float_of_string x) in
      text#set [`SIZE_POINTS size; `TEXT (renderer value); `FILL_COLOR "green"; `ANCHOR `NW]
  end
