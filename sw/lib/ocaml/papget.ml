open Printf

class message = fun ?(class_name="telemetry") msg_name ->
  object
    val mutable callbacks = []
    method connect = fun f cb -> callbacks <- (f, cb) :: callbacks
    initializer
      let module P = Pprz.Messages (struct let name = class_name end) in
      ignore (P.message_bind msg_name 
	(fun _sender values -> 
	  List.iter 
	    (fun (field_name, cb) -> 
	      let field = Pprz.string_assoc field_name values in
	      cb field) 
	    callbacks))
  end

class field = fun msg_obj field_name ->
  object (self)
    val mutable last_val = ""
    method store = fun value -> last_val <- value
    initializer
      msg_obj#connect field_name self#store
  end

class canvas_text = fun ?(text="") canvas_group x y msg_obj field_name ->
  let item = GnoCanvas.text ~x ~y ~text canvas_group in
  object
    inherit field msg_obj field_name as super
    val mutable renderer = fun x -> x
    method set_renderer = fun f -> renderer <- f 
    method store = fun value ->
      super#store value;
      item#set [`SIZE_POINTS 25.; `TEXT (renderer value); `FILL_COLOR "green"; `ANCHOR `NW]
  end
