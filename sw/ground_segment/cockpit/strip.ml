(************ Strip handling ***********************************************)

let bat_max = 12.5
let bat_min = 9.

(** window for the strip panel *)
let scrolled = GBin.scrolled_window ~width:300 ~hpolicy: `AUTOMATIC ~vpolicy: `AUTOMATIC ()
let table = GPack.table ~rows: 1 ~columns: 1 ~row_spacings: 5 ~packing: (scrolled#add_with_viewport) ()


type t = {
    gauge: GMisc.drawing_area * (GDraw.pixmap option ref);
    labels: (string * (GBin.event_box * GMisc.label)) list;
    buttons_box : GPack.box
  }

let labels_name =  [| 
  [| "AP" ; "alt" ; "->" |]; [| "RC"; "climb"; "/" |]; [| "GPS"; "speed"; "throttle" |]
|]

let labels_print = [| 
  [| "AP" ; "alt" ; "->" |]; [| "RC"; "climb"; "->" |]; [| "GPS"; "speed"; "throttle" |]
|]
let gen_int = let i = ref (-1) in fun () -> incr i; !i

let rows = 1 + Array.length labels_name + 1
let columns = 1 + 2 * Array.length labels_name.(0)


    (** add a strip to the panel *)
let add config color select center_ac commit_moves mark =
  let widget = table in
  (* number of the strip *)
  let strip_number = gen_int () in
  if strip_number > 1 then widget#set_rows strip_number;

  let strip_labels = ref  [] in
  let add_label = fun name value -> 
    strip_labels := (name, value) :: !strip_labels in

  let ac_name = Pprz.string_assoc "ac_name" config in

  let tooltips = GData.tooltips () in

  (* frame of the strip *)
  let frame = GBin.frame ~shadow_type: `IN ~packing: (widget#attach ~top: (strip_number) ~left: 0) () in
  let framevb = GPack.vbox ~packing:frame#add () in

  (** Table (everything except the user buttons) *)
  let strip = GPack.table ~rows ~columns ~col_spacings:5 ~row_spacings:3 ~packing:framevb#add () in

  (* Name in top left *)
  let name = (GMisc.label ~text: (ac_name) ~packing: (strip#attach ~top: 0 ~left: 0) ()) in
  name#set_width_chars 5;
  
  
  let plane_color = GBin.event_box ~packing:(strip#attach ~top:0 ~left:1 ~right:columns) () in
  plane_color#coerce#misc#modify_bg [`NORMAL, `NAME color];
  ignore (plane_color#event#connect#button_press ~callback:(fun _ -> select (); true));
  let h = GPack.hbox ~packing:plane_color#add () in
  let ft = GMisc.label ~text: "00:00:00" ~packing:h#add () in
  ft#set_width_chars 8;
  add_label ("flight_time_value") (plane_color, ft);
  let block_name = GMisc.label ~text: "______" ~packing:h#add () in
  add_label ("block_name_value") (plane_color, block_name);

  (* battery gauge *)
  let gauge = GMisc.drawing_area ~height:60 ~show:true ~packing:(strip#attach ~top:1 ~bottom:(rows-1) ~left:0) () in
  gauge#misc#realize ();
  let {Gtk.x=x0; y=y0; width=width; height=height} = gauge#misc#allocation in

  (* Telemetry *)
  let eb = GBin.event_box ~packing:(strip#attach ~top:(rows-1) ~left:0) () in
  let ts = GMisc.label ~text:"N/A" ~packing:eb#add () in
  add_label "telemetry_status_value" (eb, ts);
  tooltips#set_tip eb#coerce ~text:"Telemetry status\nGreen if time since last bat message < 5s";

  (* Labels *)
  Array.iteri 
    (fun i a ->
      Array.iteri
	(fun j s ->
	  ignore (GMisc.label ~text: labels_print.(i).(j) ~justify:`RIGHT ~packing: (strip#attach ~top:(1+i) ~left: (1+2*j)) ());
	  let eb = GBin.event_box  ~packing: (strip#attach ~top:(i+1)  ~left: (1+2*j+1)) () in
	  let lvalue = (GMisc.label ~text: "" ~justify: `RIGHT ~packing:eb#add ()) in
	  lvalue#set_width_chars 6;
	  add_label (s^"_value") (eb, lvalue);
	) a
    ) labels_name;

  (* Buttons *)
  let top = rows - 1 in
  let b = GButton.button ~label:"Center A/C" ~packing:(strip#attach ~top ~left:1 ~right:3) () in
  ignore(b#connect#clicked ~callback:center_ac);
  let b = GButton.button ~label:"Send WPs" ~packing:(strip#attach ~top ~left:3 ~right:5) () in
  ignore (b#connect#clicked  ~callback:commit_moves);
  let b = GButton.button ~label:"Mark" ~packing:(strip#attach ~top ~left:5 ~right:7) () in
  ignore (b#connect#clicked  ~callback:mark);

  (* User buttons *)
  let hbox = GPack.hbox ~packing:framevb#add () in

  {gauge=(gauge, ref None) ; labels= !strip_labels; buttons_box = hbox}


    (** set a label *)
let set_label strip name value = 
  let _eb, l = List.assoc (name^"_value") strip.labels in
  if l#text <> value then
    l#set_label value

    (** set a label *)
let set_color strip name color = 
  let eb, _l = List.assoc (name^"_value") strip.labels in
  eb#coerce#misc#modify_bg [`NORMAL, `NAME color]

    (** set the battery *)
let set_bat ?(color="green") strip value =
  let gauge, drref = strip.gauge in
  let {Gtk.x=x0; y=y0; width=width; height=height} = gauge#misc#allocation in
  let dr = GDraw.pixmap ~width ~height ~window:gauge () in
  dr#set_foreground (`NAME "orange");
  dr#rectangle ~x:0 ~y:0 ~width ~height ~filled:true ();
  
  let f = (value -. bat_min) /. (bat_max -. bat_min) in
  let f = max 0. (min 1. f) in
  let h = truncate (float height *. f) in
  dr#set_foreground (`NAME color);
  dr#rectangle ~x:0 ~y:(height-h) ~width ~height:h ~filled:true ();

  let context = gauge#misc#create_pango_context in
  let layout = context#create_layout in
  Pango.Layout.set_text layout (string_of_float value);
  let (w,h) = Pango.Layout.get_pixel_size layout in
  dr#put_layout ~x:((width-w)/2) ~y:((height-h)/2) ~fore:`BLACK layout;
  
  (new GDraw.drawable gauge#misc#window)#put_pixmap ~x:0 ~y:0 dr#pixmap


let add_widget = fun strip widget ->
  strip.buttons_box#add widget
