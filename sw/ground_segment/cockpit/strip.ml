(************ Strip handling ***********************************************)

let bat_max = 13.
let bat_min = 8.

(** window for the strip panel *)
let scrolled = GBin.scrolled_window ~width:300 ~hpolicy: `AUTOMATIC ~vpolicy: `AUTOMATIC ()
let table = GPack.table ~rows: 1 ~columns: 1 ~row_spacings: 5 ~packing: (scrolled#add_with_viewport) ()


type t = {
    gauge: GRange.progress_bar ;
    labels: (string * (GBin.event_box * GMisc.label)) list
  }

let labels_name =  [| 
  [| "AP" ; "alt" ; "->" |]; [| "RC"; "climb"; "/" |]; [| "GPS"; "speed"; "throttle" |]
|]

let labels_print = [| 
  [| "AP" ; "alt" ; "->" |]; [| "RC"; "climb"; "->" |]; [| "GPS"; "speed"; "throttle" |]
|]
let gen_int = let i = ref (-1) in fun () -> incr i; !i

let rows = Array.length labels_name + 1


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
  let strip = GPack.table ~rows:2 ~columns: 2 ~col_spacings: 10 ~packing: frame#add () in
  ignore (GMisc.label ~text: (ac_name) ~packing: (strip#attach ~top: 0 ~left: 0) ());
  
  let plane_color = GBin.event_box ~width:10 ~height:10 ~packing:(strip#attach ~top:0 ~left: 1) () in
  plane_color#coerce#misc#modify_bg [`NORMAL, `NAME color];
  ignore (plane_color#event#connect#button_press ~callback:(fun _ -> select (); true));
  let h = GPack.hbox ~packing:plane_color#add () in
  let ft = GMisc.label ~text: "00:00:00" ~packing:h#add () in
  ft#set_width_chars 8;
  add_label ("flight_time_value") (plane_color, ft);
  let block_name = GMisc.label ~text: "______" ~packing:h#add () in
  add_label ("block_name_value") (plane_color, block_name);

  (* battery and telemetry status *)
  let vb = GPack.vbox ~packing:(strip#attach ~top:1 ~left:0) () in
  let pb = GRange.progress_bar ~orientation: `BOTTOM_TO_TOP ~packing:vb#add () in
  pb#misc#set_size_request ~height:50 ();
  pb#coerce#misc#modify_fg [`PRELIGHT, `NAME "green"];
  pb#coerce#misc#modify_font_by_name "sans 12";

  let eb = GBin.event_box ~packing:vb#add () in
  let ts = GMisc.label ~text:"N/A" ~packing:eb#add () in
  add_label "telemetry_status_value" (eb, ts);
  tooltips#set_tip eb#coerce ~text:"Telemetry status\nGreen if time since last bat message < 5s";

  let left_box = GPack.table ~rows ~columns: 6 ~col_spacings: 5 
      ~packing: (strip#attach ~top: 1 ~left: 1) () in

  Array.iteri 
    (fun i a ->
      Array.iteri
	(fun j s ->
	  ignore (GMisc.label ~text: labels_print.(i).(j) ~justify:`RIGHT ~packing: (left_box#attach ~top: i ~left: (2*j)) ());
	  let eb = GBin.event_box  ~packing: (left_box#attach ~top: i ~left: (2*j+1)) () in
	  let lvalue = (GMisc.label ~text: "" ~justify: `RIGHT ~packing:eb#add ()) in
	  lvalue#set_width_chars 6;
	  add_label (s^"_value") (eb, lvalue);
	) a
    ) labels_name;
  let b = GButton.button ~label:"Center A/C" ~packing:(left_box#attach ~top:4 ~left:0 ~right:2) () in
  ignore(b#connect#clicked ~callback:center_ac);
  let b = GButton.button ~label:"Commit WPs" ~packing:(left_box#attach ~top:4 ~left:2 ~right:4) () in
  ignore (b#connect#clicked  ~callback:commit_moves);
  let b = GButton.button ~label:"Mark" ~packing:(left_box#attach ~top:4 ~left:4 ~right:6) () in
  ignore (b#connect#clicked  ~callback:mark);
  {gauge=pb ; labels= !strip_labels}


    (** set a label *)
let set_label strip name value = 
  let _eb, l = List.assoc (name^"_value") strip.labels in
  l#set_label value

    (** set a label *)
let set_color strip name color = 
  let eb, _l = List.assoc (name^"_value") strip.labels in
  eb#coerce#misc#modify_bg [`NORMAL, `NAME color]

    (** set the battery *)
let set_bat strip value =
  strip.gauge#set_text (string_of_float value);
  let f = (value -. bat_min) /. (bat_max -. bat_min) in
  let f = max 0. (min 1. f) in
  strip.gauge#set_fraction f

