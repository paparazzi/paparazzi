open Printf

let update_delay = 1. (* Min time in second before two updates *)
let led_delay = 500 (* Time in milliseconds while the green led is displayed *)


let space = Str.regexp "[ \t]+"

let (//) = Filename.concat

let xml_file = Env.paparazzi_src // "conf" // "messages.xml"

(* let port = ref 2010
   let domain = ref "127.255.255.255" *)

let green = "#00e000"
let red = "#ff0000"
let black = "#000000"
let yellow = "#ffff00"

let led2 color1 color2 = [|
"16 16 5 1";
"       c None";
".      c black";
"X      c white";
"o c "^color1;
"O c "^color2;
"     ......     ";
"   ........XX   ";
"  ...ooooooXXX  ";
" ..ooooooooooXX ";
" ..oooXXXooooXX ";
"..oooXXXooooooXX";
"..ooXXooooooooXX";
"..ooXXooooooooXX";
"..ooXoooooooooXX";
"..ooooooooooooXX";
"..ooooooooooooXX";
" ..ooooooooooXX ";
" ..ooooooooooXX ";
"  ...ooooooXXX  ";
"   ..XXXXXXXX   ";
"     XXXXXX     "|]

let led color = led2 color "None"

let format = fun field ->
  try
    match Xml.attrib field "type", Xml.attrib field "format" with
      "float", f -> fun x -> Printf.sprintf (Obj.magic f) (float_of_string x)
    | _ -> fun x -> x
  with _ -> fun x -> x



open GMain
let _ =
  let bus = ref "127.255.255.255:2010" in
  let classes = ref ["telemetry_ap";"ground"] in
  Arg.parse
    [ "-b", Arg.String (fun x -> bus := x), "Bus\tDefault is 127.255.255.25:2010";
      "-c",  Arg.String (fun x -> classes := x :: !classes), "class name"]
    (fun x -> prerr_endline ("WARNING: don't do anything with "^x))
    "Usage: ";

  let xml = Xml.parse_file xml_file in

  let window = GWindow.window ~title:"Paparazzi messages" () in
  let quit = fun () -> GMain.Main.quit (); exit 0 in
  ignore (window#connect#destroy ~callback:quit);

  let notebook = GPack.notebook ~packing:window#add ~tab_pos:`LEFT () in

  let pm = fun color ->
    GDraw.pixmap_from_xpm_d ~data:(led color) ~window:window () in
  let black_led = pm black
  and green_led = pm green
  and yellow_led = pm yellow
  and red_led = pm red in

  let xml_classes =
    List.filter (fun x -> List.mem (Xml.attrib x "name") !classes) (Xml.children xml) in

  let messages = List.flatten (List.map Xml.children xml_classes) in

  let pages = 
    List.map
      (fun m ->
	let id = (Xml.attrib m "name") in
	let h = GPack.hbox () in
	let v = GPack.vbox ~width:200 () in
	let l = GMisc.label ~text:id ~packing:h#add () in
	let led = GMisc.pixmap black_led ~packing:h#pack () in
	let time = GMisc.label ~text:"___" ~packing:h#pack () in
	notebook#append_page ~tab_label:h#coerce v#coerce;
	let fields = 
	  List.map
	    (fun f ->
	      let h = GPack.hbox ~packing:v#pack () in
	      let unit = try "("^Xml.attrib f "unit"^")" with _ -> "" in
	      let name = Printf.sprintf "%s %s %s: " (Xml.attrib f "type") (Xml.attrib f "name") unit in
	      let _ = GMisc.label ~text:name ~packing:h#pack () in
	      let l = GMisc.label ~text:"XXXX" ~packing:h#pack () in
	      fun x -> let fx = format f x in if l#label <> fx then l#set_text fx
	    )
	    (Xml.children m) in
	let n = List.length fields in
	let last_update = ref (Unix.gettimeofday ()) in
	let time_since_last = ref 0 in
	ignore (GMain.Timeout.add 1000 (fun () -> incr time_since_last; time#set_text (sprintf "%2d" !time_since_last); true));
	let display = fun line ->
	  time_since_last := 0;
	  let t = Unix.gettimeofday () in
	  if t > !last_update +. update_delay then begin
	    last_update := t;
	    let args = Str.split space line in
	    try
	      List.iter2 (fun f x -> f x) fields args;
	      
	      led#set_pixmap green_led;
 	      ignore (GMain.Timeout.add led_delay (fun () -> led#set_pixmap yellow_led; false))
	    with
	      Invalid_argument "List.iter2" ->
		led#set_pixmap red_led;
		  Printf.fprintf stderr "%s: expected %d, got %d (%s)\n" id n (List.length args) line; flush stderr
	  end
	in
	let regexp = Printf.sprintf "[\\.0-9]+ %s (.*)" id in
	ignore (Ivy.bind (fun _ args -> display args.(0)) regexp);
	(id, (led, fields))
      )
      messages in

  window#show ();

  Ivy.init "Paparazzi messages" "READY" (fun _ _ -> ());
  Ivy.start !bus;

  GMain.Main.main () 
