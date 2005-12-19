open Printf

let h_name = "MESSAGES_H"

let id_of = fun xml -> ExtXml.attrib xml "name"

(** No dereferencement for arrays *)
let deref = fun xml -> try let _ = Xml.attrib xml "len" in "" with _ -> "*"

let print_params = function
    [] -> ()
  | f::fields ->
      printf "%s" (id_of f);
      List.iter (fun f -> printf ", %s" (id_of f)) fields

let types = [
  "uint8", "%hhu";
  "uint16", "%hu";
  "uint32", "%u" ;
  "int8",   "%hhd";
  "int16",  "%hd";
  "int32",  "%d" ;
  "float",  "%f"
] 

let sprint_format = fun f ->
  try
    Xml.attrib f "format"
  with _ ->
    List.assoc (Xml.attrib f "type") types


let freq = 10
let buffer_length = 5
let step = 1. /. float freq
let nb_steps = (256 / freq) * freq
    
let is_periodic = fun m -> try let _ = Xml.attrib m "period" in true with _ -> false
let period_of = fun m -> float_of_string (Xml.attrib m "period")
    
let gen_periodic = fun avr_h messages ->
  let periodic_messages = List.filter is_periodic messages in
  
  let scheduled_messages = 
    List.map
      (fun m ->
	let p = period_of m in
	let period_steps = truncate (p /. step) in
	(period_steps, id_of m))
      periodic_messages in
    
    fprintf avr_h "#define PeriodicSend() {  /* %dHz */ \\\n" freq;
    fprintf avr_h "  static uint8_t i;\\\n";
    fprintf avr_h "  if (i == %d) i = 0;\\\n" nb_steps;
    List.iter
      (fun (p, id) ->
	fprintf avr_h "  if (i %% %d == 0) PERIODIC_SEND_%s();\\\n" p id)
      scheduled_messages;
    fprintf avr_h " i++;\\\n}\n"

  

let fprint_formats = fun c fields ->
  List.iter (fun f -> fprintf c " %s" (sprint_format f)) fields

let fprint_args = fun c fields ->
  List.iter (fun f -> fprintf c ", %s(%s)" (deref f) (id_of f)) fields

let one_message = fun m ->
  let id = id_of m
  and fields = Xml.children m in
  printf "#define DOWNLINK_SEND_%s(" id;
  print_params fields;
  printf "){ \\\n";
  printf "  IvySendMsg(\"%%d %s %a\",ac_id%a); \\\n" id fprint_formats fields fprint_args fields;
  printf "}\n\n"

let _ =
  if Array.length Sys.argv <> 2 then
    failwith (sprintf "Usage: %s <xml message file>" Sys.argv.(0));
  let xml = Xml2h.start_and_begin Sys.argv.(1) h_name in
  let xml = ExtXml.child xml ~select:(fun x -> Xml.attrib x "name"="telemetry_ap") "class" in
  let messages = (Xml.children xml) in

  printf "#include <ivy.h>\n";
  printf "extern uint8_t ac_id;\n";
  printf "extern uint8_t modem_nb_ovrn;\n";

  List.iter one_message messages;

  gen_periodic stdout messages;

  Xml2h.finish h_name
  
