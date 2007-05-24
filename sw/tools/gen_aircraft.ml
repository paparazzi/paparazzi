open Printf

let (//) = Filename.concat

let paparazzi_home = Env.paparazzi_home
let conf_xml = paparazzi_home // "conf" // "conf.xml"

let mkdir = fun d ->
  if not (Sys.file_exists d) then
    Unix.mkdir d 0o755

let check_unique_id = fun conf ->
  let ids = Hashtbl.create 5 in
  List.iter
    (fun x -> 
      if String.lowercase (Xml.tag x) = "aircraft" then 
	let id = ExtXml.attrib x "ac_id" in
	if Hashtbl.mem ids id then
	  failwith (sprintf "Error: A/C Id '%s' duplicated in %s" id conf_xml);
	Hashtbl.add ids id ())
    (Xml.children conf)

let _ =
  if Array.length Sys.argv <> 2 then
    failwith (sprintf "Usage: %s <A/C ident (conf.xml)>" Sys.argv.(0));
  let aircraft = Sys.argv.(1) in
  let conf = Xml.parse_file conf_xml in
  check_unique_id conf;
  let aircraft_xml =
    try
      ExtXml.child conf ~select:(fun x -> Xml.attrib x "name" = aircraft) "aircraft"
	with
      Not_found -> failwith (sprintf "Aircraft '%s' not found in '%s'" aircraft conf_xml)

 in
  let value = ExtXml.attrib aircraft_xml in

  let aircraft_dir = paparazzi_home // "var" // aircraft in

  mkdir (paparazzi_home // "var");
  mkdir aircraft_dir;
  mkdir (aircraft_dir // "fbw");
  mkdir (aircraft_dir // "autopilot");
  mkdir (aircraft_dir // "sim");

  let settings = 
    try value "settings" with 
      _ -> 
	fprintf stderr "\nWARNING: No 'settings' attribute specified for A/C '%s', using 'settings/basic.xml'\n\n%!" aircraft;
	"settings/basic.xml" in
  
  let c = sprintf "make -f Makefile.ac AIRCRAFT=%s AC_ID=%s AIRFRAME_XML=%s RADIO=%s FLIGHT_PLAN=%s TELEMETRY=%s SETTINGS=\"%s\" all_ac_h" aircraft (value "ac_id") (value "airframe") (value "radio") (value "flight_plan") (value "telemetry") settings in
  begin (** Quiet is speficied in the Makefile *)
    try if Sys.getenv "Q" <> "@" then raise Not_found with
      Not_found -> prerr_endline c
  end;
  exit (Sys.command c)
