open Printf

let (//) = Filename.concat

let paparazzi_home = Env.paparazzi_home
let conf_xml = paparazzi_home // "conf" // "conf.xml"

let mkdir = fun d ->
  if not (Sys.file_exists d) then
    Unix.mkdir d 0o755

let _ =
  if Array.length Sys.argv <> 2 then
    failwith (sprintf "Usage: %s <xml_airframe_file>" Sys.argv.(0));
  let aircraft = Sys.argv.(1) in
  let conf = Xml.parse_file conf_xml in
  let aircraft_xml =
    try
      ExtXml.child conf ~select:(fun x -> Xml.attrib x "name" = aircraft) "aircraft"
	with
      Not_found -> failwith (sprintf "Aircraft '%s' not found in '%s'" aircraft conf_xml)

 in
  let value = ExtXml.attrib aircraft_xml in

  let aircraft_dir = paparazzi_home // "var" // aircraft in

  mkdir aircraft_dir;
  mkdir (aircraft_dir // "fbw");
  mkdir (aircraft_dir // "autopilot");
  mkdir (aircraft_dir // "sim");
  
  let c = sprintf "make -f Makefile.ac AIRCRAFT=%s AIRFRAME=%s RADIO=%s FLIGHT_PLAN=%s" aircraft (value "airframe") (value "radio") (value "flight_plan") in
  prerr_endline c;
  exit (Sys.command c)
