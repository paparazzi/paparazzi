let (//) = Filename.concat

(* let pprz_conf_path = Env.paparazzi_src // "conf" *)
let user_conf_path = Env.paparazzi_home // "conf"

let conf_xml = Xml.parse_file (user_conf_path // "conf.xml")
let ground = ExtXml.child conf_xml "ground"

let messages_ap =
(*  let xml = Xml.parse_file (pprz_conf_path // "messages.xml") in *)
  let xml = Xml.parse_file (user_conf_path // "messages.xml") in
  try
    ExtXml.child xml ~select:(fun x -> Xml.attrib x "name" = "telemetry_ap") "class"
  with
    Not_found -> failwith "'telemetry_ap' class missing in messages.xml"

(* let ubx_xml = Xml.parse_file (pprz_conf_path // "ubx.xml") *)
let ubx_xml = Xml.parse_file (user_conf_path // "ubx.xml")

type aircraft = {
    name : string;
    id : int;
    airframe : Xml.xml; 
    flight_plan : Xml.xml;
    radio: Xml.xml
  }


let aircraft = fun name ->
  let aircraft_xml, id =
    let rec loop i = function
	[] -> failwith ("Aicraft not found : "^name)
      | x::_ when Xml.tag x = "aircraft" && Xml.attrib x "name" = name ->
	  (x, i)
      | x::xs -> loop (i+1) xs in
    loop 0 (Xml.children conf_xml) in
  
  let airframe_file = user_conf_path // ExtXml.attrib aircraft_xml "airframe" in

  { id = id; name = name;
    airframe = Xml.parse_file airframe_file;
    flight_plan = Xml.parse_file (user_conf_path // ExtXml.attrib aircraft_xml "flight_plan");
    radio = Xml.parse_file (user_conf_path // ExtXml.attrib aircraft_xml "radio")
  }

module type MISSION = sig val ac : aircraft end
