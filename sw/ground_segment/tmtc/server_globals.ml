exception Telemetry_error of string * string


(* options for serving xml config files and kml via http *)
let hostname = ref "localhost"
let port = ref 8889

(** FIXME: Should be read from messages.xml *)
let fixedwing_ap_modes = [|"MANUAL";"AUTO1";"AUTO2";"HOME";"NOGPS"|]
let rotorcraft_ap_modes = [|"KILL";"SAFE";"HOME";"RATE";"ATT";"R_RCC";"A_RCC";"ATT_C";"R_ZH";"A_ZH";"HOVER";"HOV_C";"H_ZH";"NAV";"RC_D";"CF";"FWD";"MODULE";"FLIP";"GUIDED"|]
let _AUTO2 = 2
let gaz_modes = [|"MANUAL";"THROTTLE";"CLIMB";"ALT"|]
let lat_modes = [|"MANUAL";"ROLL_RATE";"ROLL";"COURSE"|]
let gps_modes = [|"NOFIX";"NA";"2D";"3D";"DGPS";"RTK"|]
let state_filter_modes = [|"UNKNOWN";"INIT";"ALIGN";"OK";"GPS_LOST";"IMU_LOST";"COV_ERR";"IR_CONTRAST";"ERROR"|]
let _3D = 3
let gps_hybrid_modes = [|"OFF";"ON"|]
let horiz_modes = [|"WAYPOINT";"ROUTE";"CIRCLE";"ATTITUDE";"MANUAL"|]
let if_modes = [|"OFF";"DOWN";"UP"|]

let string_of_values = fun values ->
  Compat.bytes_concat " " (List.map (fun (_, v) -> PprzLink.string_of_value v) values)

(** get modes from autopilot xml file *)
let modes_from_autopilot = fun ap_xml ->
  let ap = ExtXml.child ap_xml
    ~select:(fun x -> Compat.uppercase_ascii (ExtXml.attrib_or_default x "gcs_mode" "") = "TRUE")
    "state_machine"
  in
  let modes = List.filter (fun x -> Xml.tag x = "mode") (Xml.children ap) in
  let l = List.map (fun x -> try Xml.attrib x "shortname" with _ -> ExtXml.attrib x "name") modes in
  Array.of_list l

