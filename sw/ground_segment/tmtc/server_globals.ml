exception Telemetry_error of string * string

let hostname = ref "localhost"

(** FIXME: Should be read from messages.xml *)
let ap_modes = [|"MANUAL";"AUTO1";"AUTO2";"HOME";"NOGPS"|]
let _AUTO2 = 2
let gaz_modes = [|"MANUAL";"GAZ";"CLIMB";"ALT"|]
let lat_modes = [|"MANUAL";"ROLL_RATE";"ROLL";"COURSE"|]
let gps_modes = [|"NOFIX";"DRO";"2D";"3D";"GPSDRO"|]
let gps_hybrid_modes = [|"OFF";"ON"|]
let horiz_modes = [|"WAYPOINT";"ROUTE";"CIRCLE"|]

let string_of_values = fun values ->
  String.concat " " (List.map (fun (_, v) -> Pprz.string_of_value v) values)
