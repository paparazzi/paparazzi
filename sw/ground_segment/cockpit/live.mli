val message_request : string -> string -> Pprz.values -> (string -> Pprz.values -> unit) -> unit

val aircrafts_msg : MapCanvas.widget -> GPack.notebook -> Pprz.values -> unit
val safe_bind : string -> (string -> Pprz.values -> unit) -> unit
val one_new_ac : MapCanvas.widget -> GPack.notebook -> string -> unit
val listen_flight_params :
  < center : MapCanvas.LL.geographic -> unit; .. > -> bool -> unit
val listen_wind_msg : unit -> unit
val listen_fbw_msg : unit -> unit
val listen_engine_status_msg : unit -> unit
val listen_if_calib_msg : unit -> unit
val listen_waypoint_moved : unit -> unit
val listen_infrared : unit -> unit
val listen_svsinfo : unit -> unit
val listen_alert : < add : string -> unit; .. > -> unit
val listen_telemetry_status : unit -> unit
