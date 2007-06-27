val path_button :
  MapCanvas.widget -> MapCanvas.world -> unit
val path_notify : MapCanvas.widget -> float * float -> bool
val path_close : unit -> unit
val path_change_radius : [> `DOWN | `UP ] -> unit
val create_wp : MapCanvas.widget -> Latlong.geographic -> MapWaypoints.waypoint
val calibrate_map : MapCanvas.widget -> GBin.frame -> Gtk.accel_group -> unit -> unit
val new_fp : MapCanvas.widget -> GBin.frame -> Gtk.accel_group -> unit -> unit
val load_fp : MapCanvas.widget -> GBin.frame -> Gtk.accel_group -> unit -> unit
val load_xml_file : MapCanvas.widget -> GBin.frame -> Gtk.accel_group -> string -> unit
val save_fp : unit -> unit
val close_fp : unit -> unit
