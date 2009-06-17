val create_wp : MapCanvas.widget -> Latlong.geographic -> MapWaypoints.waypoint
val calibrate_map : MapCanvas.widget -> GBin.frame -> Gtk.accel_group -> unit -> unit
val new_fp : MapCanvas.widget -> GBin.frame -> Gtk.accel_group -> unit -> unit
val load_fp : MapCanvas.widget -> GBin.frame -> Gtk.accel_group -> unit -> unit
val load_xml_file : MapCanvas.widget -> GBin.frame -> Gtk.accel_group -> string -> unit
val save_fp : MapCanvas.widget -> unit
val close_fp : MapCanvas.widget -> unit
val set_window_title : MapCanvas.widget -> unit
