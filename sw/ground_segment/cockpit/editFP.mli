val path_button :
  MapCanvas.widget -> MapCanvas.LL.fmeter * MapCanvas.LL.fmeter -> unit
val path_notify : MapCanvas.widget -> float * float -> bool
val path_close : unit -> unit
val path_change_radius : [> `DOWN | `UP ] -> unit
val create_wp : MapCanvas.LL.geographic -> MapWaypoints.waypoint
val calibrate_map : MapCanvas.widget -> Gtk.accel_group -> unit -> unit
val new_fp : MapCanvas.widget -> Gtk.accel_group -> unit -> unit
val load_fp : MapCanvas.widget -> Gtk.accel_group -> unit -> unit
val save_fp : unit -> unit
val close_fp : unit -> unit
