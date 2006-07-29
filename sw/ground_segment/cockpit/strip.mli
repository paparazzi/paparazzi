type t
val add :
  Pprz.values ->
  string ->
  (unit -> 'a) -> (unit -> unit) -> (unit -> unit) -> (unit -> unit) -> t
val set_label : t -> string -> string -> unit
val set_color : t -> string -> string -> unit
val set_bat : t -> float -> unit
val scrolled : GBin.scrolled_window
