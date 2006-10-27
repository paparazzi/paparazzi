type t =
    < add_widget : GObj.widget -> unit;
      connect_shift_alt : (float -> unit) -> unit;
	set_agl : ?color:string -> float -> unit;
	set_bat : ?color:string -> float -> unit;
	  set_color : string -> string -> unit;
	    set_label : string -> string -> unit >


val scrolled : GBin.scrolled_window

val add :
  Pprz.values ->
  string ->
  (unit -> 'a) -> (unit -> unit) -> (unit -> unit) ->
    t



