class alert : GBin.frame ->
  object
    method add : string -> unit
  end
class infrared : GBin.frame ->
  object
    method set_contrast_status : string -> unit
    method set_contrast_value : int -> unit
    method set_gps_hybrid_factor : float -> unit
    method set_gps_hybrid_mode : string -> unit
  end
class gps : GBin.frame ->
  object
    method svsinfo : string -> string -> int -> unit
  end
class pfd : GBin.frame ->
  object
    method set_alt : float -> unit
    method set_climb : float -> unit
    method set_pitch : float -> unit
    method set_roll : float -> unit
  end
class settings : Xml.xml list -> (int -> float -> unit) ->
  object
    method length : int
    method set : int -> float -> unit
    method widget : GObj.widget
  end
