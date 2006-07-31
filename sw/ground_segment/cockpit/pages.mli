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
class gps : ?visible:(GBin.frame -> bool) -> GBin.frame ->
  object
    method svsinfo : string -> string -> int -> unit
  end

class pfd : ?visible:(GBin.frame -> bool) -> GBin.frame ->
  object
    method set_speed : float -> unit
    method set_alt : float -> unit
    method set_climb : float -> unit
    method set_attitude : float -> float -> unit
  end

class settings : ?visible:(GObj.widget -> bool) -> Xml.xml list -> (int -> float -> unit) ->
  object
    method length : int
    method set : int -> float -> unit
    method widget : GObj.widget
  end

class misc :
  packing:(GObj.widget -> unit) ->
  GBin.frame ->
  object
    method set_wind_dir : string -> unit
    method set_wind_speed : string -> unit
  end

type rc_mode = string
type rc_setting_mode = string
class rc_settings :
  ?visible:(GObj.widget -> bool) ->
  Xml.xml list ->
  object
    method set : float -> float -> unit
    method set_rc_mode : rc_mode -> unit
    method set_rc_setting_mode : rc_setting_mode -> unit
    method widget : GObj.widget
  end

