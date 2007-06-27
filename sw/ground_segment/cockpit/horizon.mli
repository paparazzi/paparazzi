class pfd : ?visible:(GBin.frame -> bool) -> GBin.frame ->
  object
    method set_speed : float -> unit
    method set_alt : float -> unit
    method set_climb : float -> unit
    method set_attitude : float -> float -> unit
  end

