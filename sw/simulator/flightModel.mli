type meter = float
type meter_s = float
type radian = float
type radian_s = float
type state

val init : radian -> state

val get_xyz : state -> meter * meter * meter
val get_time : state -> float
val get_phi : state -> radian

val set_air_speed : state -> meter_s -> unit

module Make :
  functor (A : Data.MISSION) ->
    sig
      val do_servos : state -> Stdlib.us array -> unit
      val nb_servos : int
      val nominal_airspeed : float (* m/s *)
      val state_update : state -> float * float -> unit
    end
