(** Options for HITL and SITL simulators *)
val common_options : (string * Arg.spec * string) list

val ac_name  : string ref

(** A complete aircraft with it mission *)
module type AIRCRAFT =
  sig
    val init : int -> GPack.box -> unit
    val boot : Stdlib.value -> unit
    val servos : Stdlib.us array -> unit
    val infrared : float -> float -> unit
    val gps : Gps.state -> unit
  end

(** A simulated aircraft, without its conf *)
module type AIRCRAFT_ITL = functor (A : Data.MISSION) -> AIRCRAFT

(** Functor to build the simulator *)
module Make :
  functor (AircraftItl : AIRCRAFT_ITL) ->
    sig
      val main : unit -> unit
    end
