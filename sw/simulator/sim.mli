(** Options for HITL and SITL simulators *)
val common_options : (string * Arg.spec * string) list

val ac_name  : string ref

(** A complete aircraft with it mission *)
module type AIRCRAFT =
  sig
    val init : int -> GPack.box -> unit
    val boot : Stdlib.value -> unit
    val commands : Stdlib.pprz_t array -> unit
    val infrared_and_airspeed : float -> float -> float -> float -> unit
    val attitude_and_rates : float -> float -> float -> float -> float -> float -> unit
    val gps : Gps.state -> unit
  end

(** A simulated aircraft, without its conf *)
module type AIRCRAFT_ITL =
    functor (A : Data.MISSION) -> functor (FM: FlightModel.SIG) -> AIRCRAFT

(** Functor to build the simulator *)
module Make :
  functor (AircraftItl : AIRCRAFT_ITL) ->
    sig
      val main : unit -> unit
    end
