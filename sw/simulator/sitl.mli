(* Software In The Loop *)

module Make : functor (A : Data.MISSION) -> Sim.AIRCRAFT 
val options : (string * Arg.spec * string) list
(** Arg options specific to Sitl *)

