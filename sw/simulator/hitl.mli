val tty0 : string ref
val tty1 : string ref


module Make : functor (A : Data.MISSION) -> Sim.AIRCRAFT 
