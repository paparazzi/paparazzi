open Stdlib

let _ = 
  Arg.parse
    (Sim.common_options@[set_string "-aircraft" Sim.ac_name "aircraft name";
			 set_string "-fbw" Hitl.tty1 "Fly by wire MCU port";
		      set_string "-ap" Hitl.tty0 "Autopilot MCU port"])
    (fun x -> Printf.fprintf stderr "Warning: Don't do anythig with %s\n" x)
    "Usage: ";

module M = Sim.Make(Hitl.Make)

let _ =
  M.main ()
