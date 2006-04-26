open Stdlib

let _ = 
  Arg.parse
    (Sim.common_options@[set_string "-a" Sim.ac_name "aircraft name"])
    (fun x -> Printf.fprintf stderr "Warning: Don't do anythig with %s\n" x)
    "Usage: ";

module M = Sim.Make(Hitl.Make)

let _ =
  M.main ()
