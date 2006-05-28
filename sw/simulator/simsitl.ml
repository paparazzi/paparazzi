 

let _ =
    Arg.parse (Sim.common_options@Sitl.options)
    (fun x -> Printf.fprintf stderr "Warning: Don't do anything with %s\n" x)
    "Usage: "


module M = Sim.Make(Sitl.Make)

let _ =
  M.main ()
