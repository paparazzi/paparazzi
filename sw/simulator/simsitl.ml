
(* don't set locale to avoid float_of_string problem *)
let locale = GtkMain.Main.init ~setlocale:false ()

let _ =
    Arg.parse (Sim.common_options@Sitl.options)
    (fun x -> Printf.fprintf stderr "Warning: Don't do anything with %s\n" x)
    "Usage: "


module M = Sim.Make(Sitl.Make)

let _ =
  M.main ()
