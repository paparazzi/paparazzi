(* $Id: glibivyprobe.ml,v 1.1 2004/10/18 10:55:37 brisset Exp $ *)

let _ =
  Ivyprobe.init ();
  try
    ignore (GlibIvy.set_up_channel Unix.stdin Ivy.stop (fun _ -> Ivyprobe.read stdin));
    GlibIvy.main ()
  with
    End_of_file -> Ivy.stop ()
