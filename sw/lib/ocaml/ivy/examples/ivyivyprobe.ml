let _ =
  Ivyprobe.init ();
  try
    ignore (IvyLoop.set_up_channel Unix.stdin Ivy.stop (fun _ -> Ivyprobe.read stdin));
    IvyLoop.main ()
  with
    End_of_file -> Ivy.stop ()

