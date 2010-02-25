let _ =
  Ivyprobe.init ();
  let top = Tk.openTk () in
  try
    ignore (TkIvy.set_up_channel Unix.stdin Ivy.stop (fun _ -> Ivyprobe.read stdin));
    TkIvy.main ()
  with
    End_of_file -> Ivy.stop ()


