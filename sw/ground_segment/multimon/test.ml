let print_hex = fun s ->
  for i = 0 to String.length s - 1 do
    Printf.printf "%02x" (Char.code s.[i])
  done;
  print_newline ()


let _ =
  let dsp = Demod.init "/dev/dsp" in

  let cb = fun _ ->
    let l, r = Demod.get_data () in
    print_hex l; print_hex r; true in

  ignore (Glib.Io.add_watch [`IN] cb (Glib.Io.channel_of_descr dsp));

  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do ignore (Glib.Main.iteration true) done
