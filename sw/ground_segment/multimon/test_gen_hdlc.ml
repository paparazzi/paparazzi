let my_write = fun fd buf ->
  let rec loop = fun i ->
    Printf.printf "i=%d\n%!" i;
    let r = String.length buf - i in 
    if r > 0   then
      loop (i + Unix.write fd buf i (min 8192 r)) in
  loop 0


let _ = 
  let fd = Hdlc.init_gen "/dev/dsp" in

  let i = ref 0 in
  let cb = fun _ ->
    incr i;
(***)
    Thread.create
      Hdlc.write_data
      (let s = Printf.sprintf "Cou [%d]" !i in prerr_endline s; s);
(***
    Thread.create (my_write fd) (String.create 22000);
***)
    true in

  ignore (Glib.Timeout.add 2000 cb);
(*   ignore (Glib.Timeout.add 100 (fun _ -> prerr_endline "x"; true));*)

  (** Threaded main loop (blocking write) *)
  GtkThread.main ()

