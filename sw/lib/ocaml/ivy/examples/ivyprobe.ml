(* $Id: ivyprobe.ml,v 1.1 2004/10/18 10:55:37 brisset Exp $ *)

let print_message app message =
  Printf.printf "%s sent" (Ivy.name_of_client app);
  Array.iter (fun s -> Printf.printf " '%s'" s) message;
  print_newline ()

let read = fun channel ->
  let l = input_line channel in
  Ivy.send l

let watch_clients c e =
  let dis = match e with Ivy.Connected -> "" | Ivy.Disconnected -> "dis" in
  Printf.printf "%s %sconnected from %s\n"
    (Ivy.name_of_client c)
    dis
    (Ivy.host_of_client c);
  flush stdout

let init = fun () ->
  let regexp = ref ""
  and name = ref "MLIVYPROBE"
  and port = ref 2010
  and domain = ref "127.255.255.255" in
  Arg.parse
    [ "-b", Arg.Int (fun x -> port := x), "<Port number>\tDefault is 2010, unused if IVYBUS is set";
      "-domain", Arg.String (fun x -> domain := x), "<Network address>\tDefault is 127.255.255.255, unused if IVYBUS is set";
      "-n", Arg.String (fun s -> name := s), "<Name of the prober>\tDefault is MLIVYPROBE"]
    (fun s -> regexp := s)
    "Usage: ";

  let bus =
    try Sys.getenv "IVYBUS" with
      Not_found -> Printf.sprintf "%s:%d" !domain !port in
  Ivy.init !name "READY" watch_clients;
  Ivy.start bus;

  Printf.printf "\nEnd of file to stop\n\n"; flush stdout;

  ignore (Ivy.bind print_message !regexp)
