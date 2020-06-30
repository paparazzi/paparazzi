
open Printf

let () =
  let ivy_bus = ref Defivybus.default_ivy_bus in

  let port = ref 4242
  and ivy_from = ref "DL"
  and ivy_to = ref "TM" in

  let options = [
    "-b", Arg.Set_string ivy_bus, (sprintf "<ivy bus> Default is %s" !ivy_bus);
    "-f", Arg.Set_string ivy_from, (sprintf "<tag> Default is %s" !ivy_from);
    "-t", Arg.Set_string ivy_to , (sprintf "<tag> Default is %s" !ivy_to);
    "-p", Arg.Set_int port, (sprintf "<remote port> Default is %d" !port)
  ] in
  Arg.parse
    options
    (fun x -> fprintf stderr "Warning: Discarding '%s'" x)
    "Usage: ";

  let handler = fun i o ->
    Ivy.init "broadcaster" "READY" (fun _ _ -> ());
    Ivy.start !ivy_bus;

    (* Forward telemetry on Ivy *)
    let buffer_size = 256 in
    let buffer = Bytes.create buffer_size in
    let get_tcp = fun _ ->
      begin
        try
          let n = input i buffer 0 buffer_size in
          let data = Bytes.sub buffer 0 n in

          Ivy.send (sprintf "%s %s" !ivy_to (Base64.encode_string (Bytes.to_string data)))
        with
            exc -> prerr_endline (Printexc.to_string exc)
      end;
      true in

    let ginput = GMain.Io.channel_of_descr (Unix.descr_of_in_channel i) in
    ignore (Glib.Io.add_watch [`IN] get_tcp ginput);
    let hangup = fun _ -> prerr_endline "hangup"; exit 1 in
    ignore (Glib.Io.add_watch [`HUP] hangup ginput);

    (* Forward datalink on Tcp *)
    let get_ivy = fun _ args ->
      try fprintf o "%s%!" (Base64.decode_string args.(0)) with
          exc -> prerr_endline (Printexc.to_string exc) in
    ignore (Ivy.bind get_ivy (sprintf "^%s (.*)" !ivy_from));

    (* Main Loop *)
    GMain.main ()
  in

  let sockaddr = Unix.ADDR_INET (Unix.inet_addr_any, !port) in
  Unix.establish_server handler sockaddr;
