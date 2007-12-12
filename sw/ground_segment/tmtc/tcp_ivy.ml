open Printf

module Tm_Pprz = Pprz.Messages(struct let name = "telemetry" end)
module Dl_Pprz = Pprz.Messages(struct let name = "datalink" end)
module PprzTransport = Serial.Transport(Pprz.Transport)

let () =
  let ivy_bus = ref "127.255.255.255:2010" in
  let port = ref 4242 in

  let options = [
    "-b", Arg.Set_string ivy_bus, (sprintf "<ivy bus> Default is %s" !ivy_bus);
    "-p", Arg.Set_int port, (sprintf "<remote port> Default is %d" !port)
  ] in
  Arg.parse
    options
    (fun x -> fprintf stderr "Warning: Discarding '%s'" x)
    "Usage: ";

  let sockaddr = Unix.ADDR_INET (Unix.inet_addr_any, !port) in

  let handler = fun i o ->
    Ivy.init "tcp_ivy" "READY" (fun _ _ -> ());
    Ivy.start !ivy_bus;
    
    let buffer_size = 256 in
    let buffer = String.create buffer_size in
    let get_message = fun _ ->
      begin
	try
	  let n = input i buffer 0 buffer_size in
	  let b = String.sub buffer 0 n in
	  Debug.trace 'x' (Debug.xprint b);

	  let use_tele_message = fun payload ->
	    Debug.trace 'x' (Debug.xprint (Serial.string_of_payload payload));
	    let (msg_id, ac_id, values) = Tm_Pprz.values_of_payload payload in
	    let msg = Tm_Pprz.message_of_id msg_id in
	    Tm_Pprz.message_send (string_of_int ac_id) msg.Pprz.name values in

	  assert (PprzTransport.parse use_tele_message b = n)
	with
	  exc ->
	    prerr_endline (Printexc.to_string exc)
      end;
    true in
    
   let ginput = GMain.Io.channel_of_descr (Unix.descr_of_in_channel i) in
  ignore (Glib.Io.add_watch [`IN] get_message ginput);
    
    (* Forward datalink messages *)
    let get_ivy_message = fun _ args ->
      try
	let (msg_id, vs) = Dl_Pprz.values_of_string args.(0) in
	let ac_id = Pprz.int_assoc "ac_id" vs in
	let payload = Dl_Pprz.payload_of_values msg_id ac_id vs in
	let buf = Pprz.Transport.packet payload in
	fprintf o "%s%!" buf
      with exc -> prerr_endline (Printexc.to_string exc) in
    let _b = Ivy.bind get_ivy_message "^ground_dl (.*)" in


    let hangup = fun _ -> prerr_endline "hangup"; exit 1 in
    ignore (Glib.Io.add_watch [`HUP] hangup ginput);
    
    (* Main Loop *)
    GMain.main () in
  
  Unix.establish_server handler sockaddr
