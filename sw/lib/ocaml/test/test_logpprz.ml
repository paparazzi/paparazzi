module Tm_Pprz = PprzLink.Messages (struct let name = "telemetry" end)

module Parser = Protocol.Transport(Pprzlog_transport.Transport)

let convert_file = fun file ->
  let use_payload = fun payload ->
    let log_msg = Pprzlog_transport.parse payload in
    let (msg_id, ac_id, vs) =
      Tm_Pprz.values_of_payload log_msg.Pprzlog_transport.pprz_data in
    let msg_descr = Tm_Pprz.message_of_id msg_id in
    Printf.printf "%.3f %d %s\n" (Int32.to_float log_msg.Pprzlog_transport.timestamp /. 1e4) ac_id (Tm_Pprz.string_of_message msg_descr vs)  in

  let parser = Parser.parse use_payload in
  let Serial.Closure reader = Serial.input parser in

  let f = open_in file in
  try
    while true do
      reader (Unix.descr_of_in_channel f)
    done
  with
    End_of_file -> ()

let () =
  convert_file Sys.argv.(1)
