module Tm_Pprz = Pprz.Messages (struct let name = "telemetry" end)

let to_binary = fun ac_id m ->
  let msg_id, vs = Tm_Pprz.values_of_string m in
  let payload = Tm_Pprz.payload_of_values msg_id ac_id vs in
  let buf = Pprz.Transport.packet payload in
  Printf.printf "From caml: %s\n%!" (Debug.xprint buf);
  buf

let _ = Callback.register "to_binary" to_binary
